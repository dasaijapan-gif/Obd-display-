"""Live CAN reader: python-can bus -> OBD polling + passive decoding -> ``RawState``.

Threads
-------
* **supervisor** (``obddash-can``): opens the bus (retrying every 5 s, never raising),
  attaches a ``can.Notifier``, starts the PID scheduler, and on a bus error tears
  everything down and reconnects.  ``stop()`` joins it.
* **notifier** (python-can): every received frame goes through :meth:`CanReader._on_frame`:
  OBD response ids -> :class:`~obddash.bus.obd.IsoTpAssembler` -> parse -> ``raw.update``;
  configured broadcast ids -> :class:`~obddash.bus.signals.SignalDecoder` -> ``raw.update``;
  everything else is just counted.  This path is kept allocation-light.
* **scheduler** (``obddash-obd``, only when ``obd_enabled``): discovers the supported
  PIDs (0x00, then 0x20... while the next-range bit is set), then polls one request at
  a time, waiting for its answer or ``obd_timeout_s``.  Three rate classes from the
  vehicle config: ``fast`` PIDs round-robin continuously, ``medium`` every ~250 ms,
  ``slow`` every ~2 s.  Medium/slow requests are interleaved with fast ones (never two
  in a row while fast PIDs exist) so the fast loop keeps at least half the bandwidth,
  and sends never exceed ``obd_max_rate_hz``.

Robustness rules
----------------
* A PID that does not answer three times in a row is throttled to one try per 5 s.
  That is what keeps the fast loop fast when the ECU ignores something, and what makes
  "poll the configured list anyway" affordable when discovery got no answer at all
  (discovery is then retried every 10 s).
* Multi-PID requests (``obd_multi_pid``) pack up to 6 fast PIDs per frame.  If two
  consecutive multi requests get no answer while the ECU is otherwise alive, the
  reader falls back to single-PID requests for the rest of the session.
* ``7F 01 78`` (response pending) extends the wait; other negative responses release
  the scheduler immediately and back the PID off.
* The only shared mutations are ``raw.set/update`` (locked inside RawState) and the
  stats fields, all written under one lock.  ``stats`` is refreshed on read so the
  rolling rates are always current.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import can

from obddash.bus import obd
from obddash.bus.signals import SignalDecoder
from obddash.config import BusConfig, VehicleConfig
from obddash.model import RawState

log = logging.getLogger(__name__)

RECONNECT_S = 5.0  # retry opening the interface this often
DISCOVERY_RETRY_S = 10.0  # retry supported-PID discovery when the ECU was silent
ECU_ALIVE_S = 2.0  # ecu_ok = a response within this long
MEDIUM_PERIOD_S = 0.25
SLOW_PERIOD_S = 2.0
PID_BACKOFF_AFTER = 3  # consecutive misses before a PID is throttled
PID_BACKOFF_S = 5.0
MULTI_FALLBACK_MISSES = 2  # unanswered multi-PID requests before single-PID fallback
RESPONSE_PENDING_S = 1.0  # extra wait after NRC 0x78
SEND_ERRORS_BEFORE_RECONNECT = 20


class RateMeter:
    """Events per second over a rolling ~1 s window with O(1) cost per event.

    Counts events in the current window; when the window is full the rate becomes
    ``count / elapsed`` and a new window starts.  ``rate()`` also rolls, so a source
    that stops produces 0 instead of a stale value.  During the very first window a
    provisional ``count / elapsed`` is returned once 0.2 s have passed.
    """

    __slots__ = ("_window", "_t0", "_count", "_rate", "_full")

    def __init__(self, window: float = 1.0, now: float = 0.0):
        self._window = window
        self._t0 = now
        self._count = 0
        self._rate = 0.0
        self._full = False

    def tick(self, now: float) -> None:
        self._roll(now)
        self._count += 1

    def rate(self, now: float) -> float:
        self._roll(now)
        if not self._full:
            dt = now - self._t0
            return self._count / dt if dt > 0.2 else 0.0
        return self._rate

    def _roll(self, now: float) -> None:
        dt = now - self._t0
        if dt >= self._window:
            if self._count == 0 and not self._full:
                self._t0 = now  # nothing seen yet: restart the window, no rate to report
                return
            self._rate = self._count / dt
            self._count = 0
            self._t0 = now
            self._full = True


@dataclass
class ReaderStats:
    bus_ok: bool = False  # interface open and sends working
    ecu_ok: bool = False  # an OBD response within the last 2 s
    frames: int = 0  # every frame received
    pid_responses: int = 0  # positive OBD responses
    frame_hz: float = 0.0  # rolling 1 s
    pid_hz: float = 0.0  # rolling 1 s
    supported_pids: set[int] = field(default_factory=set)
    unknown_ids: dict[int, int] = field(default_factory=dict)  # only if log_unknown_ids
    last_error: str | None = None
    vin: str | None = None
    # extras
    requests: int = 0
    timeouts: int = 0
    send_errors: int = 0
    discovered: bool = False  # supported-PID discovery got an answer
    multi_pid: bool = False  # multi-PID requests currently in use
    active_pids: list[int] = field(default_factory=list)  # what is being polled


@dataclass
class _Pending:
    """The one outstanding request the scheduler is waiting on."""

    mode: int
    pids: frozenset[int]
    sent_t: float
    answered: bool = False
    answered_pids: set[int] = field(default_factory=set)
    negative: int | None = None  # NRC, if the ECU refused
    extend: bool = False  # NRC 0x78 seen: wait longer


class _Listener(can.Listener):
    def __init__(self, reader: CanReader):
        self._reader = reader

    def on_message_received(self, msg: can.Message) -> None:
        self._reader._on_frame(msg)

    def on_error(self, exc: Exception) -> None:
        self._reader._on_bus_error(exc)

    def stop(self) -> None:  # called by Notifier.stop()
        pass


class CanReader:
    """Feed a :class:`~obddash.model.RawState` from a CAN bus.  See the module docstring.

    ``bus`` may be injected (tests, ``obddash sim``); otherwise the interface in
    ``bus_cfg`` is opened in the background.  :meth:`stop` shuts the bus down in both
    cases.
    """

    def __init__(
        self,
        bus_cfg: BusConfig,
        vehicle: VehicleConfig,
        raw: RawState,
        *,
        bus: can.BusABC | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.bus_cfg = bus_cfg
        self.vehicle = vehicle
        self.raw = raw
        self._clock = clock
        self._bus: can.BusABC | None = bus
        self._own_bus = bus is None

        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stats = ReaderStats(multi_pid=bus_cfg.obd_multi_pid)
        self._frame_meter = RateMeter(now=clock())
        self._pid_meter = RateMeter(now=clock())
        self._last_ecu_t = -1e9

        self._ext = bus_cfg.obd_extended_ids
        self._resp_ids = frozenset(bus_cfg.obd_response_ids)
        self._decoder = SignalDecoder(vehicle.signals) if bus_cfg.passive_enabled else None
        self._assembler = obd.IsoTpAssembler(on_first_frame=self._send_flow_control, clock=clock)
        self._listener = _Listener(self)
        self._notifier: can.Notifier | None = None

        self._stop = threading.Event()
        self._bus_error = threading.Event()
        self._resp = threading.Event()
        self._thread: threading.Thread | None = None
        self._open_failed_logged = False
        self._frame_errors = 0
        self._consecutive_send_errors = 0

        # scheduler state (only touched by the scheduler thread, except _pending)
        self._pending: _Pending | None = None
        self._multi = bus_cfg.obd_multi_pid
        self._multi_misses = 0
        self._discovered = False
        self._next_discovery = 0.0
        self._cfg_fast: list[int] = []
        self._cfg_medium: list[int] = []
        self._cfg_slow: list[int] = []
        self._fast: list[int] = []
        self._medium: list[int] = []
        self._slow: list[int] = []
        self._fast_groups: list[list[int]] = []
        self._rr = 0
        self._last_was_slow = False
        self._last_sent: dict[int, float] = {}
        self._misses: dict[int, int] = {}
        self._backoff: dict[int, float] = {}
        self._backoff_logged: set[int] = set()
        self._last_send_t = -1e9

    # ----------------------------------------------------------------------------------
    # public
    # ----------------------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="obddash-can", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._resp.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=5.0)
        self._thread = None
        # the supervisor normally does this; repeat for the never-started / stuck case
        self._detach()
        self._shutdown_bus()

    @property
    def stats(self) -> ReaderStats:
        self._refresh_stats(self._clock())
        return self._stats

    @property
    def bus(self) -> can.BusABC | None:
        return self._bus

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ----------------------------------------------------------------------------------
    # supervisor
    # ----------------------------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            bus = self._bus if self._bus is not None else self._open_bus()
            if bus is None:
                self._stop.wait(RECONNECT_S)
                continue
            self._bus_error.clear()
            if not self._attach(bus):
                self._shutdown_bus()
                if not self._own_bus:
                    break
                self._stop.wait(RECONNECT_S)
                continue
            sched: threading.Thread | None = None
            if self.bus_cfg.obd_enabled:
                sched = threading.Thread(
                    target=self._scheduler_loop, name="obddash-obd", daemon=True
                )
                sched.start()
            while not self._stop.is_set() and not self._bus_error.is_set():
                self._refresh_stats(self._clock())
                self._stop.wait(0.25)
            self._resp.set()  # wake the scheduler if it is waiting on a response
            if sched is not None:
                sched.join(timeout=3.0)
            self._detach()
            if self._stop.is_set():
                break
            # bus error path
            self._shutdown_bus()
            if not self._own_bus:
                log.error("injected CAN bus failed (%s); reader stopped", self._stats.last_error)
                break
            log.warning(
                "CAN bus error (%s); reopening in %.0f s", self._stats.last_error, RECONNECT_S
            )
            self._stop.wait(RECONNECT_S)
        self._detach()
        self._shutdown_bus()

    def _open_bus(self) -> can.BusABC | None:
        cfg = self.bus_cfg
        try:
            bus = can.Bus(
                interface=cfg.interface,
                channel=cfg.channel,
                bitrate=cfg.bitrate,
                receive_own_messages=cfg.receive_own_messages,
            )
        except Exception as exc:
            msg = f"cannot open CAN {cfg.interface}:{cfg.channel}: {exc}"
            if not self._open_failed_logged:
                log.warning("%s - retrying every %.0f s", msg, RECONNECT_S)
                self._open_failed_logged = True
            else:
                log.debug(msg)
            with self._lock:
                self._stats.bus_ok = False
                self._stats.last_error = msg
            return None
        self._open_failed_logged = False
        log.info("CAN bus open: %s:%s @ %d bit/s", cfg.interface, cfg.channel, cfg.bitrate)
        return bus

    def _attach(self, bus: can.BusABC) -> bool:
        self._bus = bus
        try:
            self._notifier = can.Notifier(bus, [self._listener], timeout=0.1)
        except Exception as exc:
            with self._lock:
                self._stats.bus_ok = False
                self._stats.last_error = f"cannot start notifier: {exc}"
            log.error("cannot start CAN notifier: %s", exc)
            return False
        self._consecutive_send_errors = 0
        with self._lock:
            self._stats.bus_ok = True
            self._stats.last_error = None
        return True

    def _detach(self) -> None:
        n, self._notifier = self._notifier, None
        if n is not None:
            try:
                n.stop(timeout=1.0)
            except Exception as exc:  # pragma: no cover - defensive
                log.debug("notifier stop: %s", exc)

    def _shutdown_bus(self) -> None:
        b, self._bus = self._bus, None
        if b is not None:
            try:
                b.shutdown()
            except Exception as exc:  # pragma: no cover - defensive
                log.debug("bus shutdown: %s", exc)
        with self._lock:
            self._stats.bus_ok = False

    def _on_bus_error(self, exc: Exception) -> None:
        with self._lock:
            self._stats.last_error = f"bus error: {exc}"
        if not self._bus_error.is_set():
            log.warning("CAN bus error in receive thread: %s", exc)
        self._bus_error.set()

    # ----------------------------------------------------------------------------------
    # receive path (notifier thread)
    # ----------------------------------------------------------------------------------
    def _is_obd_response(self, arbitration_id: int) -> bool:
        if arbitration_id in self._resp_ids:
            return True
        return self._ext and obd.is_obd_response_id(arbitration_id, True)

    def _on_frame(self, msg: can.Message) -> None:
        now = self._clock()
        with self._lock:
            self._stats.frames += 1
            self._frame_meter.tick(now)
        if msg.is_error_frame or msg.is_remote_frame:
            return
        arb = msg.arbitration_id
        try:
            if self._is_obd_response(arb):
                done = self._assembler.feed(arb, msg.data, now)
                if done is not None:
                    self._on_obd_payload(done[0], done[1], now)
                return
            if self._decoder is not None and arb in self._decoder.ids:
                values = self._decoder.decode(arb, msg.data)
                if values:
                    self.raw.update(values)
                return
            if self.bus_cfg.log_unknown_ids:
                with self._lock:
                    unk = self._stats.unknown_ids
                    if arb in unk or len(unk) < 1024:
                        unk[arb] = unk.get(arb, 0) + 1
        except Exception:
            self._frame_errors += 1
            if self._frame_errors == 1:
                log.exception("error handling frame 0x%X", arb)
            elif self._frame_errors % 1000 == 0:
                log.warning("%d frame handling errors so far", self._frame_errors)

    def _on_obd_payload(self, arb: int, payload: bytes, now: float) -> None:
        resp = obd.parse_response(arb, payload)
        if resp is None:
            return
        with self._lock:
            self._last_ecu_t = now
            self._stats.ecu_ok = True
            pending = self._pending
            if resp.negative is not None:
                log.debug(
                    "negative response from 0x%X: mode %02X nrc %02X", arb, resp.mode, resp.negative
                )
                if pending is not None and pending.mode == resp.mode:
                    if resp.negative == obd.NRC_RESPONSE_PENDING:
                        pending.extend = True
                    else:
                        pending.negative = resp.negative
                    self._resp.set()
                return
            self._stats.pid_responses += 1
            self._pid_meter.tick(now)
            for pids in resp.supported.values():
                self._stats.supported_pids |= pids
            if resp.vin and not self._stats.vin:
                self._stats.vin = resp.vin
                log.info("VIN %s", resp.vin)
            if pending is not None and pending.mode == resp.mode and (pending.pids & resp.pids):
                pending.answered = True
                pending.answered_pids |= resp.pids
                self._resp.set()
        if resp.values:
            self.raw.update({name: value for _pid, name, value in resp.values})

    # ----------------------------------------------------------------------------------
    # sending
    # ----------------------------------------------------------------------------------
    def _send(self, msg: can.Message) -> bool:
        bus = self._bus
        if bus is None:
            return False
        try:
            with self._send_lock:
                bus.send(msg, timeout=0.05)
        except Exception as exc:
            self._consecutive_send_errors += 1
            with self._lock:
                self._stats.send_errors += 1
                self._stats.last_error = f"send failed: {exc}"
                n = self._stats.send_errors
            if n == 1:
                log.warning("CAN send failed: %s (is the bus connected / terminated?)", exc)
            elif n % 100 == 0:
                log.warning("%d CAN send failures so far (last: %s)", n, exc)
            if self._consecutive_send_errors >= SEND_ERRORS_BEFORE_RECONNECT:
                with self._lock:
                    self._stats.bus_ok = False
                if self._own_bus:
                    self._bus_error.set()
            return False
        if self._consecutive_send_errors:
            self._consecutive_send_errors = 0
            with self._lock:
                self._stats.bus_ok = True
        return True

    def _send_flow_control(self, response_id: int) -> None:
        """Answer an ECU's first frame with ``30 00 00`` (called from the notifier thread)."""
        self._send(obd.build_flow_control(response_id, extended=self._ext))

    # ----------------------------------------------------------------------------------
    # stats
    # ----------------------------------------------------------------------------------
    def _refresh_stats(self, now: float) -> None:
        with self._lock:
            self._stats.frame_hz = self._frame_meter.rate(now)
            self._stats.pid_hz = self._pid_meter.rate(now)
            self._stats.ecu_ok = (now - self._last_ecu_t) < ECU_ALIVE_S

    # ----------------------------------------------------------------------------------
    # PID scheduler (its own thread)
    # ----------------------------------------------------------------------------------
    def _running(self) -> bool:
        return not (self._stop.is_set() or self._bus_error.is_set())

    def _sleep(self, seconds: float) -> None:
        if seconds > 0:
            self._stop.wait(seconds)

    def _scheduler_loop(self) -> None:
        self._prepare_pid_lists()
        if not (self._fast or self._medium or self._slow):
            log.info("no PIDs configured; OBD polling idle")
            return
        min_interval = 1.0 / max(self.bus_cfg.obd_max_rate_hz, 0.1)
        errors = 0
        while self._running():
            try:
                self._scheduler_step(min_interval)
            except Exception:
                errors += 1
                if errors <= 3:
                    log.exception("PID scheduler error")
                self._sleep(0.5)

    def _scheduler_step(self, min_interval: float) -> None:
        now = self._clock()
        if not self._discovered and now >= self._next_discovery:
            self._next_discovery = now + DISCOVERY_RETRY_S
            self._discovered = self._discover()
            self._rebuild_active()
            return
        pick = self._pick(now)
        if pick is None:
            self._sleep(0.05)
            return
        pids, multi = pick
        pending = self._request(pids)
        if pending is None:  # send failed
            self._sleep(0.1)
            return
        self._account(pids, pending, multi, now)
        # rate limit between consecutive requests
        wait = min_interval - (self._clock() - self._last_send_t)
        if wait > 0:
            self._sleep(wait)

    # -- PID lists ----------------------------------------------------------------------
    def _prepare_pid_lists(self) -> None:
        by_rate: dict[str, list[int]] = {"fast": [], "medium": [], "slow": []}
        seen: set[int] = set()
        for req in self.vehicle.pids:
            if req.pid in seen:
                continue
            seen.add(req.pid)
            if req.pid not in obd.PID_TABLE:
                log.warning("PID 0x%02X has no decoder in obd.PID_TABLE; not polling it", req.pid)
                continue
            by_rate[req.rate].append(req.pid)
        self._cfg_fast, self._cfg_medium, self._cfg_slow = (
            by_rate["fast"],
            by_rate["medium"],
            by_rate["slow"],
        )
        self._rebuild_active()

    def _rebuild_active(self) -> None:
        with self._lock:
            supported = set(self._stats.supported_pids)
        if self._discovered:

            def keep(p: int) -> bool:
                return p in supported

        else:

            def keep(p: int) -> bool:
                return True

        self._fast = [p for p in self._cfg_fast if keep(p)]
        self._medium = [p for p in self._cfg_medium if keep(p)]
        self._slow = [p for p in self._cfg_slow if keep(p)]
        skipped = [p for p in self._cfg_fast + self._cfg_medium + self._cfg_slow if not keep(p)]
        if self._multi:
            self._fast_groups = [
                self._fast[i : i + obd.MAX_PIDS_PER_REQUEST]
                for i in range(0, len(self._fast), obd.MAX_PIDS_PER_REQUEST)
            ]
        else:
            self._fast_groups = [[p] for p in self._fast]
        self._rr = 0
        with self._lock:
            self._stats.active_pids = self._fast + self._medium + self._slow
            self._stats.multi_pid = self._multi
            self._stats.discovered = self._discovered
        log.info(
            "polling %d fast / %d medium / %d slow PIDs%s%s",
            len(self._fast),
            len(self._medium),
            len(self._slow),
            " (multi-PID)" if self._multi else "",
            (" ; unsupported by ECU: " + ", ".join(f"0x{p:02X}" for p in skipped))
            if skipped
            else "",
        )

    # -- discovery ----------------------------------------------------------------------
    def _discover(self) -> bool:
        """Ask for the supported-PID bitmasks.  ~1 s worst case.  True if the ECU answered."""
        timeout = max(self.bus_cfg.obd_timeout_s, 0.3)
        answered = False
        for _attempt in range(2):
            if not self._running():
                return False
            p = self._request([0x00], timeout=timeout)
            if p is not None and p.answered:
                answered = True
                break
            if p is not None and p.negative is not None:
                break  # ECU is alive but refuses mode 01 PID 00: nothing to discover
        if not answered:
            log.info(
                "no answer to PID 0x00 discovery; polling configured PIDs, retrying in %.0f s",
                DISCOVERY_RETRY_S,
            )
            return False
        base = 0x20
        while base <= 0xC0 and self._running():
            with self._lock:
                more = obd.next_range_supported(base - 0x20, self._stats.supported_pids)
            if not more:
                break
            p = self._request([base], timeout=self.bus_cfg.obd_timeout_s)
            if p is None or not p.answered:
                break
            base += 0x20
        with self._lock:
            supported = sorted(self._stats.supported_pids)
            vin = self._stats.vin
        log.info(
            "ECU supports %d PIDs: %s", len(supported), " ".join(f"{p:02X}" for p in supported)
        )
        if vin is None and self._running():
            self._request([0x02], mode=0x09, timeout=0.5)  # VIN; multi-frame, exercises ISO-TP
        return True

    # -- request / response -------------------------------------------------------------
    def _request(
        self, pids: list[int], mode: int = 0x01, *, timeout: float | None = None
    ) -> _Pending | None:
        """Send one request and wait for its answer (or the timeout).  None if the send failed."""
        cfg = self.bus_cfg
        msg = obd.build_request(pids, mode, extended=self._ext, request_id=cfg.obd_request_id)
        now = self._clock()
        pending = _Pending(mode, frozenset(pids), now)
        with self._lock:
            self._pending = pending
            self._stats.requests += 1
        self._resp.clear()
        self._last_send_t = now
        if not self._send(msg):
            with self._lock:
                self._pending = None
            return None
        deadline = now + (cfg.obd_timeout_s if timeout is None else timeout)
        while self._running():
            remaining = deadline - self._clock()
            if remaining <= 0:
                break
            if not self._resp.wait(remaining):
                break
            self._resp.clear()
            with self._lock:
                if pending.answered or pending.negative is not None:
                    break
                if pending.extend:
                    pending.extend = False
                    deadline = self._clock() + RESPONSE_PENDING_S
        with self._lock:
            self._pending = None
            if not pending.answered and pending.negative is None:
                self._stats.timeouts += 1
        return pending

    def _account(self, pids: list[int], pending: _Pending, multi: bool, now: float) -> None:
        """Update miss counters / back-off after a request, and the multi-PID fallback."""
        ecu_alive = (now - self._last_ecu_t) < ECU_ALIVE_S
        for pid in pids:
            self._last_sent[pid] = now
            if pid in pending.answered_pids:
                self._misses[pid] = 0
                self._backoff.pop(pid, None)
                self._backoff_logged.discard(pid)
                continue
            if pending.negative is not None and not multi:
                self._misses[pid] = PID_BACKOFF_AFTER  # the ECU said no: don't insist
            else:
                self._misses[pid] = self._misses.get(pid, 0) + 1
            if self._misses[pid] >= PID_BACKOFF_AFTER:
                self._backoff[pid] = now + PID_BACKOFF_S
                if pid not in self._backoff_logged and ecu_alive:
                    self._backoff_logged.add(pid)
                    log.info("PID 0x%02X not answering; retrying every %.0f s", pid, PID_BACKOFF_S)
        if multi:
            if pending.answered:
                self._multi_misses = 0
            elif pending.negative is not None or ecu_alive:
                self._multi_misses += 1
                if pending.negative is not None or self._multi_misses >= MULTI_FALLBACK_MISSES:
                    log.info("ECU does not answer multi-PID requests; falling back to single PID")
                    self._multi = False
                    self._multi_misses = 0
                    for pid in pids:  # the group failing says nothing about single PIDs
                        self._misses[pid] = 0
                        self._backoff.pop(pid, None)
                    self._rebuild_active()

    def _pick(self, now: float) -> tuple[list[int], bool] | None:
        """Choose the next request: a due medium/slow PID, or the next fast group."""
        due: int | None = None
        due_t = 0.0
        for pids, period in ((self._medium, MEDIUM_PERIOD_S), (self._slow, SLOW_PERIOD_S)):
            for pid in pids:
                last = self._last_sent.get(pid, -1e9)
                if now - last >= period and now >= self._backoff.get(pid, 0.0):
                    if due is None or last < due_t:
                        due, due_t = pid, last
        fast = self._next_fast_group(now)
        if due is not None and (fast is None or not self._last_was_slow):
            self._last_was_slow = True
            return [due], False
        if fast is not None:
            idx, group = fast
            self._rr = (idx + 1) % len(self._fast_groups)
            self._last_was_slow = False
            return group, len(group) > 1
        return None

    def _next_fast_group(self, now: float) -> tuple[int, list[int]] | None:
        n = len(self._fast_groups)
        for k in range(n):
            idx = (self._rr + k) % n
            group = [p for p in self._fast_groups[idx] if now >= self._backoff.get(p, 0.0)]
            if group:
                return idx, group
        return None
