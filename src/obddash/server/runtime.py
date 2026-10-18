"""Runtime: wires a data source, the metrics engine and the WebSocket fan-out.

::

    source threads ──▶ RawState ──▶ engine.tick() ──▶ latest Telemetry ──▶ JSON once ──▶ client queues
    (CanReader, Imu,              (event loop,                          (event loop,     (one per /ws
     Simulator, Replayer)          metrics.tick_hz)                      server.ws_rate_hz) connection)

Sources (``source=``)
---------------------
* ``"can"``: :class:`~obddash.bus.reader.CanReader` on ``cfg.bus`` (or an injected
  ``bus``), plus the MPU6050 :class:`~obddash.sensors.imu.Imu` when ``cfg.imu.enabled``.
* ``"sim"``: two python-can ``virtual`` buses on one channel, one for the
  :class:`~obddash.sim.Simulator` (fake car + ECU), one for a ``CanReader``, so the sim
  exercises the real OBD / signal decoding path.  No IMU: the sim broadcasts ``acc_*``.
* ``"replay"``: a :class:`~obddash.server.recorder.Replayer` thread feeds RawState
  from a JSONL recording.
* ``"none"``: nothing; the engine still ticks so the UI gets frames saying "no link".

Optionally a :class:`~obddash.server.recorder.Recorder` samples RawState every tick.

Timing
------
Two asyncio tasks run on the server's event loop.  Both use a drift-free schedule:
deadlines advance by exactly one period on a monotonic clock, so sleep jitter never
accumulates.  A slot that is late by less than a period runs immediately; when the
loop falls further behind (GC pause, slow SD card) the missed slots are *skipped*
(counted in ``stats.skipped_ticks``) rather than run in a burst.

* tick loop (``metrics.tick_hz``, 50 Hz): build the :class:`~obddash.model.Connection`
  from reader / IMU / replay state, ``engine.set_connection()``, ``engine.tick(raw, t)``
  with ``t = time.monotonic()``, keep the result as ``latest``, ``recorder.sample()``.
* broadcast loop (``server.ws_rate_hz``, 30 Hz): drain engine events and send each as
  ``{"type": "event", ...}``; if there is a new snapshot, serialise
  ``{"type": "telemetry", "data": latest.to_dict()}`` ONCE and put the same string in
  every client queue.  Nothing is serialised while no client is connected.

Slow clients
------------
Each client queue holds at most 2 messages.  Publishing never blocks: when a queue
is full its oldest message is discarded (``stats.dropped``), so a stalled browser
just sees fewer frames and can never back-pressure the tick loop.

Threading
---------
Producers write RawState from their own threads (RawState is locked).  Everything
else - ``engine.tick``, ``engine.command``, ``engine.drain_events``, the client queues
and the cached JSON - is touched only from the event-loop thread.  The FastAPI
handlers are ``async def`` for that reason.  ``stop()`` may be called from any thread.

Failure handling
----------------
An exception in ``engine.tick`` is logged (first one with traceback, then every 500th)
and the previous snapshot is kept; the loop keeps running.  A failing recorder is
closed and recording stops; it never takes the tick loop down.

Packages that may be absent or still in development (bus, sim, metrics, sensors) are
imported lazily, inside the functions that need them.
"""

from __future__ import annotations

import asyncio
import dataclasses
import json
import logging
import math
import time
from pathlib import Path
from typing import Any

from obddash.config import AppConfig
from obddash.model import Connection, Event, RawState, Telemetry
from obddash.server.recorder import Recorder, Replayer, make_recording_path

log = logging.getLogger(__name__)

SOURCES = ("can", "sim", "replay", "none")
CLIENT_QUEUE_SIZE = 2
DEFAULT_SIM_CHANNEL = "obddash-sim"
HEALTHY_TICK_AGE_S = 1.0  # /api/health "ok" needs a tick within this long
_JSON_SEPARATORS = (",", ":")


def _default_engine(cfg: AppConfig) -> Any:
    """The real :class:`MetricsEngine`, imported only when a Runtime needs it."""
    try:
        from obddash.metrics.engine import MetricsEngine
    except ModuleNotFoundError as exc:
        if exc.name != "obddash.metrics.engine":
            raise
        from obddash.metrics import MetricsEngine  # type: ignore[attr-defined,no-redef]
    return MetricsEngine(cfg.metrics, cfg.vehicle)


def _jsonable(obj: Any) -> Any:
    """Event payloads come from the engine and may hold dataclasses, tuples, sets or
    NaN; turn them into strict-JSON-safe values (floats rounded like Telemetry)."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        obj = dataclasses.asdict(obj)
    if isinstance(obj, dict):
        return {("lambda" if k == "lambda_" else str(k)): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list | tuple | set | frozenset):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, float):
        return round(obj, 4) if math.isfinite(obj) else None
    if obj is None or isinstance(obj, bool | int | str):
        return obj
    return str(obj)


def _next_deadline(deadline: float, period: float, now: float) -> tuple[float, int]:
    """Advance a periodic deadline by one period on a fixed grid.

    A slot that is late by less than one period is kept (the caller runs it right
    away), so ordinary jitter never costs a tick.  When the loop is a period or more
    behind, the slots that can no longer run on time are skipped instead of being
    run back to back.  Returns ``(deadline, slots skipped)``.
    """
    deadline += period
    if now < deadline + period:
        return deadline, 0
    missed = int((now - deadline) // period)
    return deadline + missed * period, missed


@dataclasses.dataclass
class RuntimeStats:
    ticks: int = 0
    tick_errors: int = 0
    skipped_ticks: int = 0  # tick slots skipped because the loop was behind
    tick_ms: float = 0.0  # moving average of one tick's cost (engine + recorder)
    tick_hz: float = 0.0  # measured tick rate (moving average)
    broadcasts: int = 0  # broadcast cycles that had at least one client
    events: int = 0  # engine events published
    dropped: int = 0  # messages discarded for slow clients
    last_tick_t: float | None = None  # monotonic time of the last tick


class Runtime:
    """Owns RawState, the metrics engine, the data source threads, the latest
    Telemetry and the client broadcast hub.  Single use: ``start()`` once, ``stop()``
    once (both idempotent).

    ``engine`` replaces the real MetricsEngine (tests); anything with ``tick``,
    ``command``, ``drain_events`` and ``set_connection`` works.  ``sim_channel`` names
    the virtual CAN channel of the ``sim`` source; give each Runtime in a process its
    own so two simulators never share a bus.
    """

    def __init__(
        self,
        cfg: AppConfig,
        *,
        source: str,
        bus: Any = None,
        replay_path: str | Path | None = None,
        sim_script: str = "lap",
        sim_seed: int = 1,
        engine: Any = None,
        sim_channel: str = DEFAULT_SIM_CHANNEL,
        replay_speed: float = 1.0,
        replay_loop: bool = False,
    ) -> None:
        if source not in SOURCES:
            raise ValueError(f"unknown source {source!r}; choose from {', '.join(SOURCES)}")
        if source == "replay" and replay_path is None:
            raise ValueError("source 'replay' needs replay_path")
        self.cfg = cfg
        self.vehicle = cfg.vehicle
        self.source = source
        self.sim_script = sim_script
        self.sim_seed = sim_seed
        self.sim_channel = sim_channel
        self.replay_path = Path(replay_path) if replay_path is not None else None
        self.replay_speed = replay_speed
        self.replay_loop = replay_loop
        self._bus = bus

        self.raw = RawState()
        self.engine = engine if engine is not None else _default_engine(cfg)
        self.latest = Telemetry(conn=Connection(source=source))
        self.stats = RuntimeStats()

        # data sources (created by start())
        self.reader: Any = None
        self.imu: Any = None
        self.sim: Any = None
        self.replayer: Replayer | None = None
        self.recorder: Recorder | None = None
        self._sim_buses: list[Any] = []

        # broadcast hub + caches (event-loop thread only)
        self._subs: set[asyncio.Queue[str]] = set()
        self._seq = 0  # bumps on every new `latest`
        self._dict_seq = -1
        self._dict: dict[str, Any] = {}
        self._json_seq = -1
        self._json = ""
        self._sent_seq = -1
        self._conn = self.latest.conn

        # lifecycle
        self.started_at: float | None = None
        self._started = False
        self._stop_requested = False
        self._running = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._tasks: list[asyncio.Task[None]] = []

    # ----------------------------------------------------------------------------------
    # lifecycle
    # ----------------------------------------------------------------------------------
    def start(self) -> None:
        """Start the data source (and recorder).  Sync; returns quickly.  On failure
        everything started so far is stopped again and the exception propagates.

        A Runtime that has been stopped never starts again: threads started after
        ``stop()`` would have no owner left to stop them (``run()`` returns at once)."""
        if self._started:
            return
        if self._stop_requested:
            log.debug("runtime already stopped; not starting the %s source", self.source)
            return
        self._started = True
        self.started_at = time.monotonic()
        try:
            if self.source == "can":
                self._start_can()
            elif self.source == "sim":
                self._start_sim()
            elif self.source == "replay":
                self._start_replay()
            self._start_recorder()
        except Exception:
            self.stop()
            raise
        if self._stop_requested:
            # stop() ran on another thread while we were starting: it may have missed
            # what was created after it looked, so stop again (every stop is idempotent)
            self.stop()
            return
        log.info("runtime started: source=%s vehicle=%s", self.source, self.vehicle.name)

    async def run(self) -> None:
        """Run the tick and broadcast loops until :meth:`stop` (returns normally) or
        until this coroutine's task is cancelled (re-raises CancelledError)."""
        if not self._started:
            self.start()
        if self._stop_requested:
            return
        self._loop = asyncio.get_running_loop()
        self._tasks = [
            asyncio.create_task(self._tick_loop(), name="obddash-tick"),
            asyncio.create_task(self._broadcast_loop(), name="obddash-broadcast"),
        ]
        self._running = True
        try:
            await asyncio.gather(*self._tasks)
        except asyncio.CancelledError:
            task = asyncio.current_task()
            if not (self._stop_requested and task is not None and not task.cancelling()):
                raise
        finally:
            self._running = False
            for t in self._tasks:
                t.cancel()
            await asyncio.gather(*self._tasks, return_exceptions=True)
            self._tasks = []

    def stop(self) -> None:
        """Stop the loops and every source thread, close the recorder.  Idempotent,
        callable from any thread, never raises."""
        self._stop_requested = True
        self._cancel_tasks()
        # order: sensors and the fake car first, then the reader, then the shared
        # virtual buses (the sim does not own its bus; the runtime does)
        for what, obj in (
            ("imu", self.imu),
            ("simulator", self.sim),
            ("reader", self.reader),
            ("replayer", self.replayer),
        ):
            if obj is None:
                continue
            try:
                obj.stop()
            except Exception:
                log.exception("stopping %s failed", what)
        buses, self._sim_buses = self._sim_buses, []
        for bus in buses:
            try:
                bus.shutdown()
            except Exception:
                log.debug("virtual bus shutdown failed", exc_info=True)
        self._close_recorder()

    def _cancel_tasks(self) -> None:
        tasks = list(self._tasks)
        loop = self._loop
        if not tasks or loop is None or loop.is_closed():
            return
        try:
            current = asyncio.get_running_loop()
        except RuntimeError:
            current = None
        if current is loop:
            for t in tasks:
                t.cancel()
        else:
            try:
                loop.call_soon_threadsafe(lambda: [t.cancel() for t in tasks])
            except RuntimeError:  # loop closed in the meantime
                pass

    @property
    def running(self) -> bool:
        return self._running

    # -- sources --------------------------------------------------------------------
    def _start_can(self) -> None:
        from obddash.bus.reader import CanReader

        self.reader = CanReader(self.cfg.bus, self.vehicle, self.raw, bus=self._bus)
        self.reader.start()
        if self.cfg.imu.enabled:
            self._start_imu()

    def _start_imu(self) -> None:
        try:
            from obddash.sensors.imu import Imu
        except ImportError as exc:
            log.warning("IMU enabled but the IMU driver is unavailable (%s); no IMU", exc)
            return
        try:
            imu = Imu(self.cfg.imu, self.raw)
            ok = imu.start()
        except Exception:
            log.exception("IMU start failed; continuing without IMU")
            return
        self.imu = imu
        if not ok:
            log.warning("IMU not detected; G-forces will come from CAN signals or estimates")

    def _start_sim(self) -> None:
        import can

        from obddash.bus.reader import CanReader
        from obddash.sim import Simulator

        channel = self.sim_channel
        sim_bus = can.Bus(interface="virtual", channel=channel)
        self._sim_buses.append(sim_bus)
        app_bus = can.Bus(interface="virtual", channel=channel)
        self._sim_buses.append(app_bus)
        self.sim = Simulator(
            self.vehicle, sim_bus, script=self.sim_script, seed=self.sim_seed, realtime=True
        )
        # the fake ECU speaks standard 11-bit OBD on 0x7DF -> 0x7E8, whatever the real car does
        bus_cfg = self.cfg.bus.model_copy(
            update={
                "interface": "virtual",
                "channel": channel,
                "obd_extended_ids": False,
                "obd_request_id": 0x7DF,
                "obd_response_ids": list(range(0x7E8, 0x7F0)),
            }
        )
        self.reader = CanReader(bus_cfg, self.vehicle, self.raw, bus=app_bus)
        self.sim.start()
        self.reader.start()

    def _start_replay(self) -> None:
        assert self.replay_path is not None
        self.replayer = Replayer(
            self.replay_path, self.raw, speed=self.replay_speed, loop=self.replay_loop
        )
        self.replayer.start()

    def _start_recorder(self) -> None:
        rc = self.cfg.recording
        if not rc.enabled:
            return
        if self.source == "replay":
            log.info("recording is off while replaying")
            return
        vehicle_file = self.cfg.vehicle_file
        meta = {
            "vehicle": self.vehicle.name,
            # absolute: `obddash replay` reopens it, possibly from another directory
            # (load_config leaves it relative when the config path was relative)
            "vehicle_file": str(Path(vehicle_file).resolve()) if vehicle_file else None,
            "source": self.source,
            "tick_hz": self.cfg.metrics.tick_hz,
        }
        if self.source == "sim":
            meta.update(sim_script=self.sim_script, sim_seed=self.sim_seed)
        path = make_recording_path(rc.dir)
        try:
            self.recorder = Recorder(path, meta=meta)
        except OSError as exc:
            log.error("cannot record to %s: %s; recording off", path, exc)
            return
        log.info("recording raw signals to %s", path)

    def _close_recorder(self) -> None:
        rec, self.recorder = self.recorder, None
        if rec is not None:
            try:
                rec.close()
            except Exception:
                log.exception("closing recorder failed")

    # ----------------------------------------------------------------------------------
    # tick
    # ----------------------------------------------------------------------------------
    def connection(self) -> Connection:
        """Link status for the UI, from the reader / IMU / replayer state."""
        conn = Connection(source=self.source)
        reader = self.reader
        if reader is not None:
            st = reader.stats
            conn.bus = bool(st.bus_ok)
            conn.ecu = bool(st.ecu_ok)
            conn.obd = bool(getattr(getattr(reader, "bus_cfg", None), "obd_enabled", False))
            conn.pid_hz = round(float(st.pid_hz), 1)
            conn.frame_hz = round(float(st.frame_hz), 1)
        elif self.replayer is not None:
            # a replay "link" is up while lines are being played
            conn.bus = conn.ecu = self.replayer.running
        imu = self.imu
        conn.imu = bool(imu is not None and getattr(imu, "available", False))
        return conn

    def step(self, t: float | None = None) -> Telemetry:
        """One tick: connection -> engine -> latest -> recorder.  The tick loop calls
        this; tests may call it directly.  Never raises."""
        now = time.monotonic() if t is None else t
        c0 = time.perf_counter()
        try:
            conn = self.connection()
        except Exception:
            log.debug("building connection status failed", exc_info=True)
            conn = Connection(source=self.source)
        self._conn = conn
        tel: Telemetry | None = None
        try:
            self.engine.set_connection(conn)
            tel = self.engine.tick(self.raw, now)
        except Exception:
            self.stats.tick_errors += 1
            n = self.stats.tick_errors
            if n == 1 or n % 500 == 0:
                log.exception("metrics tick failed (%d so far); keeping the last snapshot", n)
        if tel is not None:
            self.latest = tel
            self._seq += 1
        rec = self.recorder
        if rec is not None:
            try:
                rec.sample(self.raw, now)
            except Exception:
                log.exception("recorder failed; recording stopped")
                self._close_recorder()
        self._account_tick(now, (time.perf_counter() - c0) * 1000.0)
        return self.latest

    def _account_tick(self, now: float, cost_ms: float) -> None:
        st = self.stats
        st.ticks += 1
        st.tick_ms = cost_ms if st.ticks == 1 else st.tick_ms + 0.05 * (cost_ms - st.tick_ms)
        if st.last_tick_t is not None:
            dt = now - st.last_tick_t
            if dt > 0:
                hz = 1.0 / dt
                st.tick_hz = hz if st.tick_hz == 0 else st.tick_hz + 0.05 * (hz - st.tick_hz)
        st.last_tick_t = now

    async def _tick_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.cfg.metrics.tick_hz))
        deadline = time.monotonic()
        while not self._stop_requested:
            self.step()
            deadline, missed = _next_deadline(deadline, period, time.monotonic())
            self.stats.skipped_ticks += missed
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    # ----------------------------------------------------------------------------------
    # broadcast
    # ----------------------------------------------------------------------------------
    async def _broadcast_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.cfg.server.ws_rate_hz))
        deadline = time.monotonic()
        while not self._stop_requested:
            try:
                self.broadcast_once()
            except Exception:
                log.exception("broadcast failed")
            deadline, _missed = _next_deadline(deadline, period, time.monotonic())
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    def broadcast_once(self) -> int:
        """Publish pending events and the newest snapshot to every client queue.
        Returns the number of messages published (per client)."""
        try:
            events = self.engine.drain_events() or []
        except Exception:
            log.exception("engine.drain_events failed")
            events = []
        if not self._subs:
            return 0  # nobody listening: events are moments, not state; let them go
        sent = 0
        for ev in events:
            msg = self._event_json(ev)
            if msg is not None:
                self.publish(msg)
                self.stats.events += 1
                sent += 1
        if self._seq != self._sent_seq:
            self._sent_seq = self._seq
            self.publish(self.telemetry_json())
            sent += 1
        self.stats.broadcasts += 1
        return sent

    def publish(self, msg: str) -> None:
        """Put one message in every client queue without ever blocking: a full
        queue loses its oldest message."""
        for q in tuple(self._subs):
            if q.full():
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
                self.stats.dropped += 1
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                self.stats.dropped += 1

    def subscribe(self) -> asyncio.Queue[str]:
        q: asyncio.Queue[str] = asyncio.Queue(maxsize=CLIENT_QUEUE_SIZE)
        self._subs.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue[str]) -> None:
        self._subs.discard(q)

    @property
    def client_count(self) -> int:
        return len(self._subs)

    # -- serialisation (cached per snapshot) ------------------------------------------
    def snapshot_dict(self) -> dict[str, Any]:
        """``latest.to_dict()``, computed at most once per snapshot.  Treat as
        read-only: the same dict is handed to every caller until the next tick."""
        if self._dict_seq != self._seq:
            try:
                self._dict = self.latest.to_dict()
            except Exception:
                log.exception("Telemetry.to_dict failed")
                self._dict = Telemetry(conn=Connection(source=self.source)).to_dict()
            self._dict_seq = self._seq
        return self._dict

    def telemetry_json(self) -> str:
        """The ``{"type": "telemetry", "data": ...}`` message, serialised once per snapshot."""
        if self._json_seq != self._seq:
            msg = {"type": "telemetry", "data": self.snapshot_dict()}
            try:
                self._json = json.dumps(msg, separators=_JSON_SEPARATORS, allow_nan=False)
            except (TypeError, ValueError):
                # something non-JSON slipped into the snapshot; sanitise rather than drop
                log.debug("telemetry not strict JSON; sanitising", exc_info=True)
                self._json = json.dumps(_jsonable(msg), separators=_JSON_SEPARATORS)
            self._json_seq = self._seq
        return self._json

    def _event_json(self, ev: Any) -> str | None:
        if isinstance(ev, Event):
            name, data, t = ev.name, ev.data, ev.t
        elif isinstance(ev, dict):
            name, data, t = ev.get("name"), ev.get("data", {}), ev.get("t", 0.0)
        else:
            name, data, t = getattr(ev, "name", None), getattr(ev, "data", {}), getattr(ev, "t", 0)
        if not name:
            log.debug("dropping event without a name: %r", ev)
            return None
        msg = {"type": "event", "name": str(name), "data": _jsonable(data), "t": _jsonable(t)}
        return json.dumps(msg, separators=_JSON_SEPARATORS)

    # ----------------------------------------------------------------------------------
    # commands & status
    # ----------------------------------------------------------------------------------
    def command(self, cmd: str, args: dict[str, Any] | None = None) -> bool:
        """Forward a UI command (``dyno.arm``, ``session.reset``, ...) to the engine.
        Call from the event-loop thread.  Never raises."""
        if not isinstance(cmd, str) or not cmd:
            return False
        try:
            ok = bool(self.engine.command(cmd, dict(args or {})))
        except Exception:
            log.exception("command %r failed", cmd)
            return False
        log.info("command %s -> %s", cmd, "ok" if ok else "rejected")
        return ok

    def health(self) -> dict[str, Any]:
        """Payload of ``GET /api/health``."""
        now = time.monotonic()
        conn = self._conn
        st = self.stats
        fresh = st.last_tick_t is not None and (now - st.last_tick_t) < HEALTHY_TICK_AGE_S
        out: dict[str, Any] = {
            "ok": bool(self._running and fresh),
            "source": self.source,
            "bus": conn.bus,
            "ecu": conn.ecu,
            "imu": conn.imu,
            "pid_hz": conn.pid_hz,
            "frame_hz": conn.frame_hz,
            "uptime_s": round(now - self.started_at, 1) if self.started_at is not None else 0.0,
            "clients": len(self._subs),
            "ticks": st.ticks,
            "tick_hz": round(st.tick_hz, 1),
            "tick_ms": round(st.tick_ms, 2),
            "tick_errors": st.tick_errors,
            "skipped_ticks": st.skipped_ticks,
            "dropped": st.dropped,
            "recording": str(self.recorder.path) if self.recorder is not None else None,
        }
        reader = self.reader
        if reader is not None:
            try:
                rs = reader.stats
                out["last_error"] = rs.last_error
                out["vin"] = rs.vin
            except Exception:
                log.debug("reader stats unavailable", exc_info=True)
        rp = self.replayer
        if rp is not None:
            out["replay"] = {
                "file": str(rp.path),
                "position_s": round(rp.position, 2),
                "duration_s": round(rp.duration, 2),
                "speed": rp.speed,
                "finished": rp.finished,
            }
        return out
