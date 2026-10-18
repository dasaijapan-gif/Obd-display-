"""Bus discovery tooling behind ``obddash scan``.

Finding the brake-pressure / steering / wheel-speed frames of an unknown car is a
"watch what moves" job: park, note what the bus looks like at rest, then press the
brake and see which bytes change.  :class:`BusScanner` keeps, per arbitration id, the
frame count, rate, the last payload, per-byte change counters and which bytes changed
within the last 2 s, and renders that as a terminal table with the recently changed
bytes marked ``[xx]``.

Diff mode is the structured version of the same workflow: :meth:`BusScanner.baseline`
records every value each byte takes while the car is at rest (counters and checksums
cycle through many values, a parked brake-pressure byte stays at one).  Afterwards
:meth:`BusScanner.changes_since_baseline` lists the bytes that took values never seen
during the baseline, ranked so the "quiet in baseline, moved afterwards" bytes come
first; those are the candidates to put in the vehicle YAML.

:func:`run_scan` wires a scanner to a python-can bus and redraws the table every
``interval`` seconds until ``duration`` has passed or Ctrl-C.  Output goes through
``print`` on purpose: this is the one CLI-facing module of the bus package.
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TextIO

import can

from obddash.bus.reader import RateMeter

log = logging.getLogger(__name__)

RECENT_CHANGE_S = 2.0
MAX_BYTES = 8  # classic CAN; longer (FD) payloads are tracked up to this many bytes in the table
CLEAR_SCREEN = "\x1b[2J\x1b[H"


@dataclass
class IdStats:
    arbitration_id: int
    extended: bool = False
    count: int = 0
    hz: float = 0.0
    last: bytes = b""
    first_t: float = 0.0
    last_t: float = 0.0
    byte_changes: list[int] = field(default_factory=lambda: [0] * MAX_BYTES)
    byte_last_change_t: list[float] = field(default_factory=lambda: [-1e9] * MAX_BYTES)
    meter: RateMeter = field(default_factory=RateMeter, repr=False)

    def changed_mask(self, now: float) -> int:
        """Bitmask of bytes (bit i = byte i) that changed within the last 2 s."""
        mask = 0
        for i, t in enumerate(self.byte_last_change_t):
            if now - t <= RECENT_CHANGE_S:
                mask |= 1 << i
        return mask

    def label(self) -> str:
        return f"{self.arbitration_id:08X}" if self.extended else f"{self.arbitration_id:03X}"


@dataclass
class ByteChange:
    """One byte that took values outside its baseline set."""

    arbitration_id: int
    byte_index: int
    baseline_values: set[int]
    new_values: set[int]
    count: int  # frames in which the byte was outside the baseline
    last_value: int
    new_id: bool = False  # the id never appeared during the baseline


class BusScanner:
    """Collect per-id statistics from frames.  Feed frames yourself (:meth:`feed`) or
    attach a bus (:meth:`start`/:meth:`stop`)."""

    def __init__(
        self, bus: can.BusABC | None = None, *, clock: Callable[[], float] = time.monotonic
    ):
        self.bus = bus
        self._clock = clock
        self._lock = threading.Lock()
        self.ids: dict[int, IdStats] = {}
        self._notifier: can.Notifier | None = None
        self._start_t = clock()
        self.frames = 0
        # diff mode
        self._baselining = False
        self._baseline: dict[int, list[set[int]]] | None = None
        self._moved: dict[tuple[int, int], ByteChange] = {}

    # -- lifecycle ----------------------------------------------------------------------
    def start(self) -> None:
        if self.bus is None or self._notifier is not None:
            return
        self._start_t = self._clock()
        self._notifier = can.Notifier(self.bus, [self._on_message], timeout=0.1)

    def stop(self) -> None:
        n, self._notifier = self._notifier, None
        if n is not None:
            n.stop(timeout=1.0)

    def _on_message(self, msg: can.Message) -> None:
        if msg.is_error_frame:
            return
        self.feed(msg.arbitration_id, bytes(msg.data), extended=msg.is_extended_id)

    # -- core ---------------------------------------------------------------------------
    def feed(
        self, arbitration_id: int, data: bytes, now: float | None = None, *, extended: bool = False
    ) -> None:
        if now is None:
            now = self._clock()
        with self._lock:
            self.frames += 1
            st = self.ids.get(arbitration_id)
            if st is None:
                st = IdStats(arbitration_id, extended, first_t=now, meter=RateMeter(now=now))
                self.ids[arbitration_id] = st
            st.count += 1
            st.last_t = now
            st.meter.tick(now)
            prev = st.last
            if st.count > 1:
                for i in range(min(len(data), MAX_BYTES)):
                    if i >= len(prev) or data[i] != prev[i]:
                        st.byte_changes[i] += 1
                        st.byte_last_change_t[i] = now
            st.last = data
            if self._baselining and self._baseline is not None:
                sets = self._baseline.setdefault(arbitration_id, [set() for _ in range(MAX_BYTES)])
                for i in range(min(len(data), MAX_BYTES)):
                    sets[i].add(data[i])
            elif self._baseline is not None:
                self._diff_frame(arbitration_id, data)

    def _diff_frame(self, arbitration_id: int, data: bytes) -> None:
        base = self._baseline.get(arbitration_id) if self._baseline is not None else None
        for i in range(min(len(data), MAX_BYTES)):
            v = data[i]
            if base is not None and v in base[i]:
                continue
            key = (arbitration_id, i)
            ch = self._moved.get(key)
            if ch is None:
                ch = ByteChange(
                    arbitration_id,
                    i,
                    set(base[i]) if base is not None else set(),
                    set(),
                    0,
                    v,
                    new_id=base is None,
                )
                self._moved[key] = ch
            ch.count += 1
            ch.last_value = v
            if len(ch.new_values) < 64:
                ch.new_values.add(v)

    # -- diff mode ------------------------------------------------------------------------
    def begin_baseline(self) -> None:
        with self._lock:
            self._baseline = {}
            self._moved = {}
            self._baselining = True

    def end_baseline(self) -> None:
        with self._lock:
            self._baselining = False

    def baseline(self, seconds: float) -> None:
        """Blocking: record the resting state of the bus for ``seconds`` (needs :meth:`start`)."""
        self.begin_baseline()
        time.sleep(max(0.0, seconds))
        self.end_baseline()

    @property
    def has_baseline(self) -> bool:
        return self._baseline is not None and not self._baselining

    def changes_since_baseline(self) -> list[ByteChange]:
        """Bytes that moved outside their baseline values, most suspicious first:
        few distinct baseline values (quiet while parked), then most frames changed."""
        with self._lock:
            changes = list(self._moved.values())
        changes.sort(
            key=lambda c: (
                c.new_id,
                len(c.baseline_values),
                -c.count,
                c.arbitration_id,
                c.byte_index,
            )
        )
        return changes

    # -- rendering --------------------------------------------------------------------------
    def snapshot(self, now: float | None = None) -> list[IdStats]:
        """Sorted copy of the id table with ``hz`` refreshed."""
        if now is None:
            now = self._clock()
        with self._lock:
            out = []
            for st in self.ids.values():
                st.hz = st.meter.rate(now)
                out.append(st)
            return sorted(out, key=lambda s: s.arbitration_id)

    def render_table(self, width: int = 100, now: float | None = None) -> str:
        if now is None:
            now = self._clock()
        rows = self.snapshot(now)
        elapsed = max(now - self._start_t, 1e-6)
        head = f"{len(rows)} ids, {self.frames} frames, {self.frames / elapsed:6.1f} fps  ([xx] = byte changed in last {RECENT_CHANGE_S:.0f} s)"
        lines = [head[:width]]
        header = (
            f"{'id':>8} {'hz':>6} {'count':>7}  "
            + " ".join(f" b{i} " for i in range(MAX_BYTES))
            + "  changes/byte"
        )
        lines.append(header[:width])
        for st in rows:
            mask = st.changed_mask(now)
            cells = []
            for i in range(MAX_BYTES):
                if i < len(st.last):
                    cells.append(f"[{st.last[i]:02X}]" if mask >> i & 1 else f" {st.last[i]:02X} ")
                else:
                    cells.append("  . ")
            changes = "/".join(str(c) for c in st.byte_changes[: max(len(st.last), 1)])
            line = (
                f"{st.label():>8} {st.hz:6.1f} {st.count:7d}  " + " ".join(cells) + "  " + changes
            )
            lines.append(line[:width])
        return "\n".join(lines)

    def render_diff(self, width: int = 100, limit: int = 20) -> str:
        if not self.has_baseline:
            return "(no baseline yet)"
        changes = self.changes_since_baseline()
        if not changes:
            return "no bytes moved outside their baseline values"
        lines = [f"{'id':>8} {'byte':>4} {'frames':>7}  baseline values -> new values (last)"]
        for ch in changes[:limit]:
            base = "new id" if ch.new_id else _fmt_values(ch.baseline_values)
            new = _fmt_values(ch.new_values)
            line = f"{_label(ch.arbitration_id):>8} {ch.byte_index:>4} {ch.count:7d}  {base} -> {new} ({ch.last_value:02X})"
            lines.append(line[:width])
        if len(changes) > limit:
            lines.append(f"... and {len(changes) - limit} more")
        return "\n".join(lines)


def _label(arbitration_id: int) -> str:
    return f"{arbitration_id:08X}" if arbitration_id > 0x7FF else f"{arbitration_id:03X}"


def _fmt_values(values: set[int], limit: int = 6) -> str:
    vals = sorted(values)
    if not vals:
        return "-"
    if len(vals) > limit:
        return f"{vals[0]:02X}..{vals[-1]:02X} ({len(vals)} values)"
    return " ".join(f"{v:02X}" for v in vals)


def run_scan(
    bus: can.BusABC,
    duration: float,
    interval: float = 0.5,
    diff: bool = False,
    out: TextIO = sys.stdout,
    *,
    baseline_s: float = 3.0,
    width: int = 100,
) -> BusScanner:
    """Print a live id table every ``interval`` s for ``duration`` s (Ctrl-C stops early).

    With ``diff`` the first ``baseline_s`` seconds record the resting bus; the table is
    then followed by the bytes that moved since.  Returns the scanner for inspection.
    """
    scanner = BusScanner(bus)
    scanner.start()
    clear = CLEAR_SCREEN if getattr(out, "isatty", lambda: False)() else ""
    t_end = time.monotonic() + duration
    try:
        if diff:
            print(
                f"Recording baseline for {baseline_s:.0f} s - keep the car still...",
                file=out,
                flush=True,
            )
            scanner.baseline(min(baseline_s, duration))
            print("Baseline done. Now press the brake / turn the wheel.", file=out, flush=True)
        while time.monotonic() < t_end:
            time.sleep(max(0.0, min(interval, t_end - time.monotonic())))
            print(clear + scanner.render_table(width), file=out)
            if diff:
                print("\nmoved since baseline:", file=out)
                print(scanner.render_diff(width), file=out)
            out.flush()
    except KeyboardInterrupt:
        pass
    finally:
        scanner.stop()
    return scanner
