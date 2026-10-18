"""Record raw signals to JSON Lines and play them back.

What is recorded is the *raw* signal stream (the :class:`~obddash.model.RawState`
contents), not the processed telemetry.  Replaying raw signals through the metrics
engine reproduces every metric, so an improved shift scorer or dyno can be re-run on
an old drive, and a replay exercises exactly the code path a live car does.

File format (UTF-8, one JSON object per line)::

    {"meta": {"format": 1, "version": "0.1.0", "vehicle": "Sim Sportscar (6MT)",
              "started": 1760000000.0, "started_iso": "2025-10-09T08:53:20", ...}}
    {"t": 0.0,   "v": {"rpm": 850, "speed_kph": 0, "coolant_c": 88, ...}}   <- everything
    {"t": 0.02,  "v": {"rpm": 862.5}}                                     <- deltas
    {"t": 0.04,  "v": {"rpm": 871.25, "acc_lon_g": 0.012}}

``t`` is seconds since the first data line.  The first data line carries every signal
known at that moment; after that a line holds only the signals whose value changed
since they were last written, and no line at all is written for a tick in which
nothing changed.  Values are rounded to 4 decimals (integers are written without a
fraction) to keep files small: a 50 Hz drive is typically 5-15 kB/s.

**Keepalive.**  Pure change-encoding would lose *freshness*: a coolant temperature
that the ECU reports as 92 every two seconds never changes, so it would be written
once, and on replay it would go stale after ``metrics.stale_s`` and read as missing.
So a signal that keeps being *refreshed* (its RawState timestamp advances) but keeps
its value is re-written at most once per ``keepalive_s`` (1 s, below the 2 s stale
limit).  A signal that stops arriving is not re-written, so it goes stale on replay
exactly as it did live.

Writes go through a large buffer and are flushed every ``flush_s`` (1 s), so a
power cut loses at most about a second; the reader side tolerates a truncated last
line.  :class:`Replayer` streams the file (recordings of long drives are not loaded
into memory) and schedules lines against a monotonic deadline so timing does not
drift, scaled by ``speed``.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from collections.abc import Iterator
from datetime import datetime
from pathlib import Path
from typing import Any

from obddash import __version__
from obddash.model import RawState

log = logging.getLogger(__name__)

FORMAT_VERSION = 1
FILE_PREFIX = "obddash-"
FILE_SUFFIX = ".jsonl"
_JSON_SEPARATORS = (",", ":")


# --------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------
def make_recording_path(directory: str | Path, now: datetime | None = None) -> Path:
    """``<dir>/obddash-YYYYmmdd-HHMMSS.jsonl``; a ``-N`` suffix avoids overwriting a
    recording started within the same second.  The directory is not created here."""
    directory = Path(directory)
    stamp = (now or datetime.now()).strftime("%Y%m%d-%H%M%S")
    path = directory / f"{FILE_PREFIX}{stamp}{FILE_SUFFIX}"
    n = 1
    while path.exists():
        path = directory / f"{FILE_PREFIX}{stamp}-{n}{FILE_SUFFIX}"
        n += 1
    return path


def _compact(v: float) -> float | int:
    """Write 850.0 as 850: every byte counts at 50 lines per second."""
    return int(v) if v.is_integer() and abs(v) < 1e15 else v


def parse_line(line: str | bytes) -> tuple[float, dict[str, float]] | None:
    """Decode one data line into ``(t, {signal: value})``; None for the meta header,
    blank, truncated or otherwise invalid lines."""
    try:
        obj = json.loads(line)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(obj, dict) or "t" not in obj:
        return None
    try:
        t = float(obj["t"])
    except (TypeError, ValueError):
        return None
    if not math.isfinite(t):
        return None
    raw_values = obj.get("v") or {}
    values: dict[str, float] = {}
    if isinstance(raw_values, dict):
        for name, value in raw_values.items():
            # bool is an int subclass; RawState stores switches as 0/1 floats anyway
            if isinstance(value, int | float) and math.isfinite(value):
                values[str(name)] = float(value)
    return t, values


def iter_recording(path: str | Path) -> Iterator[tuple[float, dict[str, float]]]:
    """Stream the data lines of a recording (meta and corrupt lines skipped)."""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            rec = parse_line(line)
            if rec is not None:
                yield rec


def read_recording(path: str | Path) -> list[tuple[float, dict[str, float]]]:
    """All data lines of a recording as ``[(t, {signal: value}), ...]`` in file order.
    Convenient for tests and offline analysis; replay streams instead."""
    return list(iter_recording(path))


def read_meta(path: str | Path) -> dict[str, Any] | None:
    """The ``meta`` header of a recording, or None if the file has none."""
    with open(path, encoding="utf-8", errors="replace") as fh:
        first = fh.readline()
    try:
        obj = json.loads(first)
    except ValueError:
        return None
    if isinstance(obj, dict) and isinstance(obj.get("meta"), dict):
        return obj["meta"]
    return None


def _first_t(path: Path) -> float | None:
    for t, _values in iter_recording(path):
        return t
    return None


def _last_t(path: Path) -> float | None:
    """Time of the last valid data line, read from the end of the file so that the
    duration of a long recording is known without scanning all of it."""
    with open(path, "rb") as fh:
        fh.seek(0, 2)
        size = fh.tell()
        block = 64 * 1024
        while True:
            start = max(0, size - block)
            fh.seek(start)
            lines = fh.read(size - start).split(b"\n")
            if start > 0:
                lines = lines[1:]  # the first piece is probably a partial line
            for line in reversed(lines):
                rec = parse_line(line)
                if rec is not None:
                    return rec[0]
            if start == 0:
                return None
            block *= 4


# --------------------------------------------------------------------------------------
# Recorder
# --------------------------------------------------------------------------------------
class Recorder:
    """Write a delta-encoded JSONL recording of a :class:`RawState` (see module doc).

    ``sample()`` is called from the runtime's tick loop; it is cheap (one snapshot,
    one dict comparison) and never raises on I/O errors: the first failed write logs
    an error and turns the recorder off.  ``close()`` may be called from any thread.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        meta: dict[str, Any] | None = None,
        keepalive_s: float = 1.0,
        flush_s: float = 1.0,
        precision: int = 4,
    ) -> None:
        self.path = Path(path)
        self.keepalive_s = keepalive_s
        self.flush_s = flush_s
        self.precision = precision
        self.lines = 0  # data lines written
        self._lock = threading.Lock()
        # signal -> (value as written, RawState timestamp of that value, t when written)
        self._last: dict[str, tuple[float, float, float]] = {}
        self._t0: float | None = None  # t of the first data line (written as t=0)
        self._last_flush: float | None = None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "x", encoding="utf-8", buffering=64 * 1024)
        now = time.time()
        header: dict[str, Any] = {
            "format": FORMAT_VERSION,
            "version": __version__,
            "started": round(now, 3),
            "started_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
        }
        header.update(meta or {})
        self._write_line({"meta": header})

    # -- public ---------------------------------------------------------------------
    @property
    def closed(self) -> bool:
        return self._fh is None

    def sample(self, raw: RawState, t: float) -> bool:
        """Append one line with the signals that changed (or need a keepalive) since
        the previous line.  Returns True if a line was written."""
        with self._lock:
            if self._fh is None:
                return False
            if self._last_flush is None:
                self._last_flush = t
            changed: dict[str, float | int] = {}
            last = self._last
            for name, rv in raw.snapshot().items():
                value = rv.value
                if not math.isfinite(value):
                    continue
                value = round(value, self.precision)
                prev = last.get(name)
                if (
                    prev is None
                    or prev[0] != value
                    # unchanged but still being refreshed: keep it fresh on replay
                    or (rv.t != prev[1] and t - prev[2] >= self.keepalive_s)
                ):
                    changed[name] = _compact(value)
                    last[name] = (value, rv.t, t)
            wrote = False
            if changed:
                if self._t0 is None:
                    self._t0 = t
                wrote = self._write_line({"t": round(t - self._t0, 3), "v": changed})
                if wrote:
                    self.lines += 1
            if self._fh is not None and t - self._last_flush >= self.flush_s:
                self._last_flush = t
                self._flush()
            return wrote

    def close(self) -> None:
        with self._lock:
            fh, self._fh = self._fh, None
        if fh is None:
            return
        try:
            fh.close()
        except OSError as exc:
            log.error("closing recording %s failed: %s", self.path, exc)
        else:
            log.info("recording closed: %s (%d lines)", self.path, self.lines)

    def __enter__(self) -> Recorder:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- internals (lock held) --------------------------------------------------------
    def _write_line(self, obj: dict[str, Any]) -> bool:
        assert self._fh is not None
        try:
            self._fh.write(json.dumps(obj, separators=_JSON_SEPARATORS) + "\n")
        except (OSError, ValueError) as exc:
            self._fail(exc)
            return False
        return True

    def _flush(self) -> None:
        assert self._fh is not None
        try:
            self._fh.flush()
        except (OSError, ValueError) as exc:
            self._fail(exc)

    def _fail(self, exc: Exception) -> None:
        log.error("recording to %s failed (%s); recording stopped", self.path, exc)
        fh, self._fh = self._fh, None
        try:
            if fh is not None:
                fh.close()
        except (OSError, ValueError):
            pass


# --------------------------------------------------------------------------------------
# Replayer
# --------------------------------------------------------------------------------------
class Replayer:
    """Feed a :class:`RawState` from a recording in a background thread.

    Lines are applied with their original spacing divided by ``speed`` (which may be
    changed while playing).  The schedule is a running monotonic deadline, so sleep
    jitter does not accumulate; if the thread falls more than 0.5 s behind (machine
    suspended) it resynchronises instead of bursting.  Values are written with the
    current monotonic time, so RawState staleness works exactly as with a live car.
    With ``loop=True`` playback restarts at the beginning after the last line.
    """

    def __init__(
        self, path: str | Path, raw: RawState, speed: float = 1.0, *, loop: bool = False
    ) -> None:
        self.path = Path(path)
        if not self.path.is_file():
            raise FileNotFoundError(f"recording not found: {self.path}")
        if not speed > 0:
            raise ValueError(f"replay speed must be > 0, got {speed!r}")
        self.raw = raw
        self.speed = float(speed)
        self.loop = loop
        self.meta = read_meta(self.path) or {}
        self._t_first = _first_t(self.path)
        t_last = _last_t(self.path)
        self._duration = (
            max(0.0, t_last - self._t_first)
            if (self._t_first is not None and t_last is not None)
            else 0.0
        )
        self._position = 0.0
        self.passes = 0  # complete passes through the file
        self.lines = 0  # data lines applied (all passes)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # -- properties -------------------------------------------------------------------
    @property
    def duration(self) -> float:
        """Recording length in seconds (first to last data line)."""
        return self._duration

    @property
    def position(self) -> float:
        """Seconds into the recording of the last applied line."""
        return self._position

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def finished(self) -> bool:
        """True once a non-looping replay has applied its last line (or was stopped)."""
        return self._thread is not None and not self._thread.is_alive()

    # -- lifecycle --------------------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="obddash-replay", daemon=True)
        self._thread.start()
        log.info(
            "replaying %s (%.1f s, x%.2f%s)",
            self.path,
            self.duration,
            self.speed,
            ", loop" if self.loop else "",
        )

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t is not threading.current_thread():
            t.join(timeout=2.0)

    def wait(self, timeout: float | None = None) -> bool:
        """Block until playback ends; True if it did within ``timeout``."""
        t = self._thread
        if t is None:
            return False
        t.join(timeout)
        return not t.is_alive()

    # -- thread -----------------------------------------------------------------------
    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                applied = self._play_once()
                if self._stop.is_set():
                    break
                self.passes += 1
                if not self.loop:
                    log.info("replay finished: %s", self.path)
                    break
                if applied == 0:
                    log.warning("recording %s has no data lines; not looping", self.path)
                    break
        except Exception:
            log.exception("replay of %s failed", self.path)

    def _play_once(self) -> int:
        deadline = time.monotonic()
        prev_t: float | None = None
        t_first = self._t_first or 0.0
        applied = 0
        for t, values in iter_recording(self.path):
            if self._stop.is_set():
                break
            if prev_t is not None:
                # schedule against a running deadline: drift-free, speed may change live
                deadline += max(0.0, t - prev_t) / max(self.speed, 1e-3)
                delay = deadline - time.monotonic()
                if delay > 0:
                    if self._stop.wait(delay):
                        break
                elif delay < -0.5:
                    deadline = time.monotonic()
            prev_t = t
            if values:
                self.raw.update(values)
            self._position = max(0.0, t - t_first)
            self.lines += 1
            applied += 1
        return applied
