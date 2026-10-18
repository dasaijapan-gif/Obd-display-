"""Small, allocation-light signal helpers shared by the metric analyzers.

Everything here is pure Python and sized for a Raspberry Pi ticking at 50 Hz: the
buffers are bounded (by time *and* count), nothing grows without limit, and the
per-update cost is a handful of float operations (the differentiator is O(n) in its
window, which holds ~10-30 samples).

* :func:`clamp`, :func:`lerp` - the obvious.
* :class:`LowPass` - first-order exponential low-pass that is *time aware*:
  ``alpha = 1 - exp(-dt / tau)``, so irregular tick intervals (GC pauses, a slow
  OBD answer) do not change the filter's time constant.  Steady-state lag for a
  ramp input is exactly ``tau``.
* :class:`Differentiator` - derivative of an (optionally low-passed) signal: the
  least-squares slope of the samples inside a sliding time window.  Least squares
  is robust to irregular ``dt`` and to the repeated values you get when a 15 Hz
  OBD PID is read by a 50 Hz loop.  It also exposes the fitted value at the
  window's centroid time (``center`` / ``center_t``): slope and centre describe the
  same instant, which is what the dyno needs to pair acceleration with speed.
  Total lag of ``slope`` relative to the raw input = ``tau + window/2``.
* :class:`TimedRing` - (t, value) ring that drops entries older than ``max_s`` and
  never holds more than ``max_n``; with slicing helpers for traces.
* :func:`downsample` - evenly spaced subset (first and last points kept).
* :class:`Sampler` - "is a sample due?" at a fixed average rate from an irregular
  tick (a 20 Hz trail from a 50 Hz loop alternates 2- and 3-tick gaps).
* :class:`RunningStats` - Welford running mean / variance.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Iterator, Sequence
from typing import Any


def clamp(x: float, lo: float, hi: float) -> float:
    """Limit ``x`` to ``[lo, hi]``."""
    return lo if x < lo else hi if x > hi else x


def lerp(a: float, b: float, f: float) -> float:
    """Linear interpolation: ``a`` at ``f = 0``, ``b`` at ``f = 1`` (not clamped)."""
    return a + (b - a) * f


# --------------------------------------------------------------------------------------
# Low-pass
# --------------------------------------------------------------------------------------
class LowPass:
    """Time-aware first-order low-pass.

    ``y += (x - y) * (1 - exp(-dt / tau))``.  The first sample initialises the
    output.  ``None`` inputs and non-increasing timestamps leave the state untouched.
    ``tau_s <= 0`` passes the input straight through.
    """

    __slots__ = ("tau_s", "value", "_t")

    def __init__(self, tau_s: float, value: float | None = None) -> None:
        self.tau_s = float(tau_s)
        self.value: float | None = value
        self._t: float | None = None

    def update(self, x: float | None, t: float) -> float | None:
        if x is None:
            return self.value
        if self.value is None or self._t is None:
            self.value = float(x)
            self._t = t
            return self.value
        dt = t - self._t
        if dt <= 0.0:
            return self.value
        self._t = t
        if self.tau_s <= 0.0:
            self.value = float(x)
        else:
            self.value += (x - self.value) * (1.0 - math.exp(-dt / self.tau_s))
        return self.value

    def reset(self, value: float | None = None) -> None:
        self.value = value
        self._t = None


# --------------------------------------------------------------------------------------
# Differentiator
# --------------------------------------------------------------------------------------
class Differentiator:
    """Least-squares slope over a sliding time window, optionally after a low-pass.

    ``update(x, t)`` returns the slope (units of ``x`` per second) or ``None`` until
    the window holds at least 3 samples spanning ``min_span_s`` (default: half the
    window).  For samples ``(t_i, y_i)`` in the window::

        slope = sum((t_i - t_m) * (y_i - y_m)) / sum((t_i - t_m)^2)

    with ``t_m``, ``y_m`` the means; ``center = y_m`` is the fitted value at ``t_m``.
    Times are taken relative to the newest sample so large monotonic clocks do not
    cost precision.
    """

    __slots__ = (
        "window_s",
        "min_span_s",
        "_lp",
        "_buf",
        "slope",
        "center",
        "center_t",
    )

    def __init__(
        self,
        window_s: float = 0.3,
        tau_s: float = 0.0,
        *,
        max_n: int = 64,
        min_span_s: float | None = None,
    ) -> None:
        self.window_s = float(window_s)
        self.min_span_s = self.window_s * 0.5 if min_span_s is None else float(min_span_s)
        self._lp = LowPass(tau_s) if tau_s > 0 else None
        self._buf: deque[tuple[float, float]] = deque(maxlen=max(3, int(max_n)))
        self.slope: float | None = None
        self.center: float | None = None
        self.center_t: float | None = None

    @property
    def filtered(self) -> float | None:
        """The latest (low-passed) input value."""
        if self._lp is not None:
            return self._lp.value
        return self._buf[-1][1] if self._buf else None

    def update(self, x: float | None, t: float) -> float | None:
        if x is None:
            return self.slope
        buf = self._buf
        if buf and t <= buf[-1][0]:
            return self.slope  # same tick (or clock went backwards): nothing new
        y = self._lp.update(x, t) if self._lp is not None else float(x)
        buf.append((t, y))
        cutoff = t - self.window_s
        while len(buf) > 2 and buf[0][0] < cutoff:
            buf.popleft()
        n = len(buf)
        if n < 3 or (t - buf[0][0]) < self.min_span_s:
            self.slope = self.center = self.center_t = None
            return None
        st = sy = 0.0
        for ti, yi in buf:
            st += ti - t
            sy += yi
        tm = st / n
        ym = sy / n
        num = den = 0.0
        for ti, yi in buf:
            dt = (ti - t) - tm
            num += dt * (yi - ym)
            den += dt * dt
        if den <= 0.0:
            return self.slope
        self.slope = num / den
        self.center = ym
        self.center_t = t + tm
        return self.slope

    def reset(self) -> None:
        if self._lp is not None:
            self._lp.reset()
        self._buf.clear()
        self.slope = self.center = self.center_t = None


# --------------------------------------------------------------------------------------
# Time-bounded ring
# --------------------------------------------------------------------------------------
class TimedRing:
    """Ring of ``(t, value)`` entries, oldest first, bounded by age and count.

    ``append`` drops entries older than ``t - max_s`` (relative to the newest entry)
    and the deque's ``maxlen`` caps the count, so memory is bounded whatever the
    tick rate.
    """

    __slots__ = ("max_s", "_buf")

    def __init__(self, max_s: float, max_n: int) -> None:
        self.max_s = float(max_s)
        self._buf: deque[tuple[float, Any]] = deque(maxlen=max(1, int(max_n)))

    def append(self, t: float, value: Any) -> None:
        buf = self._buf
        buf.append((t, value))
        cutoff = t - self.max_s
        while buf and buf[0][0] < cutoff:
            buf.popleft()

    def prune(self, now: float) -> None:
        """Drop entries older than ``now - max_s`` (for rings that are not appended to
        every tick)."""
        cutoff = now - self.max_s
        buf = self._buf
        while buf and buf[0][0] < cutoff:
            buf.popleft()

    def __len__(self) -> int:
        return len(self._buf)

    def __iter__(self) -> Iterator[tuple[float, Any]]:
        return iter(self._buf)

    def __bool__(self) -> bool:
        return bool(self._buf)

    def last(self) -> tuple[float, Any] | None:
        return self._buf[-1] if self._buf else None

    def values(self) -> list[Any]:
        return [v for _, v in self._buf]

    def items(self) -> list[tuple[float, Any]]:
        return list(self._buf)

    def between(self, t0: float, t1: float) -> list[tuple[float, Any]]:
        """Entries with ``t0 <= t <= t1``, oldest first."""
        return [e for e in self._buf if t0 <= e[0] <= t1]

    def before(self, t: float) -> tuple[float, Any] | None:
        """The newest entry strictly older than ``t``."""
        for e in reversed(self._buf):
            if e[0] < t:
                return e
        return None

    def clear(self) -> None:
        self._buf.clear()


class Sampler:
    """Decimates an irregular tick to an average rate of ``1 / period_s``.

    ``due(t)`` is True when the next sample time has been reached; the schedule
    advances by exactly one period (so 20 Hz from 50 Hz ticks really is 20 Hz),
    and restarts from ``t`` after a gap instead of bursting to catch up.
    """

    __slots__ = ("period_s", "_next")

    def __init__(self, period_s: float) -> None:
        self.period_s = float(period_s)
        self._next: float | None = None

    def due(self, t: float) -> bool:
        nxt = self._next
        if nxt is not None and t < nxt - 1e-6:
            return False
        nxt = t + self.period_s if nxt is None else nxt + self.period_s
        if nxt <= t:
            nxt = t + self.period_s
        self._next = nxt
        return True

    def reset(self) -> None:
        self._next = None


def downsample(points: Sequence[Any], max_n: int) -> list[Any]:
    """At most ``max_n`` evenly spaced items of ``points`` (first and last kept)."""
    n = len(points)
    if n <= max_n:
        return list(points)
    if max_n <= 0:
        return []
    if max_n == 1:
        return [points[-1]]
    step = (n - 1) / (max_n - 1)
    return [points[int(round(i * step))] for i in range(max_n)]


# --------------------------------------------------------------------------------------
# Running statistics
# --------------------------------------------------------------------------------------
class RunningStats:
    """Welford's online mean / variance (numerically stable, O(1) per sample)."""

    __slots__ = ("n", "mean", "_m2")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self._m2 = 0.0

    def add(self, x: float) -> None:
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self._m2 += d * (x - self.mean)

    @property
    def var(self) -> float:
        return self._m2 / (self.n - 1) if self.n > 1 else 0.0

    @property
    def std(self) -> float:
        return math.sqrt(self.var)

    def reset(self) -> None:
        self.n = 0
        self.mean = 0.0
        self._m2 = 0.0


def rms(values: Sequence[float]) -> float:
    """Root mean square (0 for an empty sequence)."""
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))
