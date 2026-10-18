"""Brake pressure, braking events and the brake score.

Pressure source
---------------
* ``"can"``: a pressure signal exists (``pressure_bar`` not None).
* ``"estimate"``: no pressure, but a brake switch exists.  While the switch is on::

      pressure_bar ~= brake_cfg.estimate_bar_per_g * max(0, -lon_g)

  (0 while off).  ``lon_g`` is whatever the engine has - IMU or derived from speed;
  it includes drag and engine braking, so treat it as indicative.
* ``"none"``: neither.

``active`` = pressure >= ``active_threshold_bar`` (CAN), or the brake switch
(estimate mode).

Events
------
An event starts when ``active`` becomes true and ends once it has been inactive
for 0.2 s (``duration`` runs to the moment it went inactive).  Events shorter than
``cfg.event_min_s`` are discarded.  Per event::

    entry_kph / exit_kph = speed at start / at release
    peak_bar             = max pressure
    avg_decel_g          = mean of -lon_g over the active samples where lon_g is
                           known (0 if it never was)
    trail_pct            = % of active samples with |lat_g| > cfg.trail_lat_g
    smoothness           = clamp(100 - 0.4 * RMS(dP/dt), 0, 100)
                           dP/dt in bar/s from samples >= 40 ms apart (so a 50 Hz
                           loop re-reading a slower signal does not add zeros)

Session score = mean over the last 10 events of
``0.6 * smoothness + 0.4 * min(100, peak_bar / max_pressure_bar * 130)``
(130: hitting ~77 % of full scale counts as full use of the brakes).

``trace`` = the last ``cfg.trace_s`` seconds at <= 25 Hz as ``[t, bar]``;
``events`` capped at ``cfg.max_events`` (most recent last); ``pressure_pct`` =
pressure / max_pressure_bar * 100 clamped to 0..100; ``peak_bar`` is the session
peak (``reset_peak()`` zeroes it).
"""

from __future__ import annotations

import math
from collections import deque

from obddash.config import BrakeConfig, BrakeMetricsConfig
from obddash.model import Brake, BrakeEvent

from .filters import Sampler, TimedRing, clamp

RELEASE_S = 0.2
TRACE_DT = 0.04  # 25 Hz
DPDT_MIN_DT = 0.04
SCORE_EVENTS = 10
SCORE_PEAK_GAIN = 130.0


class _Event:
    __slots__ = (
        "t_start",
        "entry_kph",
        "peak",
        "decel_sum",
        "decel_n",
        "n",
        "trail_n",
        "dp2_sum",
        "dp_n",
        "last_p",
        "last_pt",
    )

    def __init__(self, t: float, speed_kph: float | None, p: float) -> None:
        self.t_start = t
        self.entry_kph = speed_kph if speed_kph is not None else 0.0
        self.peak = p
        self.decel_sum = 0.0
        self.decel_n = 0  # active samples with a known lon_g
        self.n = 0
        self.trail_n = 0
        self.dp2_sum = 0.0
        self.dp_n = 0
        self.last_p = p
        self.last_pt = t


class BrakeAnalyzer:
    def __init__(self, cfg: BrakeMetricsConfig, brake_cfg: BrakeConfig) -> None:
        self.cfg = cfg
        self.brake_cfg = brake_cfg
        self._trace = TimedRing(cfg.trace_s, max_n=int(cfg.trace_s / TRACE_DT) + 4)
        self._events: deque[BrakeEvent] = deque(maxlen=max(1, cfg.max_events))
        self.reset()

    def reset(self) -> None:
        self._trace.clear()
        self._events.clear()
        self._trace_due = Sampler(TRACE_DT)
        self._trace_list: list[list[float]] | None = []
        self._events_list: list[BrakeEvent] | None = []
        self._ev: _Event | None = None
        self._off_since: float | None = None
        self._exit_kph: float | None = None
        self._recent_scores: deque[float] = deque(maxlen=SCORE_EVENTS)
        self.count = 0
        self.score = 0.0
        self.peak_bar = 0.0
        self.pressure_bar = 0.0
        self.front_bar: float | None = None
        self.rear_bar: float | None = None
        self.active = False
        self.source = "none"

    def reset_peak(self) -> None:
        self.peak_bar = 0.0

    # -- main ----------------------------------------------------------------------------
    def update(
        self,
        t: float,
        pressure_bar: float | None,
        front_bar: float | None,
        rear_bar: float | None,
        brake_switch: bool | None,
        lon_g: float | None,
        lat_g: float | None,
        speed_kph: float | None,
    ) -> BrakeEvent | None:
        bc = self.brake_cfg
        if pressure_bar is not None:
            source = "can"
            p = max(0.0, pressure_bar)
            active = p >= bc.active_threshold_bar
        elif brake_switch is not None:
            source = "estimate"
            if brake_switch and lon_g is not None:
                p = bc.estimate_bar_per_g * max(0.0, -lon_g)
            else:
                p = 0.0
            active = bool(brake_switch)
        else:
            source = "none"
            p = 0.0
            active = False
        self.source = source
        self.pressure_bar = p
        self.front_bar = front_bar
        self.rear_bar = rear_bar
        self.active = active
        if p > self.peak_bar:
            self.peak_bar = p

        if self._trace_due.due(t):
            self._trace.append(t, [round(t, 3), round(p, 2)])
            self._trace_list = None  # rebuild lazily

        done: BrakeEvent | None = None
        if active:
            ev = self._ev
            if ev is None:
                ev = self._ev = _Event(t, speed_kph, p)
            self._off_since = None
            ev.n += 1
            if p > ev.peak:
                ev.peak = p
            if lon_g is not None:
                ev.decel_sum += -lon_g
                ev.decel_n += 1
            if lat_g is not None and abs(lat_g) > self.cfg.trail_lat_g:
                ev.trail_n += 1
            if t - ev.last_pt >= DPDT_MIN_DT:
                dpdt = (p - ev.last_p) / (t - ev.last_pt)
                ev.dp2_sum += dpdt * dpdt
                ev.dp_n += 1
                ev.last_p, ev.last_pt = p, t
        elif self._ev is not None:
            if self._off_since is None:
                self._off_since = t
                self._exit_kph = speed_kph
            if t - self._off_since >= RELEASE_S:
                done = self._close(self._ev, self._off_since)
                self._ev = None
                self._off_since = None
        return done

    def _close(self, ev: _Event, t_end: float) -> BrakeEvent | None:
        duration = t_end - ev.t_start
        if duration < self.cfg.event_min_s or ev.n == 0:
            return None
        exit_kph = self._exit_kph if self._exit_kph is not None else 0.0
        rms_dpdt = math.sqrt(ev.dp2_sum / ev.dp_n) if ev.dp_n else 0.0
        result = BrakeEvent(
            t_start=round(ev.t_start, 3),
            duration_s=round(duration, 3),
            peak_bar=round(ev.peak, 2),
            entry_kph=round(ev.entry_kph, 1),
            exit_kph=round(exit_kph, 1),
            avg_decel_g=round(ev.decel_sum / ev.decel_n, 3) if ev.decel_n else 0.0,
            trail_pct=round(100.0 * ev.trail_n / ev.n, 1),
            smoothness=round(clamp(100.0 - 0.4 * rms_dpdt, 0.0, 100.0), 1),
        )
        self._events.append(result)
        self._events_list = None
        self.count += 1
        max_bar = self.brake_cfg.max_pressure_bar or 1.0
        use = min(100.0, result.peak_bar / max_bar * SCORE_PEAK_GAIN)
        self._recent_scores.append(0.6 * result.smoothness + 0.4 * use)
        self.score = round(sum(self._recent_scores) / len(self._recent_scores), 1)
        return result

    # -- output --------------------------------------------------------------------------
    @property
    def live_trail_pct(self) -> float:
        ev = self._ev
        if ev is None or ev.n == 0:
            return 0.0
        return 100.0 * ev.trail_n / ev.n

    def snapshot(self) -> Brake:
        if self._trace_list is None:
            self._trace_list = self._trace.values()
        if self._events_list is None:
            self._events_list = list(self._events)
        max_bar = self.brake_cfg.max_pressure_bar or 1.0
        return Brake(
            pressure_bar=round(self.pressure_bar, 2),
            pressure_pct=round(clamp(self.pressure_bar / max_bar * 100.0, 0.0, 100.0), 1),
            front_bar=self.front_bar,
            rear_bar=self.rear_bar,
            active=self.active,
            peak_bar=round(self.peak_bar, 2),
            source=self.source,
            trail_pct=round(self.live_trail_pct, 1),
            # cached lists are rebuilt (never mutated) on change, so sharing them is safe
            events=self._events_list,
            trace=self._trace_list,
            score=self.score,
            count=self.count,
        )
