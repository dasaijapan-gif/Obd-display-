"""Session totals: duration, distance, maxima, fuel used.

* ``duration_s`` = time since the first update after (re)start.
* ``distance_km`` = trapezoidal integral of speed: ``sum((v_prev + v) / 2 * dt) / 3600``.
* ``fuel_used_l`` = trapezoidal integral of ``fuel_rate_lph`` (PID 0x5E) the same
  way; ``None`` until a fuel-rate value has been seen.
* Gaps longer than ``MAX_GAP_S`` (signal lost, process paused) are not integrated,
  so a dropout never produces a jump.
"""

from __future__ import annotations

from obddash.model import Session

MAX_GAP_S = 2.0


class SessionTracker:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._t0: float | None = None
        self._last_t: float | None = None
        self._last_speed: float | None = None
        self._last_fuel: float | None = None
        self.duration_s = 0.0
        self.distance_km = 0.0
        self.max_speed_kph = 0.0
        self.max_rpm = 0.0
        self.fuel_used_l: float | None = None

    def update(
        self,
        t: float,
        speed_kph: float | None,
        rpm: float | None,
        fuel_rate_lph: float | None = None,
    ) -> None:
        if self._t0 is None:
            self._t0 = t
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        self.duration_s = max(0.0, t - self._t0)
        integrate = 0.0 < dt <= MAX_GAP_S

        if speed_kph is not None:
            if integrate and self._last_speed is not None:
                self.distance_km += (self._last_speed + speed_kph) * 0.5 * dt / 3600.0
            if speed_kph > self.max_speed_kph:
                self.max_speed_kph = speed_kph
        self._last_speed = speed_kph

        if rpm is not None and rpm > self.max_rpm:
            self.max_rpm = rpm

        if fuel_rate_lph is not None:
            if self.fuel_used_l is None:
                self.fuel_used_l = 0.0
            if integrate and self._last_fuel is not None:
                self.fuel_used_l += (self._last_fuel + fuel_rate_lph) * 0.5 * dt / 3600.0
        self._last_fuel = fuel_rate_lph

    def snapshot(self) -> Session:
        return Session(
            duration_s=round(self.duration_s, 2),
            distance_km=round(self.distance_km, 4),
            max_speed_kph=round(self.max_speed_kph, 1),
            max_rpm=round(self.max_rpm, 0),
            fuel_used_l=None if self.fuel_used_l is None else round(self.fuel_used_l, 4),
        )
