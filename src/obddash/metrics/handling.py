"""G-forces, friction circle and handling scores.

Sources (SAE signs: +lon accelerating, +lat left turn, +steering left, +yaw left)
---------------------------------------------------------------------------------
* ``"imu"``: ``acc_lat_g`` / ``acc_lon_g`` present (MPU6050, or the car's own
  accelerometer on CAN).  A light low-pass (tau 0.08 s) takes the edge off
  vibration; ``vert`` is filtered the same way (1.0 when unknown).
* ``"derived"`` (per axis, when the IMU axis is missing):

  * lon = dv/dt / 9.81, from speed low-passed (tau 0.2 s) and differentiated over
    0.4 s (so it lags ~0.4 s and cannot see jolts);
  * lat = v * yaw_rate / 9.81 when a yaw-rate signal exists (exact kinematics for
    steady cornering, and it already contains the car's real under/oversteer);
    otherwise the bicycle model from the steering wheel::

        delta = radians(steering_deg / steering_ratio)       road-wheel angle
        yaw   = v * delta / wheelbase                         [rad/s]
        lat_g = v * yaw / 9.81  =  v^2 * delta / (L * 9.81)    (clamped to +-2 g)

* ``"none"``: nothing to work with.

Outputs
-------
* ``trail``: the last ``cfg.g_trail_s`` seconds at 20 Hz as ``[lat, lon]``.
* peaks: ``peak_lat`` = max |lat|, ``peak_accel`` = max lon, ``peak_brake`` = max -lon.
* friction envelope = ``max(peak_g_floor, 0.98 * max_total)`` where ``max_total``
  is the largest ``total = hypot(lat, lon)`` seen, decaying 1 % per minute so one
  pothole does not set it for the whole session.
  ``friction_used_pct = total / envelope * 100`` (capped at 100).
* balance (needs yaw rate, steering and speed > 20 km/h)::

      yaw_ack = v * delta / L                       (deg/s, neutral-steer yaw rate)
      raw     = s * (yaw_meas - yaw_ack) / max(|yaw_ack|, 3 deg/s),  clamped -1..1
      s       = sign of the measured yaw rate (direction the car is really turning)

  low-passed with tau 0.5 s; negative = understeer (car rotates less than the
  steering asks), positive = oversteer (incl. counter-steer slides).  Going
  straight (both rates < 1 deg/s) feeds 0.  None when the inputs are missing.
* smoothness = ``clamp(100 - 25 * RMS(lateral jerk over the last 2 s), 0, 100)``,
  jerk in g/s from the 20 Hz lateral samples.
* corners: ``|lat|`` rising above ``cfg.corner_lat_g`` starts (and counts) a
  corner; it ends when ``|lat|`` falls below half that.  Each finished corner
  scores ``min(100, peak_total_g / envelope * 100)``.
* score = ``0.6 * smoothness + 0.4 * mean(corner scores of the last 20 corners)``,
  0 until the first corner has finished.
"""

from __future__ import annotations

import math

from obddash.config import HandlingConfig, VehicleConfig
from obddash.model import GForces, Handling

from .filters import Differentiator, LowPass, Sampler, TimedRing, clamp, rms

G = 9.81
IMU_TAU_S = 0.08
DERIVED_LON_TAU_S = 0.2
DERIVED_LON_WINDOW_S = 0.4
TRAIL_DT = 0.05  # 20 Hz
JERK_WINDOW_S = 2.0
JERK_MAX_GAP_S = 0.25
ENVELOPE_DECAY_PER_MIN = 0.01
BALANCE_MIN_KPH = 20.0
BALANCE_TAU_S = 0.5
BALANCE_STRAIGHT_DPS = 1.0
BALANCE_FLOOR_DPS = 3.0
MAX_DERIVED_LAT_G = 2.0
CORNER_HISTORY = 20

_LN_DECAY = math.log(1.0 - ENVELOPE_DECAY_PER_MIN) / 60.0  # per second


class HandlingAnalyzer:
    def __init__(self, cfg: HandlingConfig, vehicle: VehicleConfig) -> None:
        self.cfg = cfg
        self.vehicle = vehicle
        self._trail = TimedRing(cfg.g_trail_s, max_n=int(cfg.g_trail_s / TRAIL_DT) + 3)
        self._jerk = TimedRing(JERK_WINDOW_S, max_n=int(JERK_WINDOW_S / TRAIL_DT) + 3)
        self._lp_lat = LowPass(IMU_TAU_S)
        self._lp_lon = LowPass(IMU_TAU_S)
        self._lp_vert = LowPass(IMU_TAU_S)
        self._dv = Differentiator(DERIVED_LON_WINDOW_S, DERIVED_LON_TAU_S)
        self._lp_bal = LowPass(BALANCE_TAU_S)
        self.reset()

    def reset(self) -> None:
        self._trail.clear()
        self._jerk.clear()
        self._lp_lat.reset()
        self._lp_lon.reset()
        self._lp_vert.reset()
        self._dv.reset()
        self._lp_bal.reset()
        self._trail_list: list[list[float]] | None = []
        self._trail_due = Sampler(TRAIL_DT)
        self._prev_sample: tuple[float, float] | None = None  # (t, lat) of the last trail sample
        self._last_t: float | None = None
        self.lat_g: float | None = None
        self.lon_g: float | None = None
        self.vert_g = 1.0
        self.total_g = 0.0
        self.source = "none"
        self.lon_from_imu = False
        self.peak_lat = 0.0
        self.peak_accel = 0.0
        self.peak_brake = 0.0
        self._max_total = 0.0
        self.envelope = self.cfg.peak_g_floor
        self.balance: float | None = None
        self.smoothness = 0.0
        self.corner_count = 0
        self._in_corner = False
        self._corner_peak = 0.0
        self._corner_scores: list[float] = []

    # -- main ----------------------------------------------------------------------------
    def update(
        self,
        t: float,
        speed_kph: float | None,
        acc_lat_g: float | None,
        acc_lon_g: float | None,
        acc_vert_g: float | None,
        steering_deg: float | None,
        yaw_rate_dps: float | None,
    ) -> None:
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        v = speed_kph / 3.6 if speed_kph is not None else None

        # -- longitudinal ----------------------------------------------------------------
        imu = False
        derived = False
        if acc_lon_g is not None:
            lon = self._lp_lon.update(acc_lon_g, t)
            imu = True
        elif v is not None:
            dvdt = self._dv.update(v, t)
            lon = dvdt / G if dvdt is not None else None
            derived = lon is not None
        else:
            lon = None
        self.lon_from_imu = acc_lon_g is not None

        # -- lateral ---------------------------------------------------------------------
        if acc_lat_g is not None:
            lat = self._lp_lat.update(acc_lat_g, t)
            imu = True
        else:
            raw_lat = self._derived_lat(v, steering_deg, yaw_rate_dps)
            lat = self._lp_lat.update(raw_lat, t) if raw_lat is not None else None
            derived = derived or lat is not None

        vert = self._lp_vert.update(acc_vert_g, t) if acc_vert_g is not None else None
        self.vert_g = vert if vert is not None else 1.0
        self.lat_g = lat
        self.lon_g = lon
        self.source = "imu" if imu else "derived" if derived else "none"
        la = lat or 0.0
        lo = lon or 0.0
        total = math.hypot(la, lo)
        self.total_g = total

        # -- peaks & envelope ------------------------------------------------------------
        if lat is not None and abs(lat) > self.peak_lat:
            self.peak_lat = abs(lat)
        if lon is not None:
            if lon > self.peak_accel:
                self.peak_accel = lon
            if -lon > self.peak_brake:
                self.peak_brake = -lon
        if dt > 0.0:
            self._max_total *= math.exp(_LN_DECAY * dt)
        if total > self._max_total:
            self._max_total = total
        self.envelope = max(self.cfg.peak_g_floor, 0.98 * self._max_total)

        # -- 20 Hz trail + lateral jerk --------------------------------------------------
        if (lat is not None or lon is not None) and self._trail_due.due(t):
            self._trail.append(t, [round(la, 3), round(lo, 3)])
            self._trail_list = None
            if lat is not None:
                prev = self._prev_sample
                if prev is not None and 0.0 < t - prev[0] <= JERK_MAX_GAP_S:
                    self._jerk.append(t, (lat - prev[1]) / (t - prev[0]))
                self._prev_sample = (t, lat)
        self._jerk.prune(t)
        if self.source == "none":
            self.smoothness = 0.0
        else:
            self.smoothness = clamp(100.0 - 25.0 * rms(self._jerk.values()), 0.0, 100.0)

        self._update_balance(t, v, steering_deg, yaw_rate_dps, speed_kph)
        self._update_corners(la, total, lat is not None)

    def _derived_lat(
        self, v: float | None, steering_deg: float | None, yaw_rate_dps: float | None
    ) -> float | None:
        if v is None:
            return None
        if yaw_rate_dps is not None:
            return v * math.radians(yaw_rate_dps) / G
        if steering_deg is not None:
            ratio = self.vehicle.steering_ratio or 15.0
            delta = math.radians(steering_deg / ratio)
            lat = v * v * delta / (max(0.5, self.vehicle.wheelbase_m) * G)
            return clamp(lat, -MAX_DERIVED_LAT_G, MAX_DERIVED_LAT_G)
        return None

    def _update_balance(
        self,
        t: float,
        v: float | None,
        steering_deg: float | None,
        yaw_dps: float | None,
        speed_kph: float | None,
    ) -> None:
        if (
            v is None
            or steering_deg is None
            or yaw_dps is None
            or speed_kph is None
            or speed_kph <= BALANCE_MIN_KPH
        ):
            self.balance = None
            self._lp_bal.reset()
            return
        ratio = self.vehicle.steering_ratio or 15.0
        delta = math.radians(steering_deg / ratio)
        yaw_ack = math.degrees(v * delta / max(0.5, self.vehicle.wheelbase_m))
        if abs(yaw_dps) < BALANCE_STRAIGHT_DPS and abs(yaw_ack) < BALANCE_STRAIGHT_DPS:
            raw = 0.0
        else:
            turn = yaw_dps if abs(yaw_dps) >= BALANCE_STRAIGHT_DPS else yaw_ack
            sign = 1.0 if turn > 0 else -1.0
            raw = clamp(
                sign * (yaw_dps - yaw_ack) / max(abs(yaw_ack), BALANCE_FLOOR_DPS), -1.0, 1.0
            )
        self.balance = self._lp_bal.update(raw, t)

    def _update_corners(self, lat: float, total: float, have_lat: bool) -> None:
        if not have_lat:
            return
        thr = self.cfg.corner_lat_g
        al = abs(lat)
        if not self._in_corner:
            if al > thr:
                self._in_corner = True
                self.corner_count += 1
                self._corner_peak = total
        else:
            if total > self._corner_peak:
                self._corner_peak = total
            if al < 0.5 * thr:
                self._in_corner = False
                self._corner_scores.append(min(100.0, self._corner_peak / self.envelope * 100.0))
                if len(self._corner_scores) > CORNER_HISTORY:
                    del self._corner_scores[0]

    # -- output --------------------------------------------------------------------------
    @property
    def friction_used_pct(self) -> float:
        return min(100.0, self.total_g / self.envelope * 100.0) if self.envelope > 0 else 0.0

    @property
    def score(self) -> float:
        if not self._corner_scores:
            return 0.0
        corners = sum(self._corner_scores) / len(self._corner_scores)
        return 0.6 * self.smoothness + 0.4 * corners

    def g_snapshot(self) -> GForces:
        if self._trail_list is None:
            self._trail_list = self._trail.values()  # rebuilt on change, never mutated
        return GForces(
            lat=round(self.lat_g or 0.0, 4),
            lon=round(self.lon_g or 0.0, 4),
            vert=round(self.vert_g, 4),
            total=round(self.total_g, 4),
            peak_lat=round(self.peak_lat, 3),
            peak_accel=round(self.peak_accel, 3),
            peak_brake=round(self.peak_brake, 3),
            source=self.source,
            trail=self._trail_list,
        )

    def snapshot(self) -> Handling:
        return Handling(
            score=round(self.score, 1),
            smoothness=round(self.smoothness, 1),
            balance=None if self.balance is None else round(self.balance, 3),
            friction_used_pct=round(self.friction_used_pct, 1),
            max_lat_g=round(self.peak_lat, 3),
            corner_count=self.corner_count,
        )
