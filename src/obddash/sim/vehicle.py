"""Vehicle physics for the simulator: a simple but believable car, pure Python.

The model is a longitudinal point mass plus a bicycle-model lateral channel,
deliberately cheap (a few hundred float ops per step) and deterministic so that
tests can assert on it.  It is driven by :class:`Controls` (what the scripted
driver does with the pedals, clutch, gear lever and wheel) and produces a
:class:`VehicleState` that the fake ECU (``sim.ecu``) encodes onto the CAN bus.

Engine
------
* Wide-open-throttle torque is a smooth parametric curve: a quadratic rise to
  ``peak_torque_nm`` at ``peak_torque_rpm`` and a quartic fall-off above it, tuned
  so the default "Sim Sportscar" makes ~205 Nm @ 4000 rpm and ~135 kW @ 7000 rpm.
* Part throttle blends WOT torque with engine braking (``-engine_brake_nm_per_rpm *
  rpm``), the ECU idle controller adds torque near idle, and the rev limiter cuts
  fuel at ``redline_rpm`` (with a little hysteresis).
* With the driveline open (neutral / clutch down) the ECU caps torque so the free
  engine revs at ~``free_rev_rate_rpm_s`` at WOT and decays toward idle under
  engine braking.  That is the behaviour a heel-toe blip plays against.

Clutch
------
A Coulomb friction clutch with a torque capacity.  ``clutch`` is the pedal
(1 = down); engagement ``e = 1 - clutch`` and nothing is transmitted below the bite
point.  Each step the clutch torque needed to bring engine and driveline to the
same speed *and* carry the engine torque is computed; if it is within the current
capacity (``e_eff * clutch_capacity_nm``) the driveline is *locked* and
``rpm = speed_kph * VehicleConfig.rpm_per_kph(gear)`` **exactly**, so the metrics'
ratio-based gear estimator sees clean ratios.  Otherwise the clutch slips at its
capacity in the direction that reduces slip (the "friction impulse" clamp keeps
this stable at any step size).  A clutch released over ~0.15 s onto an rpm
mismatch therefore syncs partway through the pedal travel: a small mismatch
syncs early at low capacity (small jolt), a big one late at high capacity (big
jolt) - the reaction torque at the wheels is the longitudinal-g jolt the rev-match
metric measures.  An anti-stall guard shrinks the capacity as the engine nears
idle, so the clutch slips rather than stalling the engine; that is also what
makes a launch from rest work.

Chassis
-------
Wheel force = crank torque x ratio x final drive x efficiency / wheel radius,
minus aero drag, rolling resistance and brake force (``brake_n_per_bar`` is set so
~80 bar is ~1 g on the 1130 kg car).  Lateral: road-wheel angle = steering /
steering ratio, Ackermann yaw rate ``v * tan(delta) / wheelbase`` (first-order
lagged), ``lat = v * yaw``.  A friction circle clamps total acceleration to
``max_grip_g``, scrubbing lateral grip first (push understeer) and then
longitudinal (wheelspin / lock-up).

Sign conventions follow ``obddash.model``: +lon = accelerating, +lat = left turn,
+steering = wheel turned left, +yaw = nose rotating left.

The slow "ECU" signals (temperatures, MAP/MAF, fuel, battery, load, timing,
lambda) drift plausibly from engine state so the UI has something to show.
"""

from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass, field

from obddash.config import VehicleConfig

log = logging.getLogger(__name__)

G = 9.80665  # m/s^2 per g
RPM_TO_RADS = 2.0 * math.pi / 60.0
RADS_TO_RPM = 60.0 / (2.0 * math.pi)
_MAX_SUBSTEP = 0.02  # s; larger dt is split so the clutch / yaw dynamics stay stable
_CREEP_MPS = 0.05  # below this the car is at rest unless driven harder than static friction


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _num(x: object) -> float:
    """``float(x)`` for an untrusted control input; None / NaN / non-numeric -> 0.0.

    ``_clamp`` lets NaN through (every comparison is False), and one NaN pedal value
    would otherwise poison rpm / speed for the rest of the run."""
    try:
        f = float(x)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return 0.0
    return f if f == f else 0.0


# --------------------------------------------------------------------------------------
# Parameters, controls, state
# --------------------------------------------------------------------------------------
@dataclass
class SimParams:
    """Tunables that are *not* in :class:`VehicleConfig` (the config carries only what
    the real-car metrics need).  Defaults describe the "Sim Sportscar"."""

    # -- engine --------------------------------------------------------------------------
    peak_torque_nm: float = 205.0
    peak_torque_rpm: float = 4000.0
    torque_low_falloff: float = 0.5  # fraction of peak lost at 0 rpm (quadratic rise)
    torque_high_falloff: float = 0.17  # fraction of peak lost at redline (quartic fall)
    torque_high_exponent: float = 4.0
    throttle_curve: float = 1.5  # torque = WOT * (1 - (1 - thr)^curve): fat mid-pedal
    engine_inertia_kgm2: float = 0.35  # crank + flywheel + clutch (effective)
    engine_brake_nm_per_rpm: float = 0.02  # closed-throttle drag torque = -k * rpm
    free_rev_rate_rpm_s: float = 3500.0  # WOT rev rate with the driveline open
    idle_gain_nm_per_rpm: float = 0.5  # ECU idle controller
    idle_torque_max_nm: float = 60.0
    limiter_hysteresis_rpm: float = 150.0
    # -- clutch --------------------------------------------------------------------------
    clutch_capacity_nm: float = 280.0  # torque a fully engaged clutch can pass (> peak)
    clutch_bite_point: float = 0.3  # engagement (1 - pedal) below which nothing is transmitted
    # -- chassis -------------------------------------------------------------------------
    brake_n_per_bar: float = 150.0  # 80 bar ~ 1.0 g on 1130 kg x 1.08 inertia factor
    brake_rear_share: float = 0.35  # front/rear pressure balance ~65/35
    max_grip_g: float = 1.1
    yaw_tau_s: float = 0.12  # yaw rate lags the steering input
    track_m: float = 1.5  # for inside/outside wheel speed difference
    air_density_kgm3: float = 1.225
    # -- slow signals --------------------------------------------------------------------
    displacement_l: float = 2.0
    tank_l: float = 45.0
    fuel_start_pct: float = 62.0
    ambient_c: float = 22.0
    baro_kpa: float = 101.0
    coolant_target_c: float = 92.0
    coolant_tau_s: float = 60.0  # 20 -> ~91 C in about four minutes
    oil_tau_s: float = 45.0
    intake_tau_s: float = 15.0
    noise: float = 1.0  # scale for the tiny sensor noise (0 = none)

    def wot_torque(self, rpm: float, redline_rpm: float) -> float:
        """Full-throttle crank torque (Nm) at ``rpm``; smooth, never negative."""
        if rpm <= self.peak_torque_rpm:
            x = (self.peak_torque_rpm - rpm) / self.peak_torque_rpm
            return self.peak_torque_nm * (1.0 - self.torque_low_falloff * x * x)
        span = max(1.0, redline_rpm - self.peak_torque_rpm)
        u = (rpm - self.peak_torque_rpm) / span
        t = self.peak_torque_nm * (1.0 - self.torque_high_falloff * u**self.torque_high_exponent)
        return max(0.0, t)


@dataclass
class Controls:
    """What the driver is doing.  ``clutch``: 0 = pedal up (engaged), 1 = pedal down."""

    throttle_pct: float = 0.0
    brake_bar: float = 0.0
    clutch: float = 0.0
    gear: int = 0  # -1 R, 0 N, 1..n
    steering_deg: float = 0.0  # steering wheel angle, + = left


@dataclass
class VehicleState:
    t: float = 0.0
    speed_mps: float = 0.0
    rpm: float = 0.0
    gear: int = 0
    clutch: float = 0.0  # pedal position 0..1 (1 = down)
    throttle_pct: float = 0.0
    brake_bar: float = 0.0
    brake_front_bar: float = 0.0
    brake_rear_bar: float = 0.0
    steering_deg: float = 0.0
    yaw_rate_dps: float = 0.0
    lat_g: float = 0.0
    lon_g: float = 0.0
    wheel_speeds_kph: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # FL FR RL RR
    coolant_c: float = 20.0
    intake_c: float = 25.0
    oil_temp_c: float = 20.0
    maf_gps: float = 0.0
    map_kpa: float = 101.0
    baro_kpa: float = 101.0
    fuel_pct: float = 62.0
    batt_v: float = 14.1
    load_pct: float = 0.0
    timing_adv_deg: float = 10.0
    lambda_: float = 1.0
    engine_torque_nm: float = 0.0  # net crank torque right now (negative on overrun)
    fuel_rate_lph: float = 0.0
    distance_m: float = 0.0
    # diagnostics (not broadcast): driveline locked, clutch torque
    locked: bool = field(default=False, repr=False)
    clutch_torque_nm: float = field(default=0.0, repr=False)

    @property
    def speed_kph(self) -> float:
        return self.speed_mps * 3.6


# --------------------------------------------------------------------------------------
# Model
# --------------------------------------------------------------------------------------
class VehicleModel:
    """Steps the car forward in time.  ``step(dt, ctrl)`` is cheap enough for 100 Hz on a
    Raspberry Pi; it never raises on odd inputs (they are clamped)."""

    def __init__(
        self, vehicle: VehicleConfig, params: SimParams | None = None, *, seed: int = 0
    ) -> None:
        self.vehicle = vehicle
        self.p = params or SimParams()
        self.rng = random.Random(seed)
        self.state = VehicleState()
        self._locked = False
        self._limiter_cut = False
        self._batt_filt = 14.1
        self._wheel_radius = vehicle.tire_circumference_m / (2.0 * math.pi)
        self._eff_mass = vehicle.mass_kg * vehicle.rotational_inertia_factor
        self._free_torque_cap = (
            self.p.engine_inertia_kgm2 * self.p.free_rev_rate_rpm_s * RPM_TO_RADS
        )
        self.reset()

    # -- helpers --------------------------------------------------------------------------
    def reset(self) -> None:
        v = self.vehicle
        p = self.p
        self.state = VehicleState(
            rpm=v.idle_rpm,
            coolant_c=p.ambient_c - 2.0,
            intake_c=p.ambient_c + 3.0,
            oil_temp_c=p.ambient_c - 2.0,
            map_kpa=p.baro_kpa * 0.3,
            baro_kpa=p.baro_kpa,
            fuel_pct=p.fuel_start_pct,
        )
        self._locked = False
        self._limiter_cut = False
        self._batt_filt = 14.1

    def in_gear(self, gear: int) -> bool:
        return 1 <= gear <= self.vehicle.gear_count()

    def rpm_per_mps(self, gear: int) -> float:
        """Engine rpm per m/s of road speed when locked in ``gear`` (0 if not a forward gear)."""
        rpk = self.vehicle.rpm_per_kph(gear)
        return 0.0 if rpk is None else rpk * 3.6

    def overall_ratio(self, gear: int) -> float:
        if not self.in_gear(gear):
            return 0.0
        return self.vehicle.gear_ratios[gear - 1] * self.vehicle.final_drive

    def engine_torque(self, rpm: float, throttle: float, *, free: bool) -> float:
        """Net crank torque (Nm) at ``rpm`` for pedal fraction ``throttle`` (0..1).
        ``free``: driveline open, so the ECU's neutral torque cap applies."""
        p, v = self.p, self.vehicle
        thr = 1.0 - (1.0 - _clamp(throttle, 0.0, 1.0)) ** p.throttle_curve
        t_brake = -p.engine_brake_nm_per_rpm * rpm
        # rev limiter with hysteresis (fuel cut -> pure engine braking)
        if rpm >= v.redline_rpm:
            self._limiter_cut = True
        elif rpm < v.redline_rpm - p.limiter_hysteresis_rpm:
            self._limiter_cut = False
        if self._limiter_cut:
            torque = t_brake
        else:
            torque = thr * p.wot_torque(rpm, v.redline_rpm) + (1.0 - thr) * t_brake
        # idle controller: a little torque to hold idle when the pedal is up
        if rpm < v.idle_rpm + 150.0:
            torque += _clamp(
                p.idle_gain_nm_per_rpm * (v.idle_rpm + 35.0 - rpm), 0.0, p.idle_torque_max_nm
            )
        if free and torque > self._free_torque_cap:
            torque = self._free_torque_cap
        return torque

    def _noise(self, amp: float) -> float:
        return self.rng.uniform(-amp, amp) * self.p.noise if self.p.noise else 0.0

    # -- stepping -----------------------------------------------------------------------------
    def step(self, dt: float, ctrl: Controls) -> VehicleState:
        """Advance the model by ``dt`` seconds under ``ctrl``.  Large ``dt`` is sub-stepped."""
        if not (dt > 0.0) or dt != dt:  # zero / negative / NaN
            return self.state
        dt = min(dt, 1.0)
        n = max(1, int(math.ceil(dt / _MAX_SUBSTEP)))
        sub = dt / n
        for _ in range(n):
            self._step(sub, ctrl)
        return self.state

    def _step(self, dt: float, ctrl: Controls) -> None:
        p, v, s = self.p, self.vehicle, self.state
        thr = _clamp(_num(ctrl.throttle_pct), 0.0, 100.0) / 100.0
        brake = _clamp(_num(ctrl.brake_bar), 0.0, v.brake.max_pressure_bar * 1.5)
        clutch = _clamp(_num(ctrl.clutch), 0.0, 1.0)
        e = 1.0 - clutch  # clutch engagement
        gear_f = _num(ctrl.gear)
        gear = int(gear_f) if math.isfinite(gear_f) else 0
        steering = _clamp(_num(ctrl.steering_deg), -720.0, 720.0)

        speed = s.speed_mps
        rpm = s.rpm
        in_gear = self.in_gear(gear)
        k_rpm = self.rpm_per_mps(gear)  # engine rpm the driveline imposes per m/s
        driveline_rpm = speed * k_rpm
        ratio = self.overall_ratio(gear)

        # ---- clutch: lock or slip ----------------------------------------------------------
        # Engagement below the bite point transmits nothing; above it the capacity grows
        # linearly to clutch_capacity_nm.  The anti-stall guard shrinks the capacity when the
        # clutch would drag the engine below idle (the engine then slips instead of stalling).
        e_eff = _clamp((e - p.clutch_bite_point) / (1.0 - p.clutch_bite_point), 0.0, 1.0)
        slip_rpm = rpm - driveline_rpm  # + = engine faster than the driveline
        cap = e_eff * p.clutch_capacity_nm if in_gear else 0.0
        if cap > 0.0 and slip_rpm > 0.0:
            cap *= _clamp((rpm - v.idle_rpm) / (0.35 * v.idle_rpm), 0.0, 1.0)
        free = (not in_gear) or e_eff <= 0.0
        t_eng = self.engine_torque(rpm, thr, free=free)

        if cap > 0.0:
            # torque that would sync engine and driveline within this step AND carry t_eng
            needed = t_eng + p.engine_inertia_kgm2 * slip_rpm * RPM_TO_RADS / dt
            # lock only above ~idle (below it the engine would stall: keep slipping)
            if abs(needed) <= cap and driveline_rpm >= v.idle_rpm * 0.85:
                self._locked = True
                t_clutch = t_eng
                rpm = driveline_rpm
            else:
                self._locked = False
                t_clutch = math.copysign(min(cap, abs(needed)), needed)
        else:
            self._locked = False
            t_clutch = 0.0

        if self._locked:
            f_trac = t_eng * ratio * v.drivetrain_efficiency / self._wheel_radius
        else:
            f_trac = t_clutch * ratio * v.drivetrain_efficiency / self._wheel_radius
            rpm += (t_eng - t_clutch) / p.engine_inertia_kgm2 * RADS_TO_RPM * dt
            rpm = _clamp(rpm, 300.0, v.redline_rpm + 400.0)

        # ---- longitudinal ----------------------------------------------------------------
        f_drag = 0.5 * p.air_density_kgm3 * v.cd * v.frontal_area_m2 * speed * speed
        f_roll = v.crr * v.mass_kg * G if speed > _CREEP_MPS else 0.0
        f_brake = brake * p.brake_n_per_bar
        a_max = p.max_grip_g * G
        # Held by the brakes / static rolling resistance.  Below _CREEP_MPS the rolling
        # term is switched off (no chatter around zero), so the stop must be explicit
        # there too, or a car coasting to rest would creep on at ~0.05 m/s forever.
        if speed <= _CREEP_MPS and f_trac <= f_roll + f_brake + v.crr * v.mass_kg * G:
            a_lon = 0.0
            new_speed = 0.0
        else:
            a_lon = (f_trac - f_drag - f_roll - f_brake) / self._eff_mass
            a_lon = _clamp(a_lon, -a_max, a_max)  # lock-up / wheelspin limit
            new_speed = speed + a_lon * dt
            if new_speed < 0.0:
                new_speed = 0.0
                a_lon = -speed / dt

        # ---- lateral (bicycle model, lagged yaw, friction circle) -------------------------
        delta = math.radians(steering / v.steering_ratio)
        yaw_target = speed * math.tan(delta) / v.wheelbase_m  # rad/s, + = left
        yaw = s.yaw_rate_dps * math.pi / 180.0
        yaw += (yaw_target - yaw) * min(1.0, dt / p.yaw_tau_s)
        a_lat = speed * yaw
        lat_allow = math.sqrt(max(0.0, a_max * a_max - a_lon * a_lon))
        if abs(a_lat) > lat_allow:
            a_lat = math.copysign(lat_allow, a_lat)  # understeer: scrub lateral first
            yaw = a_lat / speed if speed > 0.5 else 0.0

        # ---- commit motion ---------------------------------------------------------------
        speed = new_speed
        if self._locked:
            rpm = speed * k_rpm  # exact ratio: the gear estimator relies on this
        s.t += dt
        s.distance_m += speed * dt
        s.speed_mps = speed
        s.rpm = rpm
        s.gear = gear
        s.clutch = clutch
        s.throttle_pct = thr * 100.0
        s.brake_bar = brake
        s.brake_front_bar = brake
        s.brake_rear_bar = brake * p.brake_rear_share / (1.0 - p.brake_rear_share)
        s.steering_deg = steering
        s.yaw_rate_dps = math.degrees(yaw)
        s.lat_g = a_lat / G + self._noise(0.004)
        s.lon_g = a_lon / G + self._noise(0.004)
        s.engine_torque_nm = t_eng
        s.locked = self._locked
        s.clutch_torque_nm = t_clutch

        half_track = 0.5 * p.track_m * yaw  # inside wheels slower by yaw * track / 2
        kph = speed * 3.6
        left = max(0.0, (speed - half_track)) * 3.6
        right = max(0.0, (speed + half_track)) * 3.6
        s.wheel_speeds_kph = (
            left + self._noise(0.06) if kph > 0.3 else 0.0,
            right + self._noise(0.06) if kph > 0.3 else 0.0,
            left + self._noise(0.06) if kph > 0.3 else 0.0,
            right + self._noise(0.06) if kph > 0.3 else 0.0,
        )

        self._step_slow(dt, thr, rpm, speed, t_eng, in_gear and self._locked)

    def _step_slow(
        self, dt: float, thr: float, rpm: float, speed: float, t_eng: float, locked: bool
    ) -> None:
        """Temperatures, airflow, fuel, electrics: cheap first-order models."""
        p, s = self.p, self.state
        thr_eff = 1.0 - (1.0 - thr) ** p.throttle_curve
        # manifold pressure: ~0.3 baro closed throttle, ~baro at WOT; quick lag
        map_target = s.baro_kpa * (0.30 + 0.70 * thr_eff)
        s.map_kpa += (map_target - s.map_kpa) * min(1.0, dt / 0.08)
        # speed density airflow: VE improves with load; rho from MAP and intake temp
        ve = 0.55 + 0.40 * (s.map_kpa / s.baro_kpa)
        rho = s.map_kpa * 1000.0 / (287.05 * (273.15 + s.intake_c))
        disp_m3 = p.displacement_l / 1000.0
        maf = ve * disp_m3 * rpm / 120.0 * rho * 1000.0  # g/s (4-stroke: rpm/120 fills per s)
        rho_wot = s.baro_kpa * 1000.0 / (287.05 * (273.15 + s.intake_c))
        maf_wot = 0.95 * disp_m3 * rpm / 120.0 * rho_wot * 1000.0
        s.maf_gps = maf
        load = _clamp(maf / maf_wot, 0.0, 1.0) if maf_wot > 0 else 0.0
        s.load_pct = load * 100.0
        # fuelling: lambda 1.0 cruising, enriched to 0.85 at WOT; fuel cut on overrun
        lam_target = 1.0 - 0.15 * _clamp((thr - 0.7) / 0.3, 0.0, 1.0)
        s.lambda_ += (lam_target - s.lambda_) * min(1.0, dt / 0.3)
        overrun = locked and thr < 0.02 and rpm > 1500.0
        fuel_gps = 0.0 if overrun else maf / (14.7 * s.lambda_)
        s.fuel_rate_lph = fuel_gps / 745.0 * 3600.0  # petrol ~745 g/L
        s.fuel_pct = max(0.0, s.fuel_pct - fuel_gps * dt / 745.0 / p.tank_l * 100.0)
        # thermals
        cool_target = p.coolant_target_c + 6.0 * load - 2.0 * _clamp(speed / 50.0, 0.0, 1.0)
        s.coolant_c += (cool_target - s.coolant_c) * min(1.0, dt / p.coolant_tau_s)
        oil_target = s.coolant_c + 2.0 + 8.0 * load
        s.oil_temp_c += (oil_target - s.oil_temp_c) * min(1.0, dt / p.oil_tau_s)
        soak = 6.0 * (1.0 - _clamp(speed / 40.0, 0.0, 1.0))  # heat soak when slow
        intake_target = p.ambient_c + 3.0 + 12.0 * load + soak
        s.intake_c += (intake_target - s.intake_c) * min(1.0, dt / p.intake_tau_s)
        # electrics: alternator holds ~14.1 V above idle, a touch less at idle
        batt_target = 14.1 if rpm > 1000.0 else 13.9
        self._batt_filt += (batt_target - self._batt_filt) * min(1.0, dt / 0.5)
        s.batt_v = self._batt_filt + self._noise(0.02)
        # ignition advance: more with rpm, pulled back under load
        s.timing_adv_deg = _clamp(10.0 + 0.0045 * rpm - 14.0 * load, 0.0, 45.0)
