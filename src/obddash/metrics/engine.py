"""MetricsEngine: RawState in, Telemetry out, once per tick (50 Hz).

Per tick (``tick(raw, t)``, ``t`` = monotonic seconds):

1. Read every :class:`~obddash.model.Sig` with
   ``raw.get(name, None, max_age=cfg.stale_s)``; a stale, absent or non-finite
   (NaN / inf) signal is ``None`` and every analyzer copes with ``None``.  Speed, rpm and wheel speeds
   use a 1 s staleness instead: they are fast PIDs, and when the ECU stops answering
   the display should drop to 0 quickly rather than freeze.
2. Speed used by the analyzers: the mean of the wheel-speed signals when at least
   two are present (0.01 km/h resolution at 50 Hz, versus 1 km/h for OBD PID 0x0D),
   unless it keeps disagreeing with the OBD speed (by more than max(10 %, 3 km/h)
   above 20 km/h, for about a second - a mis-scaled signal map; brief differences
   from the slower, quantised OBD speed are normal) - then the OBD speed until they
   agree again for about a second.  The *displayed* speed is the OBD
   speed (it matches the car's speedometer), falling back to the wheel mean.
   Throttle: ``throttle_pct`` (PID 0x11), else ``accel_pedal_pct`` (PID 0x49).
3. Analyzers, in dependency order, all fed the session time ``t - t0``:
   gears -> handling (provides lat/lon g) -> brakes -> shifts -> dyno -> session.
   The shift analyzer only gets lon_g for its jolt measurement when it comes from
   an accelerometer (a speed-derived lon_g is too smooth to see a shunt).
4. Assemble :class:`~obddash.model.Telemetry`: ``boost = map - baro`` when both
   exist; alerts with a little hysteresis so they do not flicker:

   * ``COOLANT HIGH``: coolant >= ``coolant_alert_c`` (clears 2 C below)
   * ``OIL HOT``: oil >= ``oil_alert_c`` (clears 2 C below)
   * ``BATT LOW``: module voltage < ``batt_low_v`` (clears 0.3 V above)
   * ``NO ECU``: the runtime has said ``conn.ecu`` is False for 2 s (only once
     :meth:`set_connection` has been called; the grace covers start-up, before the
     first OBD answer)

Events (``drain_events()``): ``shift.scored`` (a scored downshift, the
ShiftResult as a dict), ``dyno.started`` (armed pulls), ``dyno.done`` (with peaks,
or ``ok: false`` and a reason), ``brake.event`` (the BrakeEvent), ``alert`` (when a
new alert appears).  The queue is capped so an undrained engine cannot grow.

Commands: ``dyno.arm`` / ``dyno.cancel`` / ``dyno.clear``, ``session.reset``
(every analyzer and the session clock), ``brake.reset_peak``, ``shift.reset``.

Robustness: ``tick()`` never raises.  Each analyzer call is wrapped; an exception
is logged (with traceback) at most once a minute per analyzer and the tick carries
on with that analyzer's last state.  A clock that jumps backwards (replay restart)
resets the session.  Public methods take a lock, so the runtime may call
``command()`` from another thread.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import threading
import time
from typing import Any

from obddash.config import MetricsConfig, VehicleConfig
from obddash.model import (
    Brake,
    Connection,
    Dyno,
    Event,
    GForces,
    Handling,
    RawState,
    Session,
    Shift,
    Sig,
    Telemetry,
    Vehicle,
)

from .brakes import BrakeAnalyzer
from .dyno import VirtualDyno
from .gears import GearEstimate, GearEstimator
from .handling import HandlingAnalyzer
from .session import SessionTracker
from .shifts import ShiftAnalyzer

log = logging.getLogger(__name__)

FAST_STALE_S = 1.0
ERROR_LOG_INTERVAL_S = 60.0
MAX_EVENTS = 256
WHEEL_SPEED_MAX_DISAGREE = 0.10
WHEEL_SPEED_MAX_DISAGREE_KPH = 3.0
WHEEL_SPEED_CHECK_KPH = 20.0
WHEEL_DISTRUST_TICKS = 50  # ~1 s of disagreement before falling back to OBD speed
NO_ECU_DEBOUNCE_S = 2.0
CLOCK_BACKWARDS_S = 1.0
COOLANT_HYST_C = 2.0
OIL_HYST_C = 2.0
BATT_HYST_V = 0.3

_WHEELS = (
    Sig.WHEEL_SPEED_FL_KPH,
    Sig.WHEEL_SPEED_FR_KPH,
    Sig.WHEEL_SPEED_RL_KPH,
    Sig.WHEEL_SPEED_RR_KPH,
)


def _r(x: float | None, nd: int) -> float | None:
    return None if x is None else round(x, nd)


def _bool(x: float | None) -> bool | None:
    return None if x is None else x >= 0.5


class MetricsEngine:
    def __init__(self, cfg: MetricsConfig, vehicle: VehicleConfig) -> None:
        self.cfg = cfg
        self.vehicle = vehicle
        self._lock = threading.RLock()
        self._events: list[Event] = []
        self._conn = Connection()
        self._conn_set = False
        self._err_logged: dict[str, float] = {}
        self._last = Telemetry()
        self._build()

    # -- lifecycle -----------------------------------------------------------------------
    def _build(self) -> None:
        cfg, veh = self.cfg, self.vehicle
        self.gears = GearEstimator(veh)
        self.handling = HandlingAnalyzer(cfg.handling, veh)
        self.brakes = BrakeAnalyzer(cfg.brake, veh.brake)
        self.shifts = ShiftAnalyzer(cfg.revmatch, veh)
        self.dyno = VirtualDyno(cfg.dyno, veh)
        self.session = SessionTracker()
        self._est = GearEstimate()
        self._t0: float | None = None
        self._last_t: float | None = None
        self._alerts: list[str] = []
        self._wheel_mismatch_logged = False
        self._wheel_bad = 0  # leaky count of ticks where wheels and OBD speed disagree
        self._ecu_down_since: float | None = None
        self._rpm_ts: float | None = None
        # last good snapshots, reused when an analyzer raises
        self._g = GForces()
        self._handling = Handling()
        self._brake = Brake()
        self._dyno = Dyno()
        self._session = Session()
        self._shift = Shift()

    def set_connection(self, conn: Connection) -> None:
        with self._lock:
            self._conn = dataclasses.replace(conn)
            self._conn_set = True

    def drain_events(self) -> list[Event]:
        with self._lock:
            ev, self._events = self._events, []
            return ev

    def command(self, cmd: str, args: dict | None = None) -> bool:
        with self._lock:
            c = (cmd or "").strip().lower()
            if c == "dyno.arm":
                self.dyno.arm()
            elif c == "dyno.cancel":
                self.dyno.cancel()
            elif c == "dyno.clear":
                self.dyno.clear()
            elif c == "session.reset":
                self._build()
            elif c == "brake.reset_peak":
                self.brakes.reset_peak()
            elif c == "shift.reset":
                self.shifts.reset()
            else:
                log.debug("unknown command %r", cmd)
                return False
            return True

    # -- tick ----------------------------------------------------------------------------
    def tick(self, raw: RawState, t: float) -> Telemetry:
        with self._lock:
            try:
                tel = self._tick(raw, t)
            except Exception:  # never take the tick loop down
                self._log_error("tick")
                tel = self._last
            self._last = tel
            return tel

    def _tick(self, raw: RawState, t: float) -> Telemetry:
        if self._last_t is not None and t < self._last_t - CLOCK_BACKWARDS_S:
            log.warning("clock went backwards by %.1f s; resetting the session", self._last_t - t)
            self._build()
        self._last_t = t
        if self._t0 is None:
            self._t0 = t
        st = t - self._t0
        cfg = self.cfg
        stale = cfg.stale_s

        def get(name: str, max_age: float = stale) -> float | None:
            v = raw.get(name, None, max_age=max_age)
            # NaN / inf (a corrupt frame, a replay file) would poison every integral and
            # peak for the rest of the session: treat it as missing
            return v if v is None or math.isfinite(v) else None

        # -- read ------------------------------------------------------------------------
        speed_obd = get(Sig.SPEED_KPH, FAST_STALE_S)
        rpm = get(Sig.RPM, FAST_STALE_S)
        wheels = [w for w in (get(s, FAST_STALE_S) for s in _WHEELS) if w is not None]
        wheel_kph = sum(wheels) / len(wheels) if len(wheels) >= 2 else None
        speed = self._metrics_speed(speed_obd, wheel_kph)
        throttle = get(Sig.THROTTLE_PCT)
        if throttle is None:
            throttle = get(Sig.ACCEL_PEDAL_PCT)
        brake_sw = _bool(get(Sig.BRAKE_SWITCH))
        clutch_sw = _bool(get(Sig.CLUTCH_SWITCH))
        gear_raw = get(Sig.GEAR)
        gear_sig = int(round(gear_raw)) if gear_raw is not None else None
        steering = get(Sig.STEERING_DEG)
        yaw = get(Sig.YAW_RATE_DPS)
        acc_lat = get(Sig.ACC_LAT_G)
        acc_lon = get(Sig.ACC_LON_G)
        acc_vert = get(Sig.ACC_VERT_G)
        p_sys = get(Sig.BRAKE_PRESSURE_BAR)
        p_front = get(Sig.BRAKE_PRESSURE_FRONT_BAR)
        p_rear = get(Sig.BRAKE_PRESSURE_REAR_BAR)
        baro = get(Sig.BARO_KPA)
        intake = get(Sig.INTAKE_C)
        map_kpa = get(Sig.MAP_KPA)
        coolant = get(Sig.COOLANT_C)
        oil = get(Sig.OIL_TEMP_C)
        batt = get(Sig.CONTROL_MODULE_V)
        fuel_rate = get(Sig.FUEL_RATE_LPH)
        torque_pct = get(Sig.ENGINE_TORQUE_PCT)
        ref_torque = get(Sig.ENGINE_REF_TORQUE_NM)

        # -- analyzers -------------------------------------------------------------------
        # a new rpm sample this tick?  (an OBD rpm PID answers at ~15 Hz; see gears.py)
        rv = raw.snapshot().get(Sig.RPM)
        rpm_ts = rv.t if rv is not None else None
        fresh = rpm_ts is not None and rpm_ts != self._rpm_ts
        self._rpm_ts = rpm_ts
        est = self._safe(
            "gears", self.gears.update, st, rpm, speed, clutch_sw, gear_sig, fresh=fresh
        )
        if est is not None:
            self._est = est
        est = self._est

        h = self.handling
        if self._safe_ok(
            "handling", h.update, st, speed, acc_lat, acc_lon, acc_vert, steering, yaw
        ):
            self._g = h.g_snapshot()
            self._handling = h.snapshot()
        lat_g, lon_g = h.lat_g, h.lon_g

        b = self.brakes
        bev = self._safe(
            "brakes", b.update, st, p_sys, p_front, p_rear, brake_sw, lon_g, lat_g, speed
        )
        if bev is not None:
            self._emit("brake.event", dataclasses.asdict(bev), st)
        self._brake = self._safe("brakes", b.snapshot) or self._brake

        sa = self.shifts
        res = self._safe(
            "shifts",
            sa.update,
            st,
            rpm,
            speed,
            throttle,
            b.active,
            clutch_sw,
            est,
            lon_g if h.lon_from_imu else None,
            brake_bar=b.pressure_bar if b.source != "none" else None,
            rpm_fresh=fresh,
        )
        if res is not None and res.kind == "down" and res.grade != "-":
            self._emit("shift.scored", dataclasses.asdict(res), st)
        self._shift = (
            self._safe(
                "shifts",
                lambda: Shift(
                    state=sa.state,
                    target_rpm=sa.target_rpm,
                    target_gear=sa.target_gear,
                    shift_light=sa.shift_light,
                    last=sa.last,
                    session=sa.session_snapshot(),
                ),
            )
            or self._shift
        )

        d = self.dyno
        self._safe(
            "dyno", d.update, st, rpm, speed, throttle, est, baro, intake, torque_pct, ref_torque
        )
        for name, data in d.drain_events():
            self._emit(name, data, st)
        self._dyno = self._safe("dyno", d.snapshot) or self._dyno

        if self._safe_ok("session", self.session.update, st, speed, rpm, fuel_rate):
            self._session = self.session.snapshot()

        # -- assemble --------------------------------------------------------------------
        boost = map_kpa - baro if map_kpa is not None and baro is not None else None
        if speed_obd is not None:
            speed_display = speed_obd
        elif wheel_kph is not None:
            speed_display = wheel_kph
        else:
            speed_display = 0.0
        veh = Vehicle(
            speed_kph=round(speed_display, 2),
            rpm=round(rpm, 1) if rpm is not None else 0.0,
            gear=est.gear,
            gear_conf=round(est.conf, 3),
            gear_source=est.source,
            throttle_pct=round(throttle, 1) if throttle is not None else 0.0,
            load_pct=_r(get(Sig.ENGINE_LOAD_PCT), 1),
            coolant_c=_r(coolant, 1),
            intake_c=_r(intake, 1),
            oil_temp_c=_r(oil, 1),
            maf_gps=_r(get(Sig.MAF_GPS), 2),
            map_kpa=_r(map_kpa, 1),
            boost_kpa=_r(boost, 1),
            baro_kpa=_r(baro, 1),
            fuel_pct=_r(get(Sig.FUEL_LEVEL_PCT), 1),
            fuel_rate_lph=_r(fuel_rate, 2),
            batt_v=_r(batt, 2),
            timing_adv_deg=_r(get(Sig.TIMING_ADV_DEG), 1),
            lambda_=_r(get(Sig.LAMBDA), 3),
            steering_deg=_r(steering, 1),
            yaw_dps=_r(yaw, 2),
            brake_switch=brake_sw if brake_sw is not None else b.active,
            clutch_switch=clutch_sw,
            ambient_c=_r(get(Sig.AMBIENT_C), 1),
        )
        alerts = self._update_alerts(coolant, oil, batt, t, st)
        return Telemetry(
            t=round(st, 3),
            ts=time.time(),
            conn=dataclasses.replace(self._conn),
            veh=veh,
            g=self._g,
            brake=self._brake,
            shift=self._shift,
            dyno=self._dyno,
            handling=self._handling,
            session=self._session,
            alerts=alerts,
        )

    # -- helpers -------------------------------------------------------------------------
    def _metrics_speed(self, speed_obd: float | None, wheel_kph: float | None) -> float | None:
        if wheel_kph is None:
            return speed_obd
        if speed_obd is None:
            return wheel_kph
        limit = max(WHEEL_SPEED_MAX_DISAGREE_KPH, WHEEL_SPEED_MAX_DISAGREE * speed_obd)
        if speed_obd >= WHEEL_SPEED_CHECK_KPH and abs(wheel_kph - speed_obd) > limit:
            self._wheel_bad = min(2 * WHEEL_DISTRUST_TICKS, self._wheel_bad + 1)
        else:
            self._wheel_bad = max(0, self._wheel_bad - 1)
        if self._wheel_bad >= WHEEL_DISTRUST_TICKS:
            if not self._wheel_mismatch_logged:
                self._wheel_mismatch_logged = True
                log.warning(
                    "wheel speeds (%.1f km/h) disagree with OBD speed (%.1f km/h); "
                    "check the wheel-speed scale in the vehicle file - using OBD speed",
                    wheel_kph,
                    speed_obd,
                )
            return speed_obd
        return wheel_kph

    def _update_alerts(
        self, coolant: float | None, oil: float | None, batt: float | None, t: float, st: float
    ) -> list[str]:
        cfg = self.cfg
        prev = self._alerts
        active: list[str] = []

        def hyst(name: str, on: bool, off: bool) -> None:
            # raise when `on`; once raised, stay until `off` (hysteresis)
            if on or (name in prev and not off):
                active.append(name)

        if coolant is not None:
            lim = cfg.coolant_alert_c
            hyst("COOLANT HIGH", coolant >= lim, coolant < lim - COOLANT_HYST_C)
        if oil is not None:
            lim = cfg.oil_alert_c
            hyst("OIL HOT", oil >= lim, oil < lim - OIL_HYST_C)
        if batt is not None:
            lim = cfg.batt_low_v
            hyst("BATT LOW", batt < lim, batt > lim + BATT_HYST_V)
        if self._conn_set and not self._conn.ecu:
            if self._ecu_down_since is None:
                self._ecu_down_since = t
            if t - self._ecu_down_since >= NO_ECU_DEBOUNCE_S:
                active.append("NO ECU")
        else:
            self._ecu_down_since = None
        for name in active:
            if name not in prev:
                self._emit("alert", {"alert": name, "alerts": list(active)}, st)
        self._alerts = active
        return list(active)

    def _emit(self, name: str, data: dict[str, Any], st: float) -> None:
        self._events.append(Event(name=name, data=data, t=round(st, 3)))
        if len(self._events) > MAX_EVENTS:
            del self._events[: len(self._events) - MAX_EVENTS]

    def _safe(self, name: str, fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except Exception:
            self._log_error(name)
            return None

    def _safe_ok(self, name: str, fn, *args, **kwargs) -> bool:
        try:
            fn(*args, **kwargs)
            return True
        except Exception:
            self._log_error(name)
            return False

    def _log_error(self, name: str) -> None:
        now = time.monotonic()
        last = self._err_logged.get(name)
        if last is None or now - last >= ERROR_LOG_INTERVAL_S:
            self._err_logged[name] = now
            log.exception("metrics: %s failed (further errors suppressed for 60 s)", name)
