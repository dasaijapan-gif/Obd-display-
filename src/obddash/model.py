"""Shared data model.

This module is THE contract between the independent parts of obddash:

    car -> [bus readers / IMU / simulator]  --RawState-->  [MetricsEngine]  --Telemetry-->  [server -> UI]

* Anything that reads the car publishes values into a :class:`RawState` under the
  canonical names in :class:`Sig`.  The metrics engine only ever reads those names,
  so a new data source (another CAN signal map, a USB ELM327, a replay file) needs
  no changes downstream.
* The metrics engine produces a :class:`Telemetry` snapshot every tick.  The server
  serialises it with :meth:`Telemetry.to_dict` and pushes it to the UI over a
  WebSocket.  ``docs/architecture.md`` shows the resulting JSON.

Sign conventions (SAE J670, vehicle axes): x forward, y left, z up.

* ``acc_lon_g`` positive = accelerating forward, negative = braking.
* ``acc_lat_g`` positive = left turn (acceleration toward the driver's left).
* ``steering_deg`` positive = steering wheel turned left (counter-clockwise).
* ``yaw_rate_dps`` positive = nose rotating left (counter-clockwise from above).
* ``gear``: -1 reverse, 0 neutral/unknown, 1..n forward gears.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any


# --------------------------------------------------------------------------------------
# Canonical raw signal names
# --------------------------------------------------------------------------------------
class Sig:
    """Canonical names for raw signals.  Units are in the name."""

    # --- standard OBD-II mode 01 PIDs -------------------------------------------------
    SPEED_KPH = "speed_kph"  # PID 0x0D
    RPM = "rpm"  # PID 0x0C
    THROTTLE_PCT = "throttle_pct"  # PID 0x11 absolute throttle position
    ACCEL_PEDAL_PCT = "accel_pedal_pct"  # PID 0x49 accelerator pedal position D
    ENGINE_LOAD_PCT = "engine_load_pct"  # PID 0x04
    COOLANT_C = "coolant_c"  # PID 0x05
    INTAKE_C = "intake_c"  # PID 0x0F
    OIL_TEMP_C = "oil_temp_c"  # PID 0x5C
    MAF_GPS = "maf_gps"  # PID 0x10 grams/second
    MAP_KPA = "map_kpa"  # PID 0x0B intake manifold absolute pressure
    BARO_KPA = "baro_kpa"  # PID 0x33
    FUEL_LEVEL_PCT = "fuel_level_pct"  # PID 0x2F
    FUEL_RATE_LPH = "fuel_rate_lph"  # PID 0x5E
    CONTROL_MODULE_V = "control_module_v"  # PID 0x42
    TIMING_ADV_DEG = "timing_adv_deg"  # PID 0x0E
    LAMBDA = "lambda"  # PID 0x44 commanded equivalence ratio
    ENGINE_TORQUE_PCT = "engine_torque_pct"  # PID 0x62 actual engine torque, % of reference
    ENGINE_REF_TORQUE_NM = "engine_ref_torque_nm"  # PID 0x63
    AMBIENT_C = "ambient_c"  # PID 0x46

    # --- vehicle specific CAN broadcast signals (see config/vehicles/*.yaml) ----------
    BRAKE_SWITCH = "brake_switch"  # 0 / 1
    CLUTCH_SWITCH = "clutch_switch"  # 0 / 1
    BRAKE_PRESSURE_BAR = "brake_pressure_bar"  # master cylinder / system pressure
    BRAKE_PRESSURE_FRONT_BAR = "brake_pressure_front_bar"
    BRAKE_PRESSURE_REAR_BAR = "brake_pressure_rear_bar"
    STEERING_DEG = "steering_deg"  # steering WHEEL angle
    YAW_RATE_DPS = "yaw_rate_dps"
    WHEEL_SPEED_FL_KPH = "wheel_speed_fl_kph"
    WHEEL_SPEED_FR_KPH = "wheel_speed_fr_kph"
    WHEEL_SPEED_RL_KPH = "wheel_speed_rl_kph"
    WHEEL_SPEED_RR_KPH = "wheel_speed_rr_kph"
    GEAR = "gear"  # only if the car broadcasts it; otherwise estimated

    # --- IMU (or simulator) ----------------------------------------------------------
    ACC_LAT_G = "acc_lat_g"
    ACC_LON_G = "acc_lon_g"
    ACC_VERT_G = "acc_vert_g"

    @classmethod
    def all(cls) -> list[str]:
        return [v for k, v in vars(cls).items() if k.isupper() and isinstance(v, str)]


# --------------------------------------------------------------------------------------
# RawState: latest value of every raw signal, with timestamps
# --------------------------------------------------------------------------------------
@dataclass(slots=True)
class RawValue:
    value: float
    t: float  # time.monotonic() when the value was received


class RawState:
    """Thread-safe "latest value" store for raw signals.

    Producers (CAN reader, IMU thread, simulator, replayer) call :meth:`set` from any
    thread.  The metrics engine calls :meth:`snapshot` once per tick.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._vals: dict[str, RawValue] = {}
        self._counts: dict[str, int] = {}

    # -- producers ----------------------------------------------------------------------
    def set(self, name: str, value: float, t: float | None = None) -> None:
        if value is None:
            return
        if t is None:
            t = self._clock()
        with self._lock:
            self._vals[name] = RawValue(float(value), t)
            self._counts[name] = self._counts.get(name, 0) + 1

    def update(self, values: dict[str, float], t: float | None = None) -> None:
        if t is None:
            t = self._clock()
        with self._lock:
            for name, value in values.items():
                if value is None:
                    continue
                self._vals[name] = RawValue(float(value), t)
                self._counts[name] = self._counts.get(name, 0) + 1

    # -- consumers ----------------------------------------------------------------------
    def get(self, name: str, default: float | None = None, max_age: float | None = None):
        with self._lock:
            rv = self._vals.get(name)
        if rv is None:
            return default
        if max_age is not None and (self._clock() - rv.t) > max_age:
            return default
        return rv.value

    def age(self, name: str) -> float | None:
        with self._lock:
            rv = self._vals.get(name)
        return None if rv is None else self._clock() - rv.t

    def has(self, name: str) -> bool:
        with self._lock:
            return name in self._vals

    def snapshot(self) -> dict[str, RawValue]:
        with self._lock:
            return dict(self._vals)

    def counts(self) -> dict[str, int]:
        with self._lock:
            return dict(self._counts)

    def clear(self) -> None:
        with self._lock:
            self._vals.clear()
            self._counts.clear()


# --------------------------------------------------------------------------------------
# Telemetry: the processed snapshot sent to the UI
# --------------------------------------------------------------------------------------
@dataclass
class Connection:
    bus: bool = False  # CAN interface is up and frames are flowing
    ecu: bool = False  # the ECU answered an OBD request within the last ~2 s
    obd: bool = False  # OBD-II polling is enabled (False: broadcast-only, replay, none)
    imu: bool = False
    source: str = "none"  # "can" | "sim" | "replay" | "none"
    pid_hz: float = 0.0  # OBD responses per second
    frame_hz: float = 0.0  # total CAN frames per second seen


@dataclass
class Vehicle:
    speed_kph: float = 0.0
    rpm: float = 0.0
    gear: int = 0  # -1 R, 0 N/unknown, 1..n
    gear_conf: float = 0.0  # 0..1 confidence in the gear estimate
    gear_source: str = "none"  # "can" | "ratio" | "none"
    throttle_pct: float = 0.0
    load_pct: float | None = None
    coolant_c: float | None = None
    intake_c: float | None = None
    oil_temp_c: float | None = None
    maf_gps: float | None = None
    map_kpa: float | None = None
    boost_kpa: float | None = None  # map - baro (relative), None if unknown
    baro_kpa: float | None = None
    fuel_pct: float | None = None
    fuel_rate_lph: float | None = None
    batt_v: float | None = None
    timing_adv_deg: float | None = None
    lambda_: float | None = None  # serialised as "lambda"
    steering_deg: float | None = None
    yaw_dps: float | None = None
    brake_switch: bool = False
    clutch_switch: bool | None = None
    ambient_c: float | None = None


@dataclass
class GForces:
    lat: float = 0.0
    lon: float = 0.0
    vert: float = 0.0
    total: float = 0.0  # sqrt(lat^2 + lon^2)
    peak_lat: float = 0.0  # absolute
    peak_accel: float = 0.0  # max positive lon
    peak_brake: float = 0.0  # max |negative lon|
    source: str = "none"  # "imu" | "derived" | "none"
    # recent (lat, lon) points for the friction-circle trail, oldest first, ~last 3 s
    trail: list[list[float]] = field(default_factory=list)


@dataclass
class BrakeEvent:
    t_start: float
    duration_s: float
    peak_bar: float
    entry_kph: float
    exit_kph: float
    avg_decel_g: float
    trail_pct: float  # share of the event with meaningful lateral G (trail braking)
    smoothness: float  # 0..100


@dataclass
class Brake:
    pressure_bar: float = 0.0
    pressure_pct: float = 0.0  # pressure / vehicle.brake.max_pressure_bar * 100
    front_bar: float | None = None
    rear_bar: float | None = None
    active: bool = False
    peak_bar: float = 0.0  # session peak
    source: str = "none"  # "can" | "estimate" | "none"
    trail_pct: float = 0.0  # current event, live
    events: list[BrakeEvent] = field(default_factory=list)  # most recent last, capped
    # recent pressure samples [t, bar] for the live trace, oldest first, ~last 8 s
    trace: list[list[float]] = field(default_factory=list)
    score: float = 0.0  # session brake score 0..100
    count: int = 0


@dataclass
class ShiftResult:
    t: float
    from_gear: int
    to_gear: int
    kind: str  # "down" | "up"
    duration_s: float  # clutch-in (or ratio unlock) to re-engagement
    target_rpm: float  # rpm the engine *should* have been at for the new gear
    actual_rpm: float  # rpm at the moment of re-engagement
    error_pct: float  # (actual - target) / target * 100, signed
    blip: bool  # a throttle blip was seen while disengaged
    heel_toe: bool  # blip while braking
    jolt_g: float | None  # longitudinal jerk spike at engagement, if IMU present
    score: float  # 0..100
    grade: str  # "S" | "A" | "B" | "C" | "D"
    # short traces around the shift for the UI: [[t, rpm], ...], [[t, throttle], ...]
    rpm_trace: list[list[float]] = field(default_factory=list)
    throttle_trace: list[list[float]] = field(default_factory=list)
    brake_trace: list[list[float]] = field(default_factory=list)


@dataclass
class ShiftSession:
    count: int = 0
    downshifts: int = 0
    upshifts: int = 0
    avg_score: float = 0.0  # downshifts only
    best_score: float = 0.0
    heel_toe_count: int = 0
    history: list[float] = field(default_factory=list)  # last N downshift scores


@dataclass
class Shift:
    state: str = "idle"  # "idle" | "disengaged" | "blip" | "engaged"
    target_rpm: float | None = None  # live: rpm needed for gear-1 at current speed
    target_gear: int | None = None
    shift_light: bool = False
    last: ShiftResult | None = None
    session: ShiftSession = field(default_factory=ShiftSession)


@dataclass
class DynoPoint:
    rpm: float
    power_kw: float
    torque_nm: float
    ecu_torque_nm: float | None = None  # from PID 0x62/0x63 when the ECU reports torque


@dataclass
class DynoRun:
    id: int
    t: float
    gear: int
    rpm_start: float
    rpm_end: float
    peak_power_kw: float
    peak_power_rpm: float
    peak_torque_nm: float
    peak_torque_rpm: float
    correction: float  # SAE J1349-style correction factor applied (1.0 = none)
    curve: list[DynoPoint] = field(default_factory=list)
    peak_crank_kw: float = 0.0  # peak wheel power / drivetrain efficiency


@dataclass
class Dyno:
    state: str = "idle"  # "idle" | "armed" | "pulling" | "done" | "rejected"
    message: str = ""  # human hint, e.g. "Hold WOT in 3rd from 2500 rpm"
    gear: int = 0
    live_rpm: float = 0.0
    live_power_kw: float = 0.0
    live_torque_nm: float = 0.0
    progress: float = 0.0  # 0..1 of the rpm span covered so far
    curve: list[DynoPoint] = field(default_factory=list)  # current / last pull
    peak_power_kw: float = 0.0
    peak_power_rpm: float = 0.0
    peak_torque_nm: float = 0.0
    peak_torque_rpm: float = 0.0
    runs: list[DynoRun] = field(default_factory=list)  # completed pulls this session
    wheel_hp: float = 0.0  # peak power expressed in hp for the readout


@dataclass
class Handling:
    score: float = 0.0  # 0..100 session handling score
    smoothness: float = 0.0  # 0..100
    balance: float | None = None  # -1 understeer .. 0 neutral .. +1 oversteer
    friction_used_pct: float = 0.0  # current |g| / peak envelope
    max_lat_g: float = 0.0
    corner_count: int = 0


@dataclass
class Session:
    duration_s: float = 0.0
    distance_km: float = 0.0
    max_speed_kph: float = 0.0
    max_rpm: float = 0.0
    fuel_used_l: float | None = None


@dataclass
class Telemetry:
    """The processed snapshot the UI renders.  One per metrics tick."""

    t: float = 0.0  # seconds since session start
    ts: float = 0.0  # wall-clock epoch seconds
    conn: Connection = field(default_factory=Connection)
    veh: Vehicle = field(default_factory=Vehicle)
    g: GForces = field(default_factory=GForces)
    brake: Brake = field(default_factory=Brake)
    shift: Shift = field(default_factory=Shift)
    dyno: Dyno = field(default_factory=Dyno)
    handling: Handling = field(default_factory=Handling)
    session: Session = field(default_factory=Session)
    alerts: list[str] = field(default_factory=list)  # short strings, e.g. "COOLANT HIGH"

    def to_dict(self) -> dict[str, Any]:
        return _clean(asdict(self))


_RENAMES = {"lambda_": "lambda"}


def _clean(obj: Any) -> Any:
    """Make a dataclass dict JSON friendly: rename reserved words, NaN/inf -> None,
    round floats to keep WebSocket payloads small."""
    if isinstance(obj, dict):
        return {_RENAMES.get(k, k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_clean(v) for v in obj]
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return round(obj, 4)
    return obj


# --------------------------------------------------------------------------------------
# Events: discrete things the UI may want to react to (flash a score, switch page)
# --------------------------------------------------------------------------------------
@dataclass
class Event:
    name: str  # "shift.scored" | "dyno.started" | "dyno.done" | "brake.event" | "alert"
    data: dict[str, Any] = field(default_factory=dict)
    t: float = 0.0
