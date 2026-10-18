"""Configuration models (pydantic) and loaders.

Two YAML files drive the app:

* an *app* config (``config/default.yaml``): CAN interface, server, IMU, metrics tuning
* a *vehicle* config (``config/vehicles/*.yaml``): physical parameters used by the gear
  estimator and the virtual dyno, plus the car-specific CAN broadcast signals
  (brake pressure, steering angle, ...) that standard OBD-II does not expose.

Signal bit layout follows the DBC convention (what ``cantools`` uses):

* ``byte_order: little_endian`` (Intel): ``start_bit`` is the bit number of the
  signal's least significant bit, counting bit 0 = LSB of byte 0, bit 8 = LSB of byte 1.
* ``byte_order: big_endian`` (Motorola): ``start_bit`` is the bit number of the
  signal's MOST significant bit, in the same bit numbering (bit 7 = MSB of byte 0).

For the common case of a whole-byte or two-byte field you can instead give ``byte``
(0-based start byte) and ``length`` in bits; big-endian is assumed for multi-byte.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field, field_validator, model_validator


# --------------------------------------------------------------------------------------
# CAN / OBD
# --------------------------------------------------------------------------------------
class PidRequest(BaseModel):
    """One mode-01 PID to poll and how fast.  Rates are relative priorities:
    ``fast`` PIDs are requested round-robin as fast as the ECU answers, ``medium``
    about every 250 ms, ``slow`` about every 2 s."""

    pid: int
    rate: Literal["fast", "medium", "slow"] = "medium"

    @field_validator("pid", mode="before")
    @classmethod
    def _hex_ok(cls, v):
        return int(v, 0) if isinstance(v, str) else v


DEFAULT_PIDS: list[PidRequest] = [
    PidRequest(pid=0x0C, rate="fast"),  # rpm
    PidRequest(pid=0x0D, rate="fast"),  # speed
    PidRequest(pid=0x11, rate="fast"),  # throttle
    PidRequest(pid=0x04, rate="medium"),  # load
    PidRequest(pid=0x0B, rate="medium"),  # MAP
    PidRequest(pid=0x10, rate="medium"),  # MAF
    PidRequest(pid=0x49, rate="medium"),  # accel pedal
    PidRequest(pid=0x62, rate="medium"),  # actual torque %
    PidRequest(pid=0x05, rate="slow"),  # coolant
    PidRequest(pid=0x0F, rate="slow"),  # intake temp
    PidRequest(pid=0x33, rate="slow"),  # baro
    PidRequest(pid=0x2F, rate="slow"),  # fuel level
    PidRequest(pid=0x42, rate="slow"),  # module voltage
    PidRequest(pid=0x5C, rate="slow"),  # oil temp
    PidRequest(pid=0x63, rate="slow"),  # reference torque
    PidRequest(pid=0x5E, rate="slow"),  # fuel rate
    PidRequest(pid=0x46, rate="slow"),  # ambient
]


class BusConfig(BaseModel):
    interface: str = "socketcan"  # python-can interface: socketcan | virtual | slcan | ...
    channel: str = "can0"
    bitrate: int = 500_000
    receive_own_messages: bool = False
    obd_enabled: bool = True  # actively poll OBD-II PIDs
    obd_request_id: int = 0x7DF  # functional request (11-bit)
    obd_response_ids: list[int] = Field(default_factory=lambda: list(range(0x7E8, 0x7F0)))
    obd_extended_ids: bool = False  # 29-bit OBD (0x18DB33F1 / 0x18DAF1xx) if True
    obd_timeout_s: float = 0.25  # per-request response timeout
    obd_max_rate_hz: float = 60.0  # never send requests faster than this
    obd_multi_pid: bool = (
        False  # request up to 6 PIDs per frame (ISO 15031-5); falls back if unsupported
    )
    passive_enabled: bool = True  # decode vehicle broadcast signals
    log_unknown_ids: bool = False

    @field_validator("obd_request_id", mode="before")
    @classmethod
    def _hex_ok(cls, v):
        return int(v, 0) if isinstance(v, str) else v

    @field_validator("obd_response_ids", mode="before")
    @classmethod
    def _hex_list_ok(cls, v):
        return [int(x, 0) if isinstance(x, str) else x for x in v]


class SignalConfig(BaseModel):
    """A vehicle-specific broadcast CAN signal, DBC-style."""

    name: str  # one of model.Sig names (free names are allowed but ignored downstream)
    id: int  # arbitration id
    extended: bool = False
    byte: int | None = None  # shortcut: start byte (0-based) for byte-aligned fields
    start_bit: int | None = None  # DBC start bit (see module docstring)
    length: int = 8  # bits
    byte_order: Literal["big_endian", "little_endian"] = "big_endian"
    signed: bool = False
    scale: float = 1.0
    offset: float = 0.0
    min: float | None = None  # values outside [min, max] are dropped as corrupt
    max: float | None = None
    unit: str = ""
    # optional mux: only decode when byte `mux_byte` == `mux_value`
    mux_byte: int | None = None
    mux_value: int | None = None

    @field_validator("id", mode="before")
    @classmethod
    def _hex_ok(cls, v):
        return int(v, 0) if isinstance(v, str) else v

    @model_validator(mode="after")
    def _resolve_start(self):
        if self.start_bit is None:
            if self.byte is None:
                raise ValueError(f"signal {self.name}: give start_bit or byte")
            if self.byte_order == "big_endian":
                self.start_bit = self.byte * 8 + 7  # MSB of the start byte
            else:
                self.start_bit = self.byte * 8  # LSB of the start byte
        if not 1 <= self.length <= 64:
            raise ValueError(f"signal {self.name}: length must be 1..64 bits")
        return self


# --------------------------------------------------------------------------------------
# Vehicle
# --------------------------------------------------------------------------------------
class BrakeConfig(BaseModel):
    max_pressure_bar: float = 100.0  # full-scale for the % readout and the gauge
    active_threshold_bar: float = 2.0  # pressure above which "braking" is true
    # When no pressure signal exists, estimate pressure from deceleration:
    #   pressure_bar ~= estimate_bar_per_g * |decel_g|   while brake_switch is on
    estimate_bar_per_g: float = 60.0


class VehicleConfig(BaseModel):
    name: str = "Generic"
    transmission: Literal["manual", "auto"] = "manual"
    mass_kg: float = 1400.0  # curb + driver + fuel
    cd: float = 0.32
    frontal_area_m2: float = 2.1
    crr: float = 0.012  # rolling resistance coefficient
    tire_circumference_m: float = 1.95  # rolling circumference of the driven tyres
    gear_ratios: list[float] = Field(default_factory=lambda: [3.6, 2.1, 1.4, 1.0, 0.8, 0.65])
    final_drive: float = 3.9
    drivetrain_efficiency: float = 0.85  # engine -> wheels, used for crank-power estimate
    rotational_inertia_factor: float = 1.08  # effective-mass multiplier for driveline inertia
    idle_rpm: float = 800.0
    redline_rpm: float = 6500.0
    shift_light_rpm: float = 6000.0
    wheelbase_m: float = 2.6
    steering_ratio: float = 15.0  # steering wheel deg per road wheel deg
    brake: BrakeConfig = Field(default_factory=BrakeConfig)
    pids: list[PidRequest] = Field(default_factory=lambda: list(DEFAULT_PIDS))
    signals: list[SignalConfig] = Field(default_factory=list)
    notes: str = ""

    # -- derived helpers ----------------------------------------------------------------
    def rpm_per_kph(self, gear: int) -> float | None:
        """Engine rpm per km/h of road speed in ``gear`` (1-based)."""
        if gear < 1 or gear > len(self.gear_ratios):
            return None
        ratio = self.gear_ratios[gear - 1] * self.final_drive
        # kph -> m/min = kph*1000/60 ; wheel rpm = m/min / circumference
        return (1000.0 / 60.0) / self.tire_circumference_m * ratio

    def gear_count(self) -> int:
        return len(self.gear_ratios)


# --------------------------------------------------------------------------------------
# App
# --------------------------------------------------------------------------------------
class ImuConfig(BaseModel):
    enabled: bool = False
    driver: Literal["mpu6050", "none"] = "mpu6050"
    i2c_bus: int = 1
    address: int = 0x68
    sample_hz: float = 100.0
    # Which IMU axis is the car's forward (lon), left (lat), up (vert).  Format "+x", "-y"...
    axis_lon: str = "+x"
    axis_lat: str = "+y"
    axis_vert: str = "+z"
    lowpass_hz: float = 5.0
    auto_level_s: float = 3.0  # average this long at startup (standing still) to zero offsets

    @field_validator("address", mode="before")
    @classmethod
    def _hex_ok(cls, v):
        return int(v, 0) if isinstance(v, str) else v


class ServerConfig(BaseModel):
    host: str = "0.0.0.0"
    port: int = 8080
    ws_rate_hz: float = 30.0
    open_browser: bool = False


class RevMatchConfig(BaseModel):
    # scoring: full marks at |error| <= perfect_pct, zero at |error| >= zero_pct
    perfect_pct: float = 2.0
    zero_pct: float = 15.0
    under_rev_penalty: float = 1.5  # under-revving (engine too slow) is worse than over
    fast_shift_s: float = 0.5  # shifts quicker than this get the timing bonus
    slow_shift_s: float = 1.5  # shifts slower than this get the timing penalty
    blip_min_throttle_pct: float = 25.0
    min_speed_kph: float = 15.0  # ignore shifts below this speed
    history: int = 20  # scores kept in the session history


class DynoConfig(BaseModel):
    min_throttle_pct: float = 85.0  # WOT threshold for a valid pull
    min_rpm_span: float = 1500.0  # pull must cover at least this many rpm
    min_duration_s: float = 1.5
    rpm_bin: float = 100.0
    smoothing_bins: int = 3  # moving-average half-width in bins
    speed_smoothing_s: float = 0.25  # low-pass on speed before differentiating
    preferred_gears: list[int] = Field(default_factory=lambda: [3, 4])
    air_density_kgm3: float = 1.225
    correction: Literal["none", "sae"] = "sae"  # SAE J1349-style using baro + intake temp
    max_runs: int = 8


class BrakeMetricsConfig(BaseModel):
    trail_lat_g: float = 0.25  # lateral G above which braking counts as trail braking
    event_min_s: float = 0.3
    trace_s: float = 8.0
    max_events: int = 20


class HandlingConfig(BaseModel):
    corner_lat_g: float = 0.3  # a corner starts above this
    g_trail_s: float = 3.0
    peak_g_floor: float = 0.8  # friction-circle envelope never shrinks below this


class MetricsConfig(BaseModel):
    tick_hz: float = 50.0
    stale_s: float = 2.0  # raw signal older than this is treated as missing
    revmatch: RevMatchConfig = Field(default_factory=RevMatchConfig)
    dyno: DynoConfig = Field(default_factory=DynoConfig)
    brake: BrakeMetricsConfig = Field(default_factory=BrakeMetricsConfig)
    handling: HandlingConfig = Field(default_factory=HandlingConfig)
    coolant_alert_c: float = 110.0
    oil_alert_c: float = 130.0
    batt_low_v: float = 12.0


class RecordingConfig(BaseModel):
    enabled: bool = False
    dir: str = "recordings"


class AppConfig(BaseModel):
    bus: BusConfig = Field(default_factory=BusConfig)
    imu: ImuConfig = Field(default_factory=ImuConfig)
    server: ServerConfig = Field(default_factory=ServerConfig)
    metrics: MetricsConfig = Field(default_factory=MetricsConfig)
    recording: RecordingConfig = Field(default_factory=RecordingConfig)
    vehicle_file: str | None = None  # path to a vehicle yaml, relative to the app config
    vehicle: VehicleConfig = Field(default_factory=VehicleConfig)


# --------------------------------------------------------------------------------------
# Loaders
# --------------------------------------------------------------------------------------
PACKAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_DIR.parent.parent
CONFIG_DIR = REPO_ROOT / "config"


def load_yaml(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_vehicle(path: str | Path) -> VehicleConfig:
    return VehicleConfig.model_validate(load_yaml(path))


def load_config(
    path: str | Path | None = None, vehicle_path: str | Path | None = None
) -> AppConfig:
    """Load the app config; ``vehicle_path`` (or the ``vehicle_file`` key) overrides the
    inline ``vehicle`` section."""
    if path is None:
        path = CONFIG_DIR / "default.yaml"
    path = Path(path)
    data = load_yaml(path) if path.exists() else {}
    cfg = AppConfig.model_validate(data)
    vp = vehicle_path or cfg.vehicle_file
    if vp:
        vp = Path(vp)
        if not vp.is_absolute():
            candidate = path.parent / vp
            vp = candidate if candidate.exists() else (CONFIG_DIR / vp)
        cfg.vehicle = load_vehicle(vp)
        cfg.vehicle_file = str(vp)
    return cfg
