"""MPU-6050 accelerometer driver: G-forces for cars whose CAN bus does not broadcast them.

The chip is read over I2C (``smbus2``) on its own thread at ``imu.sample_hz`` and the
three accelerations are written into :class:`~obddash.model.RawState` as
``acc_lon_g``, ``acc_lat_g`` and ``acc_vert_g``.  The metrics engine prefers these over
values derived from speed and steering.

Axes.  The car uses SAE axes: x forward, y left, z up.  An accelerometer measures
specific force, so at rest the axis pointing up reads +1 g, accelerating forward reads
positive on the forward axis and a left turn reads positive on the left axis, which
is exactly the sign convention in ``model.py``.  ``imu.axis_lon/lat/vert`` say which
chip axis points which way, e.g. ``axis_lon: "-y"`` when the chip's y arrow points
backwards.

Auto-level.  A display is rarely mounted perfectly level, and a few degrees of tilt
leaks gravity into lon/lat (sin 3 deg = 0.05 g).  For ``imu.auto_level_s`` after start
the driver averages the readings; if the car was still (every axis' standard
deviation under ``LEVEL_STILL_G``) those means become offsets: lon and lat are zeroed
and vert is pulled to 1 g.  If the car was moving the window is discarded and
retried, up to ``LEVEL_MAX_TRIES`` times; until then raw values are published.

Filtering.  The chip's own low-pass (DLPF ~44 Hz) runs first, then a first-order
low-pass at ``imu.lowpass_hz`` (5 Hz default: enough for braking and cornering,
removes engine and road vibration).

Register map (MPU-6000/6050 register map rev 4.2): PWR_MGMT_1 0x6B, SMPLRT_DIV 0x19,
CONFIG 0x1A (DLPF), ACCEL_CONFIG 0x1C (full scale), ACCEL_XOUT_H 0x3B (six bytes,
big-endian signed X, Y, Z), WHO_AM_I 0x75.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from typing import Any

from obddash.config import ImuConfig
from obddash.model import RawState, Sig

log = logging.getLogger(__name__)

# registers
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

# WHO_AM_I values of register-compatible parts: MPU-6050, MPU-6500, MPU-9250, MPU-9255
KNOWN_WHO_AM_I = {0x68: "MPU-6050", 0x70: "MPU-6500", 0x71: "MPU-9250", 0x73: "MPU-9255"}

ACCEL_FS_4G = 0x08  # ACCEL_CONFIG AFS_SEL=1: +-4 g (potholes exceed 2 g vertically)
LSB_PER_G = 8192.0  # at +-4 g
DLPF_44HZ = 0x03

LEVEL_STILL_G = 0.03  # max per-axis std dev for the car to count as stationary
LEVEL_MAX_TRIES = 20
MAX_CONSECUTIVE_ERRORS = 10  # then mark unavailable and re-initialise
REINIT_S = 2.0
STALE_S = 0.5  # `available` turns False when no sample for this long


def parse_axis(spec: str) -> tuple[int, float]:
    """``"+x"`` / ``"-z"`` -> (chip axis index 0..2, sign)."""
    s = spec.strip().lower()
    if len(s) == 1:
        s = "+" + s
    if len(s) != 2 or s[0] not in "+-" or s[1] not in "xyz":
        raise ValueError(f"IMU axis must look like '+x' or '-y', got {spec!r}")
    return "xyz".index(s[1]), (1.0 if s[0] == "+" else -1.0)


def axis_map(cfg: ImuConfig) -> tuple[tuple[int, float], tuple[int, float], tuple[int, float]]:
    """(lon, lat, vert) mappings; raises if two car axes use the same chip axis."""
    lon, lat, vert = parse_axis(cfg.axis_lon), parse_axis(cfg.axis_lat), parse_axis(cfg.axis_vert)
    if len({lon[0], lat[0], vert[0]}) != 3:
        raise ValueError(
            f"IMU axes must be three different chip axes: lon={cfg.axis_lon} "
            f"lat={cfg.axis_lat} vert={cfg.axis_vert}"
        )
    return lon, lat, vert


def to_signed16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


class _LowPass:
    """First-order low-pass with cutoff ``fc`` Hz, time-aware."""

    def __init__(self, fc: float):
        self.rc = 1.0 / (2.0 * math.pi * fc) if fc > 0 else 0.0
        self.y: float | None = None

    def update(self, x: float, dt: float) -> float:
        if self.y is None or self.rc <= 0 or dt <= 0:
            self.y = x
        else:
            a = dt / (self.rc + dt)
            self.y += a * (x - self.y)
        return self.y


def _default_bus_factory(bus_number: int) -> Any:
    from smbus2 import SMBus  # optional dependency: pip install -e ".[imu]"

    return SMBus(bus_number)


class Imu:
    """MPU-6050 reader.  ``start()`` returns False (and never raises) when the driver,
    the I2C bus or the chip is missing; ``available`` says whether fresh samples flow."""

    def __init__(
        self,
        cfg: ImuConfig,
        raw: RawState,
        *,
        bus_factory: Callable[[int], Any] | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ):
        self.cfg = cfg
        self.raw = raw
        self._bus_factory = bus_factory or _default_bus_factory
        self._clock = clock
        self._sleep = sleep
        self._axes = axis_map(cfg)
        self._bus: Any = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lp = [_LowPass(cfg.lowpass_hz) for _ in range(3)]
        self._last_t: float | None = None
        self.chip: str | None = None
        self.samples = 0
        self.errors = 0
        self.last_error: str | None = None
        self.last_sample_t: float | None = None
        self.offsets = (0.0, 0.0, 0.0)  # subtracted from (lon, lat, vert)
        self.leveled = False
        self._level_buf: list[tuple[float, float, float]] = []
        self._level_t0: float | None = None
        self._level_tries = 0

    # -- lifecycle -----------------------------------------------------------------
    def start(self) -> bool:
        if not self._open():
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="obddash-imu", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._close()
        self.last_sample_t = None  # `available` is False once stopped

    @property
    def available(self) -> bool:
        t = self.last_sample_t
        return t is not None and (self._clock() - t) < STALE_S

    # -- hardware ------------------------------------------------------------------
    def _open(self) -> bool:
        try:
            bus = self._bus_factory(self.cfg.i2c_bus)
        except ImportError as exc:
            self.last_error = f"smbus2 not installed ({exc})"
            log.warning("IMU: %s; install with pip install -e '.[imu]'", self.last_error)
            return False
        except Exception as exc:  # no /dev/i2c-N, permissions
            self.last_error = f"cannot open I2C bus {self.cfg.i2c_bus}: {exc}"
            log.warning("IMU: %s", self.last_error)
            return False
        addr = self.cfg.address
        try:
            who = bus.read_byte_data(addr, REG_WHO_AM_I)
            if who not in KNOWN_WHO_AM_I:
                raise OSError(f"unexpected WHO_AM_I 0x{who:02X} at address 0x{addr:02X}")
            bus.write_byte_data(addr, REG_PWR_MGMT_1, 0x01)  # wake, PLL with x-gyro clock
            self._sleep(0.05)
            bus.write_byte_data(addr, REG_CONFIG, DLPF_44HZ)
            div = max(0, min(255, round(1000.0 / max(1.0, self.cfg.sample_hz)) - 1))
            bus.write_byte_data(addr, REG_SMPLRT_DIV, div)  # 1 kHz / (1 + div) with DLPF
            bus.write_byte_data(addr, REG_ACCEL_CONFIG, ACCEL_FS_4G)
        except Exception as exc:
            self.last_error = f"no MPU-6050 at 0x{addr:02X} on bus {self.cfg.i2c_bus}: {exc}"
            log.warning("IMU: %s", self.last_error)
            try:
                bus.close()
            except Exception:
                pass
            return False
        self._bus = bus
        self.chip = KNOWN_WHO_AM_I[who]
        log.info("IMU: %s at 0x%02X on i2c-%d", self.chip, addr, self.cfg.i2c_bus)
        return True

    def _close(self) -> None:
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass

    def read_g(self) -> tuple[float, float, float]:
        """One reading in chip axes (x, y, z), in g."""
        d = self._bus.read_i2c_block_data(self.cfg.address, REG_ACCEL_XOUT_H, 6)
        return (
            to_signed16(d[0], d[1]) / LSB_PER_G,
            to_signed16(d[2], d[3]) / LSB_PER_G,
            to_signed16(d[4], d[5]) / LSB_PER_G,
        )

    # -- processing (pure; unit-tested directly) -------------------------------------
    def to_car(self, chip: tuple[float, float, float]) -> tuple[float, float, float]:
        (i0, s0), (i1, s1), (i2, s2) = self._axes
        return s0 * chip[i0], s1 * chip[i1], s2 * chip[i2]

    def process(self, chip: tuple[float, float, float], t: float) -> tuple[float, float, float]:
        """Map axes, auto-level, low-pass and publish one sample.  Returns (lon, lat, vert)."""
        lon, lat, vert = self.to_car(chip)
        if not self.leveled:
            self._level(lon, lat, vert, t)
        o = self.offsets
        lon, lat, vert = lon - o[0], lat - o[1], vert - o[2]
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        lon = self._lp[0].update(lon, dt)
        lat = self._lp[1].update(lat, dt)
        vert = self._lp[2].update(vert, dt)
        self.raw.update({Sig.ACC_LON_G: lon, Sig.ACC_LAT_G: lat, Sig.ACC_VERT_G: vert}, t)
        self.samples += 1
        self.last_sample_t = t
        return lon, lat, vert

    def _level(self, lon: float, lat: float, vert: float, t: float) -> None:
        if self.cfg.auto_level_s <= 0 or self._level_tries >= LEVEL_MAX_TRIES:
            return
        if self._level_t0 is None:
            self._level_t0 = t
        self._level_buf.append((lon, lat, vert))
        if t - self._level_t0 < self.cfg.auto_level_s:
            return
        n = len(self._level_buf)
        means = [sum(s[k] for s in self._level_buf) / n for k in range(3)]
        stds = [
            math.sqrt(sum((s[k] - means[k]) ** 2 for s in self._level_buf) / n) for k in range(3)
        ]
        self._level_buf = []
        self._level_t0 = None
        self._level_tries += 1
        if max(stds) < LEVEL_STILL_G and n >= 5:
            self.offsets = (means[0], means[1], means[2] - 1.0)
            self.leveled = True
            # restart the filters so the step from the offset change does not ring
            for lp in self._lp:
                lp.y = None
            log.info(
                "IMU: levelled, offsets lon %+.3f g lat %+.3f g vert %+.3f g",
                *self.offsets,
            )
        elif self._level_tries >= LEVEL_MAX_TRIES:
            log.warning("IMU: car never still long enough to auto-level; using raw values")

    # -- thread ----------------------------------------------------------------------
    def _run(self) -> None:
        period = 1.0 / max(1.0, self.cfg.sample_hz)
        next_t = self._clock()
        consecutive = 0
        while not self._stop.is_set():
            if self._bus is None:
                if self._stop.wait(REINIT_S):
                    break
                self._open()
                continue
            try:
                chip = self.read_g()
            except Exception as exc:
                self.errors += 1
                consecutive += 1
                self.last_error = str(exc)
                if consecutive >= MAX_CONSECUTIVE_ERRORS:
                    log.warning(
                        "IMU: %d read errors in a row (%s); re-initialising", consecutive, exc
                    )
                    self._close()
                    consecutive = 0
                self._stop.wait(period)
                continue
            consecutive = 0
            self.process(chip, self._clock())
            next_t += period
            delay = next_t - self._clock()
            if delay < -0.5:  # fell far behind (scheduler hiccup): resync, do not burst
                next_t = self._clock()
            elif delay > 0:
                self._stop.wait(delay)


class FakeImu:
    """Stand-in for development and tests: writes fixed or scripted G-forces."""

    def __init__(self, raw: RawState, lat: float = 0.0, lon: float = 0.0, vert: float = 1.0):
        self.raw = raw
        self.available = False
        self.set(lat=lat, lon=lon, vert=vert)

    def set(self, lat: float, lon: float, vert: float = 1.0) -> None:
        self.raw.update({Sig.ACC_LAT_G: lat, Sig.ACC_LON_G: lon, Sig.ACC_VERT_G: vert})

    def start(self) -> bool:
        self.available = True
        return True

    def stop(self) -> None:
        self.available = False
