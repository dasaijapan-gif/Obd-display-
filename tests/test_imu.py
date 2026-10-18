"""MPU-6050 driver tests against a simulated I2C chip (no hardware, no smbus2 needed)."""

from __future__ import annotations

import math
import time

import pytest

from obddash.config import ImuConfig
from obddash.model import RawState, Sig
from obddash.sensors.imu import (
    ACCEL_FS_4G,
    LSB_PER_G,
    REG_ACCEL_CONFIG,
    REG_PWR_MGMT_1,
    FakeImu,
    Imu,
    axis_map,
    parse_axis,
    to_signed16,
)


class FakeChip:
    """Register-level MPU-6050: WHO_AM_I, config writes and a settable acceleration."""

    def __init__(self, who=0x68, g=(0.0, 0.0, 1.0), fail_reads=0):
        self.who = who
        self.g = list(g)
        self.writes: dict[int, int] = {}
        self.fail_reads = fail_reads
        self.closed = False

    def read_byte_data(self, addr, reg):
        assert addr == 0x68
        if reg == 0x75:
            return self.who
        return self.writes.get(reg, 0)

    def write_byte_data(self, addr, reg, value):
        self.writes[reg] = value

    def read_i2c_block_data(self, addr, reg, n):
        assert (reg, n) == (0x3B, 6)
        if self.fail_reads > 0:
            self.fail_reads -= 1
            raise OSError("Remote I/O error")
        out = []
        for v in self.g:
            raw = max(-32768, min(32767, round(v * LSB_PER_G))) & 0xFFFF
            out += [raw >> 8, raw & 0xFF]
        return out

    def close(self):
        self.closed = True


def make(chip, **cfg):
    raw = RawState()
    imu = Imu(ImuConfig(enabled=True, **cfg), raw, bus_factory=lambda n: chip, sleep=lambda s: None)
    return imu, raw


def test_parse_axis_and_validation():
    assert parse_axis("+x") == (0, 1.0)
    assert parse_axis("-Z") == (2, -1.0)
    assert parse_axis("y") == (1, 1.0)
    with pytest.raises(ValueError):
        parse_axis("+w")
    with pytest.raises(ValueError):
        axis_map(ImuConfig(axis_lon="+x", axis_lat="-x", axis_vert="+z"))


def test_to_signed16():
    assert to_signed16(0x7F, 0xFF) == 32767
    assert to_signed16(0x80, 0x00) == -32768
    assert to_signed16(0xFF, 0xFF) == -1


def test_start_configures_chip_and_reads_g():
    chip = FakeChip(g=(0.25, -0.5, 1.0))
    imu, _ = make(chip)
    assert imu._open() is True
    assert chip.writes[REG_PWR_MGMT_1] == 0x01
    assert chip.writes[REG_ACCEL_CONFIG] == ACCEL_FS_4G
    x, y, z = imu.read_g()
    assert (x, y, z) == pytest.approx((0.25, -0.5, 1.0), abs=1e-3)
    assert imu.chip == "MPU-6050"


def test_start_fails_cleanly_on_wrong_chip_or_missing_bus():
    imu, _ = make(FakeChip(who=0x12))
    assert imu.start() is False and "WHO_AM_I" in imu.last_error

    def no_bus(n):
        raise FileNotFoundError("/dev/i2c-1")

    imu2 = Imu(ImuConfig(enabled=True), RawState(), bus_factory=no_bus)
    assert imu2.start() is False and "I2C bus" in imu2.last_error
    assert imu2.available is False


def test_axis_mapping_follows_sae_signs():
    # chip mounted rotated: its -y points forward, +x points left, +z up
    imu, raw = make(FakeChip(), axis_lon="-y", axis_lat="+x", axis_vert="+z", auto_level_s=0)
    lon, lat, vert = imu.to_car((0.3, -0.6, 1.0))
    assert (lon, lat, vert) == pytest.approx((0.6, 0.3, 1.0))
    imu.process((0.3, -0.6, 1.0), 10.0)
    assert raw.get(Sig.ACC_LON_G) == pytest.approx(0.6)
    assert raw.get(Sig.ACC_LAT_G) == pytest.approx(0.3)
    assert raw.get(Sig.ACC_VERT_G) == pytest.approx(1.0)


def test_auto_level_removes_mounting_tilt_when_still():
    tilt = math.radians(4)  # display tipped 4 deg nose-up: gravity leaks into lon
    still = (-math.sin(tilt), 0.02, math.cos(tilt))
    imu, raw = make(FakeChip(), auto_level_s=1.0, lowpass_hz=0)
    for k in range(120):  # 1.2 s at 100 Hz
        imu.process(still, k * 0.01)
    assert imu.leveled
    imu.process(still, 1.21)
    assert raw.get(Sig.ACC_LON_G) == pytest.approx(0.0, abs=1e-6)
    assert raw.get(Sig.ACC_LAT_G) == pytest.approx(0.0, abs=1e-6)
    assert raw.get(Sig.ACC_VERT_G) == pytest.approx(1.0, abs=1e-6)
    # a real 0.5 g brake now reads as -0.5 g, not -0.5 - sin(4 deg)
    imu.process((-math.sin(tilt) - 0.5, 0.02, math.cos(tilt)), 1.22)
    assert raw.get(Sig.ACC_LON_G) == pytest.approx(-0.5, abs=1e-6)


def test_auto_level_waits_while_moving():
    imu, _ = make(FakeChip(), auto_level_s=0.5, lowpass_hz=0)
    for k in range(100):  # driving: lon swings +-0.3 g
        imu.process((0.3 * math.sin(k * 0.4), 0.0, 1.0), k * 0.01)
    assert not imu.leveled and imu.offsets == (0.0, 0.0, 0.0)
    for k in range(100, 200):  # then parked
        imu.process((0.05, 0.0, 1.0), k * 0.01)
    assert imu.leveled and imu.offsets[0] == pytest.approx(0.05)


def test_lowpass_smooths_vibration():
    imu, raw = make(FakeChip(), auto_level_s=0, lowpass_hz=5.0)
    peak = 0.0
    for k in range(400):  # 4 s of 40 Hz +-0.4 g engine vibration on a steady 0.2 g
        lon = imu.process(
            (0.2 + 0.4 * math.sin(2 * math.pi * 40 * k * 0.01 + 0.3), 0.0, 1.0), k * 0.01
        )[0]
        if k > 200:
            peak = max(peak, abs(lon - 0.2))
    assert peak < 0.12  # 40 Hz through a 5 Hz first-order filter: about 8x smaller


def test_thread_publishes_and_survives_read_errors():
    chip = FakeChip(g=(0.0, 0.0, 1.0), fail_reads=3)
    raw = RawState()
    imu = Imu(
        ImuConfig(enabled=True, sample_hz=200, auto_level_s=0), raw, bus_factory=lambda n: chip
    )
    assert imu.start() is True
    try:
        deadline = time.monotonic() + 2.0
        while imu.samples < 20 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert imu.samples >= 20 and imu.errors == 3
        assert imu.available
        assert raw.get(Sig.ACC_VERT_G) == pytest.approx(1.0, abs=1e-3)
    finally:
        imu.stop()
    assert chip.closed
    assert not imu.available


def test_fake_imu():
    raw = RawState()
    f = FakeImu(raw, lat=0.4, lon=-0.2)
    assert f.start() and f.available
    assert raw.get(Sig.ACC_LAT_G) == 0.4 and raw.get(Sig.ACC_LON_G) == -0.2
    f.stop()
    assert not f.available
