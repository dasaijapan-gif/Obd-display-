"""Tests for obddash.metrics.gears (GearEstimator) and the shared filters."""

from __future__ import annotations

import math

import pytest

from obddash.config import CONFIG_DIR, load_vehicle
from obddash.metrics.filters import (
    Differentiator,
    LowPass,
    RunningStats,
    Sampler,
    TimedRing,
    clamp,
    downsample,
    lerp,
    rms,
)
from obddash.metrics.gears import GearEstimator

DT = 0.02


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


class Feed:
    """Drives a GearEstimator at 50 Hz."""

    def __init__(self, vehicle):
        self.v = vehicle
        self.g = GearEstimator(vehicle)
        self.t = 0.0
        self.est = None

    def tick(self, rpm, kph, clutch=None, gear=None, n=1, fresh=True):
        for _ in range(n):
            self.t += DT
            self.est = self.g.update(self.t, rpm, kph, clutch, gear, fresh=fresh)
        return self.est

    def in_gear(self, ratio_gear, kph, n=1, err_pct=0.0, **kw):
        """Ticks with the rpm of ``ratio_gear`` at ``kph`` (``gear=`` is the CAN signal)."""
        rpm = kph * self.v.rpm_per_kph(ratio_gear) * (1 + err_pct / 100.0)
        return self.tick(rpm, kph, n=n, **kw)


# --------------------------------------------------------------------------------------
# ratio method
# --------------------------------------------------------------------------------------
@pytest.mark.parametrize("gear", [1, 2, 3, 4, 5, 6])
def test_ratio_lock_in_each_gear(vehicle, gear):
    f = Feed(vehicle)
    kph = 3500.0 / vehicle.rpm_per_kph(gear)
    est = f.in_gear(gear, kph, n=3)
    assert est.locked
    assert est.gear == gear
    assert est.source == "ratio"
    assert est.conf == pytest.approx(1.0)
    assert est.ratio_err_pct == pytest.approx(0.0, abs=1e-9)
    assert f.g.last_engaged_gear == gear


def test_lock_needs_three_agreeing_ticks(vehicle):
    f = Feed(vehicle)
    assert not f.in_gear(3, 80).locked
    assert not f.in_gear(3, 80).locked
    est = f.in_gear(3, 80)
    assert est.locked and est.gear == 3
    assert est.match_t == pytest.approx(DT)  # dated from the first agreeing tick


def test_confidence_falls_with_ratio_error(vehicle):
    f = Feed(vehicle)
    est = f.in_gear(4, 100, n=5, err_pct=3.0)
    assert est.locked and est.gear == 4
    assert est.ratio_err_pct == pytest.approx(3.0, abs=0.01)
    assert est.conf == pytest.approx(0.5, abs=0.01)  # 1 - 3/6


def test_nothing_within_tolerance_is_unlocked(vehicle):
    f = Feed(vehicle)
    # halfway (geometrically) between 3rd and 4th: > 6 % from both
    kph = 100.0
    rpm = kph * math.sqrt(vehicle.rpm_per_kph(3) * vehicle.rpm_per_kph(4))
    est = f.tick(rpm, kph, n=10)
    assert not est.locked
    assert est.gear == 0
    assert est.conf == 0.0


def test_hysteresis_one_bad_tick_does_not_switch(vehicle):
    f = Feed(vehicle)
    f.in_gear(3, 80, n=10)
    bad = f.in_gear(4, 80)  # one tick that looks like 4th
    assert bad.gear == 3  # held for display, not switched
    assert not bad.locked
    back = f.in_gear(3, 80)
    assert back.locked and back.gear == 3  # immediate re-lock into the same gear
    # two bad ticks in a row are still not enough to switch
    f.in_gear(4, 80, n=2)
    assert f.in_gear(3, 80).gear == 3


def test_switch_after_three_ticks(vehicle):
    f = Feed(vehicle)
    f.in_gear(3, 80, n=10)
    f.in_gear(4, 80, n=2)
    est = f.in_gear(4, 80)
    assert est.locked and est.gear == 4


def test_clutch_pressed_unlocks_and_reports_neutral(vehicle):
    f = Feed(vehicle)
    f.in_gear(4, 100, n=5)
    est = f.in_gear(4, 100, clutch=True)
    assert not est.locked
    assert est.gear == 0
    assert f.g.last_engaged_gear == 4
    est = f.in_gear(4, 100, clutch=False)
    assert est.locked and est.gear == 4


def test_low_speed_and_low_rpm_unlock(vehicle):
    f = Feed(vehicle)
    f.in_gear(1, 20, n=5)
    assert f.in_gear(1, 4.0).locked is False  # < 5 km/h
    f2 = Feed(vehicle)
    f2.in_gear(6, 40, n=5)  # 6th at 40 km/h = ~590 rpm < idle - 100
    assert not f2.est.locked


def test_missing_signals_mean_no_source(vehicle):
    f = Feed(vehicle)
    est = f.tick(None, None)
    assert est.source == "none" and not est.locked and est.gear == 0
    est = f.tick(3000, None)
    assert est.source == "none"


def test_free_revving_sweep_through_a_window_is_not_a_lock(vehicle):
    """A blip sweeping the rpm through 3rd's +-6 % window at 3500 rpm/s must not lock."""
    f = Feed(vehicle)
    f.in_gear(4, 100, n=10)
    target = 100 * vehicle.rpm_per_kph(3)
    rpm = target * 0.85
    locked_in_3 = False
    while rpm < target * 1.15:
        est = f.tick(rpm, 100)
        locked_in_3 |= est.locked and est.gear == 3
        rpm += 3500 * DT
    assert not locked_in_3


def test_stale_repeats_do_not_confirm_a_new_gear(vehicle):
    f = Feed(vehicle)
    f.in_gear(4, 100, n=10)
    f.in_gear(4, 100, clutch=True, n=5)
    # the new gear's ratio, but the same (stale) sample repeated by a 50 Hz loop
    est = f.in_gear(3, 100, n=1)
    est = f.in_gear(3, 100, n=5, fresh=False)
    assert not est.locked
    est = f.in_gear(3, 100, n=2, fresh=True)
    assert est.locked and est.gear == 3


# --------------------------------------------------------------------------------------
# CAN gear signal
# --------------------------------------------------------------------------------------
def test_can_gear_signal_wins(vehicle):
    f = Feed(vehicle)
    # the lever says 2nd although the ratio says 4th: the lever is reported...
    est = f.in_gear(4, 100, gear=2, n=5)
    assert est.source == "can"
    assert est.gear == 2
    assert est.conf == 1.0
    # ...but the driveline is not coupled in 2nd
    assert not est.locked
    est = f.in_gear(2, 60, gear=2, n=3)
    assert est.locked and est.gear == 2


def test_can_gear_with_clutch_pressed_is_not_locked(vehicle):
    f = Feed(vehicle)
    f.in_gear(3, 80, gear=3, n=5)
    est = f.in_gear(3, 80, gear=3, clutch=True)
    assert est.gear == 3 and est.source == "can"
    assert not est.locked


def test_can_reverse_and_neutral(vehicle):
    f = Feed(vehicle)
    est = f.tick(1200, 6, gear=-1, n=3)
    assert est.gear == -1 and est.source == "can"
    est = f.tick(900, 0, gear=0, n=3)
    assert est.gear == 0 and not est.locked


def test_out_of_range_can_gear_code_is_not_a_gear(vehicle):
    """Many cars broadcast an "invalid" code (7, 15) while the lever is between gates.
    It must not be reported - let alone locked - as a gear (that produced a phantom
    "15 -> 3" downshift); the ratio is used instead, and the real gear wins again."""
    f = Feed(vehicle)
    f.in_gear(4, 100, gear=4, n=5)
    est = f.tick(3300, 100, gear=15, n=10)  # free-revving between 4th and 3rd
    assert est.gear != 15 and est.source == "ratio" and not est.locked
    est = f.in_gear(4, 100, gear=7, n=5)  # still coupled in 4th: the ratio says so
    assert est.source == "ratio" and est.locked and est.gear == 4
    est = f.tick(900, 50, gear=-3, n=3)
    assert est.gear != -3
    est = f.in_gear(3, 100, gear=3, n=3)
    assert est.source == "can" and est.locked and est.gear == 3


# --------------------------------------------------------------------------------------
# filters
# --------------------------------------------------------------------------------------
def test_clamp_and_lerp():
    assert clamp(5, 0, 1) == 1 and clamp(-1, 0, 1) == 0 and clamp(0.5, 0, 1) == 0.5
    assert lerp(5, -10, 0.5) == pytest.approx(-2.5)


def test_lowpass_is_time_aware():
    a, b = LowPass(0.5), LowPass(0.5)
    a.update(0.0, 0.0)
    b.update(0.0, 0.0)
    for i in range(1, 101):  # 1 s at 100 Hz
        a.update(1.0, i * 0.01)
    for i in range(1, 11):  # 1 s at 10 Hz
        b.update(1.0, i * 0.1)
    expected = 1 - math.exp(-1.0 / 0.5)
    assert a.value == pytest.approx(expected, rel=1e-6)
    assert b.value == pytest.approx(expected, rel=1e-6)
    assert a.update(None, 2.0) == a.value  # None holds


def test_differentiator_slope_and_center_with_irregular_dt():
    d = Differentiator(window_s=0.3)
    t = 0.0
    steps = [0.013, 0.021, 0.017, 0.031, 0.009] * 10
    for dt in steps:
        t += dt
        d.update(2.0 + 3.0 * t, t)
    assert d.slope == pytest.approx(3.0, rel=1e-6)
    assert d.center == pytest.approx(2.0 + 3.0 * d.center_t, rel=1e-6)


def test_differentiator_needs_span_and_ignores_repeats():
    d = Differentiator(window_s=0.4)
    assert d.update(1.0, 0.0) is None
    assert d.update(1.0, 0.0) is None  # same timestamp
    assert d.update(1.1, 0.05) is None  # span too short
    d.reset()
    assert d.slope is None and d.filtered is None


def test_timed_ring_bounds_and_slices():
    r = TimedRing(1.0, max_n=30)
    for i in range(200):
        r.append(i * 0.01, i)
    assert len(r) <= 30
    assert r.last() == (1.99, 199)
    r2 = TimedRing(0.5, max_n=1000)
    for i in range(200):
        r2.append(i * 0.01, i)
    assert r2.items()[0][0] >= 1.99 - 0.5 - 1e-9
    assert [v for _, v in r2.between(1.9, 1.95)] == [190, 191, 192, 193, 194, 195]
    assert r2.before(1.9)[1] == 189


def test_downsample_keeps_ends():
    pts = list(range(1000))
    ds = downsample(pts, 120)
    assert len(ds) == 120 and ds[0] == 0 and ds[-1] == 999
    assert downsample(pts[:10], 120) == pts[:10]


def test_running_stats_and_rms():
    s = RunningStats()
    for x in [1, 2, 3, 4]:
        s.add(x)
    assert s.mean == pytest.approx(2.5)
    assert s.var == pytest.approx(5 / 3)
    assert rms([3, -4]) == pytest.approx(math.sqrt(12.5))
    assert rms([]) == 0.0


def test_sampler_keeps_the_average_rate():
    s = Sampler(0.05)  # 20 Hz from a 50 Hz tick
    hits = [i for i in range(500) if s.due(i * 0.02)]
    assert len(hits) == 200  # exactly 20 Hz over 10 s
    gaps = {b - a for a, b in zip(hits, hits[1:], strict=False)}
    assert gaps == {2, 3}
    # after a long gap it restarts instead of bursting
    assert s.due(100.0)
    assert not s.due(100.02)
    s.reset()
    assert s.due(100.02)
