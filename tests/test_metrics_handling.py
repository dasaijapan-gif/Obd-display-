"""Tests for obddash.metrics.handling (HandlingAnalyzer)."""

from __future__ import annotations

import math

import pytest

from obddash.config import CONFIG_DIR, HandlingConfig, load_vehicle
from obddash.metrics.handling import HandlingAnalyzer

DT = 0.02
G = 9.81


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


class Rig:
    def __init__(self, vehicle, cfg: HandlingConfig | None = None):
        self.v = vehicle
        self.h = HandlingAnalyzer(cfg or HandlingConfig(), vehicle)
        self.t = 0.0

    def tick(self, kph=80.0, lat=None, lon=None, vert=None, steer=None, yaw=None, n=1):
        for _ in range(n):
            self.t += DT
            self.h.update(self.t, kph, lat, lon, vert, steer, yaw)

    def ack_yaw_dps(self, kph, steer_deg):
        v = kph / 3.6
        delta = math.radians(steer_deg / self.v.steering_ratio)
        return math.degrees(v * delta / self.v.wheelbase_m)


def test_left_turn_is_positive_lat_from_imu(vehicle):
    rig = Rig(vehicle)
    rig.tick(lat=0.6, lon=-0.1, vert=1.0, n=50)
    g = rig.h.g_snapshot()
    assert g.source == "imu"
    assert g.lat == pytest.approx(0.6, abs=1e-3) and g.lon == pytest.approx(-0.1, abs=1e-3)
    assert g.total == pytest.approx(math.hypot(0.6, 0.1), abs=1e-3)
    assert g.vert == pytest.approx(1.0)
    assert g.peak_lat == pytest.approx(0.6, abs=1e-3)
    assert g.peak_brake == pytest.approx(0.1, abs=1e-3)


def test_left_turn_is_positive_lat_from_steering_bicycle_model(vehicle):
    rig = Rig(vehicle)
    kph, steer = 60.0, 45.0  # wheel turned left
    rig.tick(kph=kph, steer=steer, n=50)
    g = rig.h.g_snapshot()
    assert g.source == "derived"
    v = kph / 3.6
    delta = math.radians(steer / vehicle.steering_ratio)
    expected = v * v * delta / (vehicle.wheelbase_m * G)
    assert g.lat == pytest.approx(expected, rel=1e-3)
    assert g.lat > 0
    rig.tick(kph=kph, steer=-steer, n=50)
    assert rig.h.g_snapshot().lat < 0


def test_derived_lat_prefers_yaw_rate(vehicle):
    rig = Rig(vehicle)
    rig.tick(kph=72.0, steer=30.0, yaw=10.0, n=50)
    expected = 20.0 * math.radians(10.0) / G  # v * r / g
    assert rig.h.g_snapshot().lat == pytest.approx(expected, rel=1e-3)


def test_derived_lon_from_speed(vehicle):
    rig = Rig(vehicle)
    kph = 120.0
    for _ in range(100):  # braking at exactly 1 g
        rig.tick(kph=kph)
        kph -= G * DT * 3.6
    g = rig.h.g_snapshot()
    assert g.source == "derived"
    assert g.lon == pytest.approx(-1.0, abs=0.01)
    assert g.peak_brake == pytest.approx(1.0, abs=0.01)


def test_understeer_is_negative_balance_in_both_directions(vehicle):
    for sign in (1, -1):
        rig = Rig(vehicle)
        kph, steer = 80.0, 40.0 * sign
        ack = rig.ack_yaw_dps(kph, steer)
        rig.tick(kph=kph, lat=0.5 * sign, steer=steer, yaw=0.7 * ack, n=150)
        bal = rig.h.snapshot().balance
        assert bal == pytest.approx(-0.3, abs=0.01)
        rig2 = Rig(vehicle)
        rig2.tick(kph=kph, lat=0.5 * sign, steer=steer, yaw=1.2 * ack, n=150)
        assert rig2.h.snapshot().balance == pytest.approx(0.2, abs=0.01)


def test_counter_steer_slide_reads_oversteer(vehicle):
    rig = Rig(vehicle)
    # rotating left hard with the wheel turned right (catching a slide)
    rig.tick(kph=80.0, lat=0.6, steer=-30.0, yaw=25.0, n=150)
    assert rig.h.snapshot().balance == pytest.approx(1.0, abs=0.01)


def test_balance_needs_speed_and_signals(vehicle):
    rig = Rig(vehicle)
    rig.tick(kph=15.0, lat=0.3, steer=90.0, yaw=20.0, n=20)
    assert rig.h.snapshot().balance is None
    rig.tick(kph=80.0, lat=0.3, steer=20.0, yaw=None, n=20)
    assert rig.h.snapshot().balance is None
    rig.tick(kph=80.0, lat=0.0, steer=0.0, yaw=0.0, n=100)
    assert rig.h.snapshot().balance == pytest.approx(0.0, abs=1e-6)  # straight: neutral


def test_corner_counting_with_hysteresis(vehicle):
    rig = Rig(vehicle)
    rig.tick(lat=0.0, lon=0.0, n=20)
    assert rig.h.snapshot().corner_count == 0
    rig.tick(lat=0.6, n=50)
    assert rig.h.snapshot().corner_count == 1
    rig.tick(lat=0.2, n=20)  # still above the 50 % exit threshold (0.15)
    rig.tick(lat=0.6, n=20)
    assert rig.h.snapshot().corner_count == 1
    rig.tick(lat=0.0, n=50)
    rig.tick(lat=-0.5, n=50)  # a right-hander
    rig.tick(lat=0.0, n=50)
    assert rig.h.snapshot().corner_count == 2


def test_score_zero_before_first_corner_then_positive(vehicle):
    rig = Rig(vehicle)
    rig.tick(lat=0.1, lon=0.0, n=100)
    snap = rig.h.snapshot()
    assert snap.score == 0.0 and snap.smoothness == pytest.approx(100.0, abs=0.5)
    rig.tick(lat=0.8, n=100)
    assert rig.h.snapshot().score == 0.0  # corner not finished yet
    rig.tick(lat=0.0, n=100)
    snap = rig.h.snapshot()
    # envelope = max(0.8, 0.98 * 0.8) = 0.8 -> corner scores 100 %
    assert snap.score == pytest.approx(0.6 * snap.smoothness + 0.4 * 100.0, abs=0.5)
    assert snap.max_lat_g == pytest.approx(0.8, abs=1e-3)


def test_smoothness_drops_with_lateral_jerk(vehicle):
    calm, jerky = Rig(vehicle), Rig(vehicle)
    for i in range(200):
        calm.tick(lat=0.5 * math.sin(i * DT * 0.5))
        jerky.tick(lat=0.5 if (i // 5) % 2 else -0.5)
    assert calm.h.snapshot().smoothness > 95.0
    assert jerky.h.snapshot().smoothness < 50.0


def test_trail_is_bounded_20hz_and_recent(vehicle):
    rig = Rig(vehicle)
    for i in range(500):  # 10 s
        rig.tick(lat=0.01 * (i % 50), lon=0.0)
    trail = rig.h.g_snapshot().trail
    assert len(trail) <= int(3.0 / 0.05) + 3
    assert len(trail) >= 55
    assert all(len(p) == 2 for p in trail)


def test_friction_envelope_and_decay(vehicle):
    rig = Rig(vehicle)
    rig.tick(lat=1.2, lon=0.0, n=50)
    h = rig.h
    assert h.envelope == pytest.approx(0.98 * 1.2, rel=1e-3)
    assert h.snapshot().friction_used_pct == 100.0
    rig.tick(lat=0.6, lon=0.0, n=50)
    assert h.snapshot().friction_used_pct == pytest.approx(0.6 / h.envelope * 100.0, abs=0.1)
    # ten quiet minutes later the envelope has decayed ~1 %/min, never below the floor
    for _ in range(600):
        rig.t += 1.0
        h.update(rig.t, 80.0, 0.0, 0.0, None, None, None)
    expected = max(0.8, 0.98 * 1.2 * 0.99**10)
    assert h.envelope == pytest.approx(expected, rel=0.01)


def test_no_inputs_no_crash(vehicle):
    rig = Rig(vehicle)
    rig.tick(kph=None, n=10)
    g = rig.h.g_snapshot()
    assert g.source == "none" and g.lat == 0.0 and g.trail == []
    snap = rig.h.snapshot()
    assert snap.smoothness == 0.0 and snap.balance is None
    assert rig.h.lat_g is None and rig.h.lon_g is None
    rig.h.reset()
    assert rig.h.corner_count == 0
