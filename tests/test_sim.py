"""Tests for the simulator physics (sim.vehicle) and the scripted drivers (sim.driver)."""

from __future__ import annotations

import math

import pytest

from obddash.config import CONFIG_DIR, load_vehicle
from obddash.sim import Simulator
from obddash.sim.driver import SCRIPTS, LapDriver, make_driver, steady_throttle
from obddash.sim.vehicle import Controls, SimParams, VehicleModel

DT = 0.01


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


def _model_at(vehicle, kph: float, gear: int, *, seed: int = 1) -> VehicleModel:
    """A model already rolling at ``kph`` locked in ``gear`` (or coasting if gear 0)."""
    m = VehicleModel(vehicle, seed=seed)
    m.state.speed_mps = kph / 3.6
    rpk = vehicle.rpm_per_kph(gear)
    m.state.rpm = kph * rpk if rpk else vehicle.idle_rpm
    return m


# --------------------------------------------------------------------------------------
# engine / torque curve
# --------------------------------------------------------------------------------------
def test_torque_curve_matches_sim_sportscar_spec(vehicle):
    p = SimParams()
    red = vehicle.redline_rpm
    assert p.wot_torque(4000, red) == pytest.approx(205.0)
    kw_7000 = p.wot_torque(7000, red) * 7000 * 2 * math.pi / 60 / 1000
    assert 128 <= kw_7000 <= 142
    # smooth, positive and single-peaked around 4000
    curve = [p.wot_torque(r, red) for r in range(800, 7600, 100)]
    assert all(t > 100 for t in curve)
    assert max(curve) == pytest.approx(205.0, abs=0.5)


def test_idle_settles_near_configured_idle(vehicle):
    m = VehicleModel(vehicle, seed=1)
    for _ in range(500):
        s = m.step(DT, Controls())
    assert abs(s.rpm - vehicle.idle_rpm) < 40
    assert s.speed_mps == 0.0
    assert 13.5 < s.batt_v < 14.5
    assert 25 <= s.map_kpa <= 40


# --------------------------------------------------------------------------------------
# driveline lock: rpm == speed * rpm_per_kph(gear)
# --------------------------------------------------------------------------------------
def test_locked_in_third_rpm_follows_ratio_exactly(vehicle):
    m = _model_at(vehicle, 60.0, 3)
    rpk = vehicle.rpm_per_kph(3)
    for i in range(300):
        thr = 100.0 if i < 150 else 0.0
        s = m.step(DT, Controls(throttle_pct=thr, gear=3, clutch=0.0))
        assert s.locked
        assert s.rpm == pytest.approx(s.speed_kph * rpk, rel=0.005)
        assert s.gear == 3


def test_wot_in_third_from_40kph_accelerates_in_a_sane_band(vehicle):
    m = _model_at(vehicle, 40.0, 3)
    lon = []
    for _ in range(300):
        s = m.step(DT, Controls(throttle_pct=100.0, gear=3))
        lon.append(s.lon_g)
    # 205 Nm through 3rd (4.57:1) on 1130 kg x 1.08 is ~0.17 g; anything in 0.12-0.4 g is sane
    assert 0.12 < min(lon[10:]) and max(lon) < 0.4
    assert s.speed_kph > 52.0
    assert s.map_kpa > 95 and s.load_pct > 90 and s.lambda_ < 0.9


def test_rev_limiter_holds_rpm_near_redline(vehicle):
    m = _model_at(vehicle, 50.0, 1)
    peak = 0.0
    for _ in range(600):
        s = m.step(DT, Controls(throttle_pct=100.0, gear=1))
        peak = max(peak, s.rpm)
    assert vehicle.redline_rpm - 200 < peak < vehicle.redline_rpm + 150


def test_engine_braking_when_throttle_closed_in_gear(vehicle):
    m = _model_at(vehicle, 120.0, 3)
    s = None
    for _ in range(100):
        s = m.step(DT, Controls(throttle_pct=0.0, gear=3))
    assert s.engine_torque_nm < 0
    assert -0.35 < s.lon_g < -0.08
    assert s.fuel_rate_lph == 0.0  # overrun fuel cut


# --------------------------------------------------------------------------------------
# braking
# --------------------------------------------------------------------------------------
def test_80_bar_braking_is_about_one_g(vehicle):
    m = _model_at(vehicle, 100.0, 0)
    lon = []
    for _ in range(100):
        s = m.step(DT, Controls(brake_bar=80.0, clutch=1.0, gear=3))
        lon.append(s.lon_g)
    assert -1.1 <= min(lon) and max(lon[5:]) < -0.9
    assert s.brake_bar == 80.0
    assert s.brake_front_bar == pytest.approx(80.0)
    assert s.brake_rear_bar == pytest.approx(80.0 * 0.35 / 0.65, rel=0.01)


def test_brakes_hold_the_car_at_a_standstill(vehicle):
    m = _model_at(vehicle, 20.0, 0)
    for _ in range(600):
        s = m.step(DT, Controls(brake_bar=60.0))
    assert s.speed_mps == 0.0
    assert s.lon_g == pytest.approx(0.0, abs=0.01)
    assert s.wheel_speeds_kph == (0.0, 0.0, 0.0, 0.0)


# --------------------------------------------------------------------------------------
# free engine (clutch down) and clutch engagement
# --------------------------------------------------------------------------------------
def test_blip_with_clutch_down_raises_rpm_then_decays(vehicle):
    m = _model_at(vehicle, 50.0, 3)
    rpm0 = m.state.rpm
    trace = []
    for i in range(150):
        thr = 100.0 if i < 40 else 0.0
        s = m.step(DT, Controls(throttle_pct=thr, gear=3, clutch=1.0))
        trace.append(s.rpm)
        assert not s.locked
    peak = max(trace)
    rise = (peak - rpm0) / 0.4
    assert 2500 < rise < 4500  # ~3000-3500 rpm/s at WOT
    assert trace[-1] < peak - 800  # decays toward idle afterwards
    assert trace[-1] > vehicle.idle_rpm


def test_free_engine_settles_at_idle_in_neutral_when_blip_ends(vehicle):
    m = VehicleModel(vehicle, seed=1)
    for i in range(500):
        s = m.step(DT, Controls(throttle_pct=100.0 if i < 30 else 0.0, gear=0))
    assert abs(s.rpm - vehicle.idle_rpm) < 60


def test_clutch_engage_with_mismatch_produces_jolt_scaling_with_mismatch(vehicle):
    """Clutch released over 0.2 s (as the driver does) onto an under-revving engine: the
    extra deceleration spike grows with the mismatch until the clutch capacity caps it."""
    target = 80.0 * vehicle.rpm_per_kph(2)

    def jolt(mismatch_rpm: float) -> float:
        m = _model_at(vehicle, 80.0, 0)
        m.state.rpm = target - mismatch_rpm
        base = None
        worst = 0.0
        for i in range(50):
            clutch = max(0.0, 1.0 - i * DT / 0.2)
            thr = steady_throttle(target)
            s = m.step(DT, Controls(throttle_pct=thr, gear=2, clutch=clutch, brake_bar=30.0))
            if base is None:
                base = s.lon_g
            worst = max(worst, base - s.lon_g)
        assert s.locked
        assert s.rpm == pytest.approx(s.speed_kph * vehicle.rpm_per_kph(2), rel=0.005)
        return worst

    j_none, j_small, j_mid, j_big = jolt(0), jolt(100), jolt(300), jolt(800)
    assert j_none < 0.03
    assert j_none < j_small < j_mid < j_big
    assert j_big > 0.3


def test_launch_from_rest_locks_up_and_accelerates(vehicle):
    m = VehicleModel(vehicle, seed=1)
    s = None
    for i in range(400):
        t = i * DT
        clutch = 1.0 if t < 0.3 else max(0.0, 1.0 - (t - 0.3) / 0.9)
        s = m.step(DT, Controls(throttle_pct=60.0 if t >= 0.3 else 0.0, gear=1, clutch=clutch))
        assert s.rpm > 500  # anti-stall: the clutch slips instead of stalling the engine
    assert s.locked
    assert s.speed_kph > 30
    assert s.rpm == pytest.approx(s.speed_kph * vehicle.rpm_per_kph(1), rel=0.005)


# --------------------------------------------------------------------------------------
# lateral
# --------------------------------------------------------------------------------------
def test_cornering_left_gives_positive_lat_g_below_grip(vehicle):
    m = _model_at(vehicle, 100.0, 0)
    p = m.p
    s = None
    for _ in range(100):
        s = m.step(DT, Controls(throttle_pct=30.0, gear=0, steering_deg=60.0))
    assert 0.3 < s.lat_g <= p.max_grip_g + 0.01
    assert s.yaw_rate_dps > 0
    fl, fr, rl, rr = s.wheel_speeds_kph
    assert fl < fr and rl < rr  # inside (left) wheels slower in a left turn
    assert abs(fl - fr) / s.speed_kph < 0.05
    assert math.hypot(s.lat_g, s.lon_g) <= p.max_grip_g + 0.02


def test_small_steering_input_matches_ackermann_and_yaw_lags(vehicle):
    m = _model_at(vehicle, 100.0, 0)
    v = 100.0 / 3.6
    delta = math.radians(10.0 / vehicle.steering_ratio)
    expected_lat = v * v * math.tan(delta) / vehicle.wheelbase_m / 9.80665
    first = m.step(DT, Controls(gear=0, steering_deg=10.0)).lat_g
    s = None
    for _ in range(100):
        s = m.step(DT, Controls(gear=0, steering_deg=10.0))
    assert first < 0.5 * expected_lat  # yaw rate lags the step input
    assert s.lat_g == pytest.approx(expected_lat, rel=0.08)
    assert s.steering_deg == 10.0


def test_right_turn_is_negative(vehicle):
    m = _model_at(vehicle, 60.0, 0)
    s = None
    for _ in range(60):
        s = m.step(DT, Controls(gear=0, steering_deg=-40.0))
    assert s.lat_g < -0.2 and s.yaw_rate_dps < 0


# --------------------------------------------------------------------------------------
# slow signals & robustness
# --------------------------------------------------------------------------------------
def test_slow_signals_warm_up_and_drift_plausibly(vehicle):
    m = VehicleModel(vehicle, seed=1)
    s = None
    for _ in range(int(240 / 0.05)):  # four minutes at 20 Hz
        s = m.step(0.05, Controls(throttle_pct=30.0, gear=0))
    assert 85 < s.coolant_c < 100
    assert s.oil_temp_c > 70
    assert 25 <= s.intake_c <= 45
    assert s.fuel_pct < SimParams().fuel_start_pct
    assert 0 <= s.timing_adv_deg <= 45
    assert s.maf_gps > 0 and s.fuel_rate_lph > 0


def test_step_tolerates_garbage_inputs(vehicle):
    m = VehicleModel(vehicle, seed=1)
    m.step(0.0, Controls())
    m.step(-1.0, Controls())
    m.step(float("nan"), Controls())
    s = m.step(0.5, Controls(throttle_pct=1e9, brake_bar=-5, clutch=7, gear=42, steering_deg=1e6))
    assert math.isfinite(s.rpm) and math.isfinite(s.speed_mps) and math.isfinite(s.lat_g)
    assert s.gear == 42 and not s.locked  # not a forward gear: treated as neutral


def test_nan_and_none_controls_do_not_poison_the_state(vehicle):
    """_clamp lets NaN through; one NaN pedal value must not leave rpm/speed NaN forever."""
    m = _model_at(vehicle, 60.0, 3)
    fields = ("rpm", "speed_mps", "lat_g", "lon_g", "yaw_rate_dps", "engine_torque_nm")
    for name in ("throttle_pct", "brake_bar", "clutch", "steering_deg", "gear"):
        for bad in (float("nan"), float("inf") if name == "gear" else float("nan")):
            c = Controls(throttle_pct=20.0, gear=3)
            setattr(c, name, bad)
            s = m.step(DT, c)
            assert all(math.isfinite(getattr(s, f)) for f in fields), name
    m.step(DT, Controls(throttle_pct=None, brake_bar=None, clutch=None, gear=None))
    for _ in range(50):
        s = m.step(DT, Controls(throttle_pct=20.0, gear=3))
    assert s.locked and s.speed_kph > 50 and math.isfinite(s.fuel_pct)


def test_coasting_car_comes_to_rest_and_can_pull_away_again(vehicle):
    """Rolling resistance is off below 0.05 m/s; the car must still stop, not creep."""
    m = _model_at(vehicle, 7.2, 0)
    for _ in range(6000):  # 60 s coasting in neutral, no brakes
        s = m.step(DT, Controls(gear=0))
    assert s.speed_mps == 0.0
    parked_at = s.distance_m
    for _ in range(1000):
        s = m.step(DT, Controls(gear=0))
    assert s.distance_m == parked_at and s.wheel_speeds_kph == (0.0, 0.0, 0.0, 0.0)
    for i in range(300):  # and a normal launch still works from there
        clutch = max(0.0, 1.0 - i * DT / 0.9)
        s = m.step(DT, Controls(throttle_pct=40.0, gear=1, clutch=clutch))
    assert s.speed_kph > 5.0 and s.locked


def test_substepping_large_dt_matches_small_steps_closely(vehicle):
    a = _model_at(vehicle, 50.0, 3)
    b = _model_at(vehicle, 50.0, 3)
    a.p.noise = b.p.noise = 0.0
    for _ in range(100):
        a.step(0.01, Controls(throttle_pct=80.0, gear=3))
    for _ in range(10):
        b.step(0.1, Controls(throttle_pct=80.0, gear=3))
    assert a.state.speed_mps == pytest.approx(b.state.speed_mps, rel=0.01)


# --------------------------------------------------------------------------------------
# drivers
# --------------------------------------------------------------------------------------
def _run(script: str, seconds: float, seed: int = 1):
    sim = Simulator(
        load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml"),
        None,
        script=script,
        seed=seed,
        realtime=False,
    )
    trace = []
    for _ in range(int(seconds / DT)):
        s = sim.step(DT)
        trace.append(
            (
                s.t,
                s.speed_kph,
                s.rpm,
                s.gear,
                s.clutch,
                s.brake_bar,
                s.lat_g,
                s.lon_g,
                s.throttle_pct,
                s.steering_deg,
            )
        )
    return sim, trace


def _count_downshifts(trace) -> int:
    """Gear decreased with the clutch pedal down in between."""
    n = 0
    prev_gear = trace[0][3]
    clutch_seen = False
    for _, _, _, gear, clutch, *_ in trace:
        if clutch > 0.5:
            clutch_seen = True
        if gear != prev_gear:
            if gear < prev_gear and gear >= 1 and clutch_seen:
                n += 1
            clutch_seen = False
            prev_gear = gear
    return n


def test_lap_script_downshifts_brakes_and_gets_fast():
    sim, trace = _run("lap", 120.0)
    assert _count_downshifts(trace) >= 3
    assert max(r[1] for r in trace) > 140.0
    assert max(r[5] for r in trace) > 50.0
    assert min(r[6] for r in trace) < -0.7  # the 0.9 g right-hander
    assert max(r[6] for r in trace) > 0.4  # the 0.6 g left
    assert sim.driver.shift.n_down >= 3 and sim.driver.shift.n_up >= 4
    assert max(r[2] for r in trace) < sim.vehicle.redline_rpm + 100
    assert isinstance(sim.driver, LapDriver) and sim.driver.lap_count >= 1


def test_lap_downshift_blips_hit_the_target_most_of_the_time():
    sim, trace = _run("lap", 200.0)
    records = [r for r in sim.driver.shift.history if r.kind == "down"]
    assert len(records) >= 6
    styles = {r.style for r in records}
    assert "normal" in styles
    assert all(0.3 <= r.duration_s <= 1.0 for r in records)
    # blips raise the rpm while the clutch is down on normal/overrev downshifts
    by_t = {round(t, 2): rpm for t, _, rpm, *_ in trace}
    for r in records:
        if r.style == "lazy":
            continue
        start = round(r.t - r.duration_s, 2)
        window = [by_t[k] for k in by_t if start <= k <= round(r.t, 2)]
        assert max(window) > window[0] + 300


def test_lap_script_is_seed_reproducible_and_seed_sensitive():
    _, a = _run("lap", 40.0, seed=7)
    _, b = _run("lap", 40.0, seed=7)
    _, c = _run("lap", 40.0, seed=8)
    assert a == b
    assert a != c


def test_dyno_script_pulls_in_third_at_wot():
    sim, trace = _run("dyno", 45.0)
    assert sim.driver.pulls >= 1
    pull_rows = [r for r in trace if r[3] == 3 and r[8] > 95 and r[4] < 0.1]
    rpms = [r[2] for r in pull_rows]
    assert max(rpms) - min(rpms) > 4000  # a wide rpm sweep in one gear
    assert min(rpms) < 2800 and max(rpms) > 7200


def test_idle_script_stays_parked_but_busy():
    sim, trace = _run("idle", 40.0)
    assert max(r[1] for r in trace) == 0.0
    assert max(r[2] for r in trace) > 1800  # blips
    assert max(r[5] for r in trace) > 5  # brake presses
    assert max(abs(r[6]) for r in trace) < 0.05  # parked: no lateral g
    assert max(abs(r[9]) for r in trace) > 30  # steering wheel moved
    assert any(r[4] > 0.5 for r in trace)  # a clutch press


def test_highway_script_cruises_in_sixth():
    sim, trace = _run("highway", 90.0)
    tail = trace[-1500:]
    assert all(r[3] == 6 for r in tail)
    assert all(95 < r[1] < 118 for r in tail)
    assert max(abs(r[6]) for r in tail) < 0.4  # gentle lane changes only
    assert max(abs(r[6]) for r in tail) > 0.05


@pytest.mark.parametrize("n_gears", [4, 5])
def test_scripts_cope_with_a_shorter_gearbox(vehicle, n_gears):
    """`obddash sim --vehicle` may load a 4/5-speed car: the lap must still get past
    its straight (cruise gear capped at top) and the highway must cruise in top."""
    short = vehicle.model_copy(update={"gear_ratios": vehicle.gear_ratios[:n_gears]})
    lap = Simulator(short, None, script="lap", seed=1, realtime=False)
    lap.run(120.0, DT)
    assert lap.driver.lap_count >= 1 and lap.driver.shift.n_down >= 3
    hwy = Simulator(short, None, script="highway", seed=1, realtime=False)
    hwy.run(90.0, DT)
    assert hwy.state.gear == n_gears and 95 < hwy.state.speed_kph < 118


def test_make_driver_rejects_unknown_script(vehicle):
    with pytest.raises(ValueError):
        make_driver("drift", vehicle)
    assert set(SCRIPTS) == {"lap", "dyno", "idle", "highway"}


def test_simulator_realtime_thread_runs_and_stops(vehicle):
    import time

    sim = Simulator(vehicle, None, script="idle", seed=3, realtime=True, rate_hz=200.0)
    sim.speed = 4.0
    sim.start()
    time.sleep(0.3)
    sim.stop()
    assert not sim.running
    assert 0.2 < sim.t < 3.0  # ~0.3 s wall clock x 4 (less on a starved CI box)


def test_realtime_speed_multiplier_does_not_build_a_backlog(vehicle):
    """Asking for more than the per-tick step cap must not pile up debt that keeps the
    sim fast-forwarding after `speed` is lowered (here: paused)."""
    import time

    sim = Simulator(vehicle, None, script="idle", seed=3, realtime=True, rate_hz=100.0)
    sim.speed = 500.0  # 10x beyond the 50 steps/tick cap
    sim.start()
    try:
        time.sleep(0.3)
        sim.speed = 0.0
        time.sleep(0.05)
        t_paused = sim.t
        time.sleep(0.3)
        drift = sim.t - t_paused
    finally:
        sim.stop()
    assert t_paused > 1.0  # it did fast-forward
    assert drift < 0.05  # and stopped at once: no hidden backlog being paid off
