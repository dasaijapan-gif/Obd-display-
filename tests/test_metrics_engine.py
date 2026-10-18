"""Tests for obddash.metrics.engine (MetricsEngine): RawState in, Telemetry out."""

from __future__ import annotations

import ast
import json
import logging
import math

import pytest

from obddash.config import CONFIG_DIR, MetricsConfig, load_vehicle
from obddash.metrics import MetricsEngine
from obddash.metrics.dyno import CURVE_MAX, RUN_CURVE_MAX, DynoRunRecord
from obddash.metrics.shifts import TRACE_MAX
from obddash.model import (
    Brake,
    BrakeEvent,
    Connection,
    Dyno,
    DynoPoint,
    GForces,
    RawState,
    Shift,
    ShiftResult,
    ShiftSession,
    Sig,
    Telemetry,
)
from obddash.sim.driver import make_driver
from obddash.sim.vehicle import VehicleModel

DT = 0.02
TOP_KEYS = {
    "t",
    "ts",
    "conn",
    "veh",
    "g",
    "brake",
    "shift",
    "dyno",
    "handling",
    "session",
    "alerts",
}


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


class Rig:
    def __init__(self, vehicle, cfg: MetricsConfig | None = None):
        self.v = vehicle
        self.now = 1000.0
        self.raw = RawState(clock=lambda: self.now)
        self.eng = MetricsEngine(cfg or MetricsConfig(), vehicle)
        self.tel = None

    def tick(self, values: dict | None = None, n=1):
        for _ in range(n):
            self.now += DT
            if values:
                self.raw.update(values)
            self.tel = self.eng.tick(self.raw, self.now)
        return self.tel


def cruise_state(vehicle, kph=100.0, gear=4, **over):
    rpm = kph * vehicle.rpm_per_kph(gear)
    d = {
        Sig.SPEED_KPH: kph,
        Sig.RPM: rpm,
        Sig.THROTTLE_PCT: 30.0,
        Sig.ENGINE_LOAD_PCT: 40.0,
        Sig.COOLANT_C: 92.0,
        Sig.INTAKE_C: 30.0,
        Sig.OIL_TEMP_C: 101.0,
        Sig.MAF_GPS: 20.5,
        Sig.MAP_KPA: 60.0,
        Sig.BARO_KPA: 101.0,
        Sig.FUEL_LEVEL_PCT: 48.0,
        Sig.FUEL_RATE_LPH: 8.0,
        Sig.CONTROL_MODULE_V: 14.1,
        Sig.TIMING_ADV_DEG: 22.5,
        Sig.LAMBDA: 1.0,
        Sig.AMBIENT_C: 21.0,
        Sig.ENGINE_TORQUE_PCT: 40.0,
        Sig.ENGINE_REF_TORQUE_NM: 250.0,
        Sig.STEERING_DEG: -12.0,
        Sig.YAW_RATE_DPS: -4.1,
        Sig.BRAKE_PRESSURE_BAR: 0.0,
        Sig.BRAKE_SWITCH: 0.0,
        Sig.CLUTCH_SWITCH: 0.0,
        Sig.ACC_LAT_G: -0.31,
        Sig.ACC_LON_G: 0.05,
        Sig.ACC_VERT_G: 1.0,
        Sig.WHEEL_SPEED_FL_KPH: kph,
        Sig.WHEEL_SPEED_FR_KPH: kph,
        Sig.WHEEL_SPEED_RL_KPH: kph,
        Sig.WHEEL_SPEED_RR_KPH: kph,
    }
    d.update(over)
    return d


# --------------------------------------------------------------------------------------
def test_plausible_state_gives_round_tripping_telemetry(vehicle):
    rig = Rig(vehicle)
    tel = rig.tick(cruise_state(vehicle), n=50)
    assert isinstance(tel, Telemetry)
    d = tel.to_dict()
    assert set(d) == TOP_KEYS
    text = json.dumps(d)
    assert json.loads(text) == d
    v = tel.veh
    assert v.speed_kph == 100.0 and v.gear == 4 and v.gear_source == "ratio"
    assert v.gear_conf == pytest.approx(1.0)
    assert v.boost_kpa == pytest.approx(-41.0)
    assert v.batt_v == 14.1 and v.lambda_ == 1.0 and d["veh"]["lambda"] == 1.0
    assert v.clutch_switch is False and v.brake_switch is False
    assert tel.g.source == "imu" and tel.g.lat == pytest.approx(-0.31, abs=1e-3)
    assert tel.brake.source == "can"
    assert tel.shift.target_gear == 3
    assert tel.dyno.state == "idle"
    assert tel.alerts == []
    assert tel.t == pytest.approx(49 * DT, abs=1e-6)  # first tick is t = 0
    assert tel.session.duration_s == pytest.approx(49 * DT, abs=1e-3)
    assert tel.session.distance_km == pytest.approx(100.0 * 49 * DT / 3600.0, abs=1e-4)
    assert tel.session.fuel_used_l == pytest.approx(8.0 * 49 * DT / 3600.0, abs=1e-4)


def test_empty_raw_never_raises(vehicle):
    rig = Rig(vehicle)
    for _ in range(200):
        tel = rig.tick()
    assert tel.veh.speed_kph == 0.0 and tel.veh.rpm == 0.0 and tel.veh.gear == 0
    assert tel.veh.gear_source == "none" and tel.g.source == "none"
    assert tel.brake.source == "none" and tel.alerts == []
    assert tel.veh.coolant_c is None and tel.veh.boost_kpa is None
    json.dumps(tel.to_dict())


def test_stale_signals_are_dropped(vehicle):
    rig = Rig(vehicle)
    rig.tick(cruise_state(vehicle), n=5)
    rig.now += 1.5  # fast signals (1 s) are now stale, slow ones (2 s) not yet
    tel = rig.tick()
    assert tel.veh.speed_kph == 0.0 and tel.veh.rpm == 0.0
    assert tel.veh.coolant_c == 92.0
    rig.now += 1.0
    tel = rig.tick()
    assert tel.veh.coolant_c is None and tel.g.source == "none"


def test_alerts_with_hysteresis_and_events(vehicle):
    rig = Rig(vehicle)
    rig.tick(cruise_state(vehicle), n=3)
    rig.eng.drain_events()
    tel = rig.tick({Sig.COOLANT_C: 112.0, Sig.CONTROL_MODULE_V: 11.5, Sig.OIL_TEMP_C: 135.0})
    assert set(tel.alerts) == {"COOLANT HIGH", "BATT LOW", "OIL HOT"}
    alerts = [e for e in rig.eng.drain_events() if e.name == "alert"]
    assert sorted(e.data["alert"] for e in alerts) == ["BATT LOW", "COOLANT HIGH", "OIL HOT"]
    tel = rig.tick({Sig.COOLANT_C: 109.0}, n=3)  # within the 2 C hysteresis: still on
    assert "COOLANT HIGH" in tel.alerts
    assert [e for e in rig.eng.drain_events() if e.name == "alert"] == []  # no repeat
    tel = rig.tick({Sig.COOLANT_C: 107.0, Sig.CONTROL_MODULE_V: 14.0, Sig.OIL_TEMP_C: 100.0})
    assert tel.alerts == []
    # NO ECU once the runtime reports the ECU as gone
    rig.eng.set_connection(Connection(bus=True, ecu=False, obd=True, source="can", pid_hz=0.0))
    tel = rig.tick(n=50)
    assert tel.alerts == []  # 2 s grace: start-up, before the first OBD answer
    tel = rig.tick(n=55)
    assert tel.alerts == ["NO ECU"]
    assert tel.conn.bus is True and tel.conn.source == "can"
    rig.eng.set_connection(Connection(bus=True, ecu=True, obd=True, source="can", pid_hz=48.0))
    tel = rig.tick()
    assert tel.alerts == [] and tel.conn.pid_hz == 48.0


def test_commands(vehicle):
    rig = Rig(vehicle)
    rig.tick(cruise_state(vehicle), n=10)
    eng = rig.eng
    assert eng.command("dyno.arm") and rig.tick().dyno.state == "armed"
    assert eng.command("dyno.cancel") and rig.tick().dyno.state == "idle"
    assert eng.command("dyno.clear")
    rig.tick({Sig.BRAKE_PRESSURE_BAR: 50.0}, n=5)
    assert rig.tick().brake.peak_bar == 50.0
    assert eng.command("brake.reset_peak")
    assert rig.tick({Sig.BRAKE_PRESSURE_BAR: 0.0}).brake.peak_bar == 0.0
    assert eng.command("shift.reset")
    assert eng.command("session.reset")
    tel = rig.tick(cruise_state(vehicle))
    assert tel.t == 0.0 and tel.session.distance_km == 0.0
    assert eng.command("no.such.thing") is False
    assert eng.command("") is False


def downshift_states(vehicle, kph=100.0):
    """Cruise in 4th, heel-toe downshift to 3rd with the clutch switch, cruise in 3rd."""
    rpk3, rpk4 = vehicle.rpm_per_kph(3), vehicle.rpm_per_kph(4)
    target = kph * rpk3
    seq = [cruise_state(vehicle, kph, 4, **{Sig.BRAKE_PRESSURE_BAR: 40.0})] * 50
    rpm = kph * rpk4
    for _ in range(20):
        rpm = min(target, rpm + 5000 * DT)
        thr = 60.0 if rpm < target else 15.0
        seq.append(
            cruise_state(
                vehicle,
                kph,
                4,
                **{
                    Sig.RPM: rpm,
                    Sig.THROTTLE_PCT: thr,
                    Sig.CLUTCH_SWITCH: 1.0,
                    Sig.BRAKE_PRESSURE_BAR: 40.0,
                },
            )
        )
    seq += [cruise_state(vehicle, kph, 3, **{Sig.BRAKE_PRESSURE_BAR: 40.0})] * 80
    return seq


def test_downshift_is_scored_and_events_drain(vehicle):
    rig = Rig(vehicle)
    for st in downshift_states(vehicle):
        rig.tick(st)
    events = rig.eng.drain_events()
    scored = [e for e in events if e.name == "shift.scored"]
    assert len(scored) == 1
    data = scored[0].data
    assert data["from_gear"] == 4 and data["to_gear"] == 3 and data["grade"] == "S"
    assert data["heel_toe"] is True
    json.dumps(data)
    assert rig.eng.drain_events() == []
    tel = rig.tel
    assert tel.shift.last is not None and tel.shift.last.grade == "S"
    assert tel.shift.session.downshifts == 1


def test_analyzer_failure_is_contained_and_logged_once(vehicle, caplog):
    rig = Rig(vehicle)
    calls = {"n": 0}

    def boom(*a, **k):
        calls["n"] += 1
        raise RuntimeError("synthetic failure")

    rig.eng.dyno.update = boom
    with caplog.at_level(logging.ERROR, logger="obddash.metrics.engine"):
        for _ in range(100):
            tel = rig.tick(cruise_state(vehicle))
    assert calls["n"] == 100
    assert tel.veh.gear == 4 and tel.dyno.state == "idle"
    failures = [r for r in caplog.records if "dyno failed" in r.getMessage()]
    assert len(failures) == 1


def test_whole_tick_failure_returns_last_snapshot(vehicle):
    rig = Rig(vehicle)
    good = rig.tick(cruise_state(vehicle), n=5)

    class Broken:
        def get(self, *a, **k):
            raise RuntimeError("raw exploded")

    assert rig.eng.tick(Broken(), rig.now + DT) is good


def test_clock_going_backwards_resets_the_session(vehicle):
    rig = Rig(vehicle)
    rig.tick(cruise_state(vehicle), n=100)
    assert rig.tel.t > 1.9
    rig.now -= 100.0
    tel = rig.tick(cruise_state(vehicle))
    assert tel.t == 0.0


def wheels(kph):
    return {
        Sig.WHEEL_SPEED_FL_KPH: kph,
        Sig.WHEEL_SPEED_FR_KPH: kph,
        Sig.WHEEL_SPEED_RL_KPH: kph,
        Sig.WHEEL_SPEED_RR_KPH: kph,
    }


def test_wheel_speeds_feed_the_analyzers(vehicle):
    rig = Rig(vehicle)
    # OBD speed is quantised to 1 km/h; the wheels carry the real 100.4 km/h
    st = cruise_state(vehicle, 100.4, 4, **{Sig.SPEED_KPH: 100.0})
    tel = rig.tick(st, n=10)
    assert tel.veh.speed_kph == 100.0  # display: the car's own speed
    assert rig.eng.gears.estimate.ratio_err_pct == pytest.approx(0.0, abs=1e-6)


def test_mis_scaled_wheel_speeds_fall_back_to_obd_speed(vehicle):
    rig = Rig(vehicle)
    bad = cruise_state(vehicle, 100.0, 4, **wheels(62.0))
    rig.tick(bad, n=60)  # persistent disagreement (> ~1 s)
    tel = rig.tick(bad, n=5)
    assert tel.veh.gear == 4  # ratio from the OBD speed, not the bogus wheels
    assert tel.veh.speed_kph == 100.0
    # a brief disagreement (OBD speed lagging under hard acceleration) is tolerated
    rig2 = Rig(vehicle)
    rig2.tick(cruise_state(vehicle, 100.0, 4), n=20)
    rig2.tick(cruise_state(vehicle, 100.0, 4, **{Sig.SPEED_KPH: 88.0}), n=5)
    assert rig2.eng._wheel_bad < 50


# --------------------------------------------------------------------------------------
# bounded output
# --------------------------------------------------------------------------------------
def sim_drive(vehicle, seconds, script="lap", seed=1):
    """Yield RawState-style dicts from the pure physics model (no CAN), 50 Hz."""
    model = VehicleModel(vehicle, seed=seed)
    driver = make_driver(script, vehicle, seed=seed)
    for i in range(int(seconds / 0.01)):
        s = model.state
        model.step(0.01, driver.get(s.t, s))
        if i % 2:
            continue
        s = model.state
        fl, fr, rl, rr = s.wheel_speeds_kph
        yield (
            {
                Sig.SPEED_KPH: float(int(s.speed_kph)),
                Sig.RPM: round(s.rpm * 4) / 4,
                Sig.THROTTLE_PCT: s.throttle_pct,
                Sig.BRAKE_PRESSURE_BAR: s.brake_bar,
                Sig.BRAKE_PRESSURE_FRONT_BAR: s.brake_front_bar,
                Sig.BRAKE_PRESSURE_REAR_BAR: s.brake_rear_bar,
                Sig.BRAKE_SWITCH: float(s.brake_bar > 2.0),
                Sig.CLUTCH_SWITCH: float(s.clutch > 0.5),
                Sig.STEERING_DEG: s.steering_deg,
                Sig.YAW_RATE_DPS: s.yaw_rate_dps,
                Sig.ACC_LAT_G: s.lat_g,
                Sig.ACC_LON_G: s.lon_g,
                Sig.WHEEL_SPEED_FL_KPH: fl,
                Sig.WHEEL_SPEED_FR_KPH: fr,
                Sig.WHEEL_SPEED_RL_KPH: rl,
                Sig.WHEEL_SPEED_RR_KPH: rr,
                Sig.GEAR: float(s.gear),
                Sig.COOLANT_C: s.coolant_c,
                Sig.INTAKE_C: s.intake_c,
                Sig.BARO_KPA: s.baro_kpa,
                Sig.MAP_KPA: s.map_kpa,
                Sig.FUEL_RATE_LPH: s.fuel_rate_lph,
                Sig.CONTROL_MODULE_V: s.batt_v,
            },
            s,
        )


def test_sizes_stay_bounded_over_5000_ticks(vehicle):
    rig = Rig(vehicle)
    biggest = 0
    n = 0
    for values, _state in sim_drive(vehicle, 100.0):
        tel = rig.tick(values)
        n += 1
        if n % 25 == 0:
            biggest = max(biggest, len(json.dumps(tel.to_dict())))
        rig.eng.drain_events()
    assert n == 5000
    assert biggest < 60_000
    cfg = rig.eng.cfg
    assert len(tel.brake.trace) <= int(cfg.brake.trace_s / 0.04) + 4
    assert len(tel.brake.events) <= cfg.brake.max_events
    assert len(tel.g.trail) <= int(cfg.handling.g_trail_s / 0.05) + 3
    assert len(tel.shift.session.history) <= cfg.revmatch.history
    assert len(tel.dyno.runs) <= cfg.dyno.max_runs
    assert len(tel.dyno.curve) <= CURVE_MAX
    assert tel.shift.session.count > 10 and tel.brake.count >= 3
    assert tel.handling.corner_count >= 5


def test_worst_case_payload_fits(vehicle):
    """Every list at its cap must still serialise to < 60 kB."""
    cfg = MetricsConfig()

    def pt(i):
        return DynoPoint(rpm=2500.0 + 100 * i, power_kw=123.4, torque_nm=187.6, ecu_torque_nm=201.3)

    run = DynoRunRecord(
        id=1,
        t=12345.678,
        gear=3,
        rpm_start=2512.3,
        rpm_end=7312.4,
        peak_power_kw=128.4,
        peak_power_rpm=6900.0,
        peak_torque_nm=187.6,
        peak_torque_rpm=4300.0,
        correction=0.9952,
        curve=[pt(i) for i in range(RUN_CURVE_MAX)],
        peak_crank_kw=149.3,
    )
    trace = [[12345.678, 4567.8]] * TRACE_MAX
    last = ShiftResult(
        t=12345.678,
        from_gear=4,
        to_gear=3,
        kind="down",
        duration_s=0.421,
        target_rpm=4480.1,
        actual_rpm=4310.2,
        error_pct=-3.81,
        blip=True,
        heel_toe=True,
        jolt_g=0.081,
        score=86.3,
        grade="A",
        rpm_trace=trace,
        throttle_trace=trace,
        brake_trace=trace,
    )
    ev = BrakeEvent(12345.678, 2.412, 74.21, 142.3, 61.2, 0.861, 32.1, 81.2)
    tel = Telemetry(
        g=GForces(trail=[[-0.312, 0.223]] * (int(cfg.handling.g_trail_s / 0.05) + 3)),
        brake=Brake(
            events=[ev] * cfg.brake.max_events,
            trace=[[12345.678, 74.21]] * (int(cfg.brake.trace_s / 0.04) + 4),
        ),
        shift=Shift(last=last, session=ShiftSession(history=[86.3] * cfg.revmatch.history)),
        dyno=Dyno(curve=[pt(i) for i in range(CURVE_MAX)], runs=[run] * cfg.dyno.max_runs),
        alerts=["COOLANT HIGH", "OIL HOT", "BATT LOW", "NO ECU"],
    )
    assert len(json.dumps(tel.to_dict())) < 60_000


# --------------------------------------------------------------------------------------
# end to end against the simulator's ground truth
# --------------------------------------------------------------------------------------
def test_sim_lap_shifts_and_dyno_match_ground_truth(vehicle):
    rig = Rig(vehicle)
    model_state = None
    events = []
    for values, s in sim_drive(vehicle, 45.0):
        rig.tick(values)
        events += rig.eng.drain_events()
        model_state = s
    assert model_state is not None
    tel = rig.tel
    downs = [e.data for e in events if e.name == "shift.scored"]
    # the lap's first braking zone (~36 s) has three heel-toe downshifts 5-4-3-2
    assert [(d["from_gear"], d["to_gear"]) for d in downs] == [(5, 4), (4, 3), (3, 2)]
    assert tel.shift.session.upshifts == 4  # the launch run 1-2-3-4-5
    # the launch's 3rd-gear WOT stretch is captured as an (unarmed) dyno run
    runs = [e.data for e in events if e.name == "dyno.done" and e.data.get("ok")]
    assert runs and runs[0]["gear"] == 3
    assert 95.0 < runs[0]["peak_power_kw"] < 125.0
    assert tel.brake.count >= 1


# --------------------------------------------------------------------------------------
# reviewer: adversarial cases
# --------------------------------------------------------------------------------------
def obd_only(vehicle, kph, gear, thr=20.0):
    return {Sig.SPEED_KPH: kph, Sig.RPM: kph * vehicle.rpm_per_kph(gear), Sig.THROTTLE_PCT: thr}


def test_non_finite_raw_values_are_treated_as_missing(vehicle):
    """One NaN speed would otherwise make distance_km NaN for the rest of the session,
    and an inf from the IMU would pin peak_lat / friction_used for good."""
    rig = Rig(vehicle)
    rig.tick(cruise_state(vehicle, 100.0, 4), n=50)
    nan, inf = float("nan"), float("inf")
    bad = {
        Sig.SPEED_KPH: nan,
        Sig.RPM: inf,
        Sig.ACC_LAT_G: inf,
        Sig.ACC_LON_G: nan,
        Sig.BRAKE_PRESSURE_BAR: nan,
        Sig.COOLANT_C: nan,
        Sig.FUEL_RATE_LPH: inf,
        **{s: nan for s in (Sig.WHEEL_SPEED_FL_KPH, Sig.WHEEL_SPEED_FR_KPH)},
        **{s: nan for s in (Sig.WHEEL_SPEED_RL_KPH, Sig.WHEEL_SPEED_RR_KPH)},
    }
    rig.tick(bad, n=5)
    tel = rig.tick(cruise_state(vehicle, 100.0, 4), n=50)
    assert math.isfinite(tel.session.distance_km) and tel.session.distance_km > 0
    assert tel.session.fuel_used_l is not None and math.isfinite(tel.session.fuel_used_l)
    assert math.isfinite(tel.g.peak_lat) and tel.g.peak_lat < 1.0
    assert tel.handling.friction_used_pct < 100.0
    assert math.isfinite(tel.brake.peak_bar)
    assert tel.veh.gear == 4
    json.dumps(tel.to_dict(), allow_nan=False)


def test_ecu_dropout_then_reconnect_in_another_gear(vehicle):
    """The ECU goes silent for 2.5 s (the driver changes 4 -> 3 meanwhile), then
    answers again: the display drops to 0, NO ECU shows, nothing raises, and no
    phantom shift is scored for the engagement nobody saw."""
    rig = Rig(vehicle)
    rig.eng.set_connection(Connection(bus=True, ecu=True, obd=True, source="can"))
    rig.tick(obd_only(vehicle, 100.0, 4), n=100)
    assert rig.tel.veh.gear == 4
    rig.eng.set_connection(Connection(bus=True, ecu=False, obd=True, source="can"))
    tel = rig.tick(None, n=int(2.5 / DT))
    assert (tel.veh.rpm, tel.veh.speed_kph, tel.veh.gear) == (0.0, 0.0, 0)
    assert "NO ECU" in tel.alerts
    rig.eng.set_connection(Connection(bus=True, ecu=True, obd=True, source="can"))
    tel = rig.tick(obd_only(vehicle, 95.0, 3), n=100)
    assert tel.veh.gear == 3 and tel.veh.gear_source == "ratio"
    assert "NO ECU" not in tel.alerts
    names = [e.name for e in rig.eng.drain_events()]
    assert "shift.scored" not in names
    assert tel.shift.session.count == 0 and tel.shift.last is None


def test_repeated_timestamps_do_not_double_count(vehicle):
    """tick() called twice with the same t (dt = 0) must not integrate twice."""
    once, twice = Rig(vehicle), Rig(vehicle)
    state = cruise_state(vehicle, 100.0, 4)
    for _ in range(250):
        once.tick(state)
        twice.now += DT
        twice.raw.update(state)
        twice.eng.tick(twice.raw, twice.now)
        twice.tel = twice.eng.tick(twice.raw, twice.now)
    a, b = once.tel, twice.tel
    assert b.session.distance_km == pytest.approx(a.session.distance_km, rel=1e-9)
    assert b.session.duration_s == pytest.approx(a.session.duration_s)
    assert b.veh.gear == a.veh.gear == 4
    assert len(b.g.trail) == len(a.g.trail)
    assert len(b.brake.trace) == len(a.brake.trace)


@pytest.mark.parametrize(
    "path",
    sorted(
        (CONFIG_DIR.parent / "src" / "obddash" / "metrics").glob("*.py"),
        key=lambda p: p.name,
    ),
    ids=lambda p: p.name,
)
def test_sources_parse_as_python_311(path):
    """The Pi runs Python 3.11: no 3.12+ syntax (PEP 695 ``type`` aliases / generics)."""
    with open(path, encoding="utf-8") as fh:
        ast.parse(fh.read(), filename=str(path), feature_version=(3, 11))


def test_fuzzed_raw_values_never_trip_an_analyzer(vehicle, caplog):
    """Seeded fuzz: random signals (None-by-staleness, negative, huge, odd gear codes),
    repeated / jumping / slightly backwards timestamps, commands and connection flips.
    tick() hides analyzer exceptions behind a log line, so assert there were none."""
    import random

    rng = random.Random(7)
    rig = Rig(vehicle)
    names = Sig.all()
    values = [0.0, -1.0, 1.0, 2.0, 7.0, 15.0, 1e9, -1e9]
    cmds = ["dyno.arm", "dyno.cancel", "dyno.clear", "session.reset", "shift.reset", "x"]
    with caplog.at_level(logging.ERROR, logger="obddash.metrics"):
        for i in range(3000):
            rig.now += rng.choice([0.0, 0.005, 0.02, 0.02, 0.02, 0.05, 0.3])
            for _ in range(rng.randint(0, 6)):
                v = rng.choice(values + [rng.uniform(-50, 300), rng.uniform(0, 9000)])
                rig.raw.set(rng.choice(names), v)
            if rng.random() < 0.01:
                rig.eng.command(rng.choice(cmds))
            if rng.random() < 0.02:
                rig.eng.set_connection(Connection(bus=True, ecu=rng.random() < 0.5, obd=True))
            jitter = rng.random() * 0.5 if rng.random() < 0.01 else 0.0
            tel = rig.eng.tick(rig.raw, rig.now - jitter)
            rig.eng.drain_events()
            if i % 250 == 0:
                json.dumps(tel.to_dict(), allow_nan=False)
    assert [r.getMessage() for r in caplog.records] == []


def test_no_ecu_alert_needs_obd_polling(vehicle):
    """Broadcast-only cars (OBD polling off), replays and source none must not raise
    NO ECU just because no ECU answers requests nobody sent."""
    rig = Rig(vehicle)
    rig.eng.set_connection(Connection(bus=True, ecu=False, obd=False, source="can"))
    tel = rig.tick(cruise_state(vehicle), n=200)  # 4 s, well past the 2 s debounce
    assert "NO ECU" not in tel.alerts
    rig.eng.set_connection(Connection(bus=True, ecu=False, obd=True, source="can"))
    tel = rig.tick(n=200)
    assert "NO ECU" in tel.alerts
