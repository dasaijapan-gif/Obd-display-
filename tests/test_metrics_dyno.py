"""Tests for obddash.metrics.dyno (VirtualDyno).

The synthetic pull integrates the same longitudinal force model the dyno inverts,
driven by a KNOWN crank torque curve, at 1 kHz; the dyno sees it at 50 Hz.  The
wheel power it recovers must match ``torque * omega * drivetrain_efficiency``.
"""

from __future__ import annotations

import math

import pytest

from obddash.config import CONFIG_DIR, DynoConfig, load_vehicle
from obddash.metrics.dyno import HP_PER_KW, VirtualDyno, ordinal, sae_correction
from obddash.metrics.gears import GearEstimator

G = 9.81
DT = 0.02
PEAK_NM, PEAK_RPM = 200.0, 4500.0


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


def crank_torque(rpm: float) -> float:
    x = (rpm - PEAK_RPM) / 3500.0
    return PEAK_NM * (1.0 - 0.35 * x * x)


def synth_pull(vehicle, gear=3, rpm0=2500.0, rpm1=7300.0, wot_s=None, lead_s=1.0, tail_s=1.5):
    """[(speed_kph, rpm, throttle)] at 50 Hz: cruise at rpm0, WOT (until rpm1 or for
    ``wot_s`` seconds), then lift."""
    rpk = vehicle.rpm_per_kph(gear)
    radius = vehicle.tire_circumference_m / (2.0 * math.pi)
    ratio = vehicle.gear_ratios[gear - 1] * vehicle.final_drive
    m, k, eta = vehicle.mass_kg, vehicle.rotational_inertia_factor, vehicle.drivetrain_efficiency
    aero = 0.5 * 1.225 * vehicle.cd * vehicle.frontal_area_m2
    v = rpm0 / rpk / 3.6
    out = [(v * 3.6, v * 3.6 * rpk, 20.0)] * int(lead_s / DT)
    t = 0.0
    while True:
        for _ in range(20):  # 1 kHz integration
            rpm = v * 3.6 * rpk
            force = crank_torque(rpm) * ratio * eta / radius
            v += (force - aero * v * v - vehicle.crr * m * G) / (m * k) * 0.001
        t += DT
        rpm = v * 3.6 * rpk
        out.append((v * 3.6, rpm, 100.0))
        if rpm >= rpm1 or (wot_s is not None and t >= wot_s):
            break
    out += [(v * 3.6, v * 3.6 * rpk, 0.0)] * int(tail_s / DT)
    return out


def true_wheel_peak(vehicle, lo, hi):
    best = max(
        (crank_torque(r) * r * 2 * math.pi / 60 * vehicle.drivetrain_efficiency / 1000, r)
        for r in range(int(lo), int(hi) + 1, 10)
    )
    return best


class Bench:
    def __init__(self, vehicle, cfg: DynoConfig | None = None):
        self.v = vehicle
        self.dyno = VirtualDyno(cfg or DynoConfig(), vehicle)
        self.gears = GearEstimator(vehicle)
        self.t = 0.0
        self.events: list[tuple[str, dict]] = []
        self.snaps = []

    def feed(self, ticks, baro=99.0, intake=25.0, ecu=False, every=0):
        for i, (kph, rpm, thr) in enumerate(ticks):
            self.t += DT
            est = self.gears.update(self.t, rpm, kph, None, None)
            pct = crank_torque(rpm) / 250.0 * 100.0 if ecu else None
            ref = 250.0 if ecu else None
            self.dyno.update(self.t, rpm, kph, thr, est, baro, intake, pct, ref)
            self.events += self.dyno.drain_events()
            if every and i % every == 0:
                self.snaps.append(self.dyno.snapshot())

    def idle(self, secs):
        kph = 2500 / self.v.rpm_per_kph(3)
        self.feed([(kph, 2500.0, 15.0)] * int(secs / DT))

    def names(self):
        return [n for n, _ in self.events]


# --------------------------------------------------------------------------------------
def test_recovers_a_known_torque_curve(vehicle):
    b = Bench(vehicle)
    b.dyno.arm()
    assert b.dyno.state == "armed"
    b.feed(synth_pull(vehicle, gear=3))
    assert b.names() == ["dyno.started", "dyno.done"]
    done = b.events[-1][1]
    assert done["ok"] is True and done["gear"] == 3
    snap = b.dyno.snapshot()
    assert snap.state == "done"
    run = snap.runs[-1]
    truth_kw, truth_rpm = true_wheel_peak(vehicle, run.rpm_start, run.rpm_end)
    assert snap.peak_power_kw == pytest.approx(truth_kw, rel=0.05)
    assert abs(snap.peak_torque_rpm - PEAK_RPM) <= 300
    assert snap.peak_torque_nm == pytest.approx(PEAK_NM * vehicle.drivetrain_efficiency, rel=0.05)
    assert abs(snap.peak_power_rpm - truth_rpm) <= 400
    assert snap.wheel_hp == pytest.approx(snap.peak_power_kw * HP_PER_KW, abs=0.1)
    assert run.peak_crank_kw == pytest.approx(
        run.peak_power_kw / vehicle.drivetrain_efficiency, rel=1e-3
    )
    assert run.correction == pytest.approx(1.0, abs=0.002)  # baro 99, intake 25 C
    assert snap.message.startswith("PEAK ") and "whp @" in snap.message
    assert 3 <= len(snap.curve) <= 120 and len(run.curve) <= 60
    assert snap.progress == 1.0
    # the curve tracks the truth across the range, not just at the peak
    for pt in snap.curve[3:-3]:
        truth = crank_torque(pt.rpm) * vehicle.drivetrain_efficiency
        assert pt.torque_nm == pytest.approx(truth, rel=0.04)
    assert not b.dyno.armed  # the arm is consumed


def test_live_values_while_pulling(vehicle):
    b = Bench(vehicle)
    b.dyno.arm()
    b.feed(synth_pull(vehicle, gear=3), every=25)
    pulling = [s for s in b.snaps if s.state == "pulling"]
    assert len(pulling) > 10
    mid = pulling[len(pulling) // 2]
    assert mid.message == "PULLING … hold it"
    assert mid.gear == 3
    assert 0.0 < mid.progress < 1.0
    assert mid.live_rpm > 2500 and mid.live_power_kw > 0 and mid.live_torque_nm > 0
    assert mid.curve and mid.peak_power_kw > 0
    progress = [s.progress for s in pulling]
    assert progress == sorted(progress)


def test_ecu_torque_overlay(vehicle):
    b = Bench(vehicle)
    b.dyno.arm()
    b.feed(synth_pull(vehicle, gear=3), ecu=True)
    curve = b.dyno.snapshot().curve
    assert all(p.ecu_torque_nm is not None for p in curve)
    for p in curve[3:-3]:
        # ECU torque is crank torque: not corrected, not multiplied by efficiency
        assert p.ecu_torque_nm == pytest.approx(crank_torque(p.rpm), rel=0.02)


def test_too_short_pull_is_rejected_then_returns_to_armed(vehicle):
    b = Bench(vehicle)
    b.dyno.arm()
    b.feed(synth_pull(vehicle, gear=3, wot_s=1.0, tail_s=0.2))
    snap = b.dyno.snapshot()
    assert snap.state == "rejected"
    assert "too short" in snap.message
    assert b.events[-1][0] == "dyno.done" and b.events[-1][1]["ok"] is False
    assert snap.runs == []
    b.idle(3.2)
    assert b.dyno.state == "armed"  # still armed: try again


def test_small_rpm_span_is_rejected(vehicle):
    b = Bench(vehicle)
    b.feed(synth_pull(vehicle, gear=3, wot_s=3.0, tail_s=0.2))  # ~900 rpm in 3 s
    snap = b.dyno.snapshot()
    assert snap.state == "rejected" and "span" in snap.message
    b.idle(3.2)
    assert b.dyno.state == "idle"


def test_unarmed_capture_only_in_preferred_gears(vehicle):
    b = Bench(vehicle)
    b.feed(synth_pull(vehicle, gear=1, rpm0=2500, rpm1=7300))
    assert b.events == [] and b.dyno.state == "idle" and b.dyno.runs == []
    b.feed(synth_pull(vehicle, gear=3))
    assert b.names() == ["dyno.done"]  # silent: no dyno.started without an arm
    assert b.dyno.state == "done" and len(b.dyno.runs) == 1


def test_armed_pull_in_any_gear(vehicle):
    b = Bench(vehicle)
    b.dyno.arm()
    b.feed(synth_pull(vehicle, gear=2, rpm0=2500, rpm1=7300))
    assert b.names() == ["dyno.started", "dyno.done"]
    assert b.dyno.runs[-1].gear == 2


def test_arm_cancel_clear(vehicle):
    b = Bench(vehicle)
    assert b.dyno.snapshot().message == "WOT in 3rd or 4th from ~2500 rpm to redline"
    b.dyno.arm()
    assert b.dyno.snapshot().message == "ARMED — floor it"
    b.dyno.cancel()
    assert b.dyno.state == "idle" and not b.dyno.armed
    b.dyno.arm()
    b.feed(synth_pull(vehicle, gear=3))
    assert len(b.dyno.runs) == 1
    b.dyno.clear()
    snap = b.dyno.snapshot()
    assert snap.state == "idle" and snap.runs == [] and snap.curve == []
    assert snap.peak_power_kw == 0.0 and snap.wheel_hp == 0.0
    # cancel in the middle of a pull aborts it
    b.dyno.arm()
    ticks = synth_pull(vehicle, gear=3)
    b.feed(ticks[: len(ticks) // 2])
    assert b.dyno.state == "pulling"
    b.dyno.cancel()
    assert b.dyno.state == "idle"
    assert b.dyno.drain_events() == [("dyno.done", {"ok": False, "reason": "cancelled", "gear": 3})]
    b.feed(ticks[len(ticks) // 2 :])
    assert b.dyno.runs == []


def test_done_returns_to_idle_and_keeps_the_curve(vehicle):
    b = Bench(vehicle)
    b.feed(synth_pull(vehicle, gear=3))
    assert b.dyno.state == "done"
    b.idle(10.5)
    snap = b.dyno.snapshot()
    assert snap.state == "idle" and snap.curve and snap.peak_power_kw > 0


def test_max_runs_kept(vehicle):
    b = Bench(vehicle, DynoConfig(max_runs=2))
    pull = synth_pull(vehicle, gear=4, rpm0=3000, rpm1=5200)
    for _ in range(3):
        b.feed(pull)
        b.idle(1.0)
    assert [r.id for r in b.dyno.runs] == [2, 3]


def test_sae_correction_factor(vehicle):
    assert sae_correction(99.0, 25.0) == pytest.approx(1.0, abs=1e-3)
    assert sae_correction(97.0, 30.0) == pytest.approx(
        1.18 * (99.0 / 97.0) * math.sqrt(303.0 / 298.0) - 0.18
    )
    assert sae_correction(80.0, 50.0) == 1.1  # clamped
    assert sae_correction(110.0, -30.0) == 0.9
    assert sae_correction(None, 25.0) == 1.0 and sae_correction(99.0, None) == 1.0

    pull = synth_pull(vehicle, gear=4, rpm0=3000, rpm1=5500)
    raw_b = Bench(vehicle, DynoConfig(correction="none"))
    raw_b.feed(pull, baro=97.0, intake=30.0)
    sae_b = Bench(vehicle, DynoConfig(correction="sae"))
    sae_b.feed(pull, baro=97.0, intake=30.0)
    cf = sae_correction(97.0, 30.0)
    r_raw, r_sae = raw_b.dyno.runs[-1], sae_b.dyno.runs[-1]
    assert r_raw.correction == 1.0
    assert r_sae.correction == pytest.approx(cf, abs=1e-4)
    assert r_sae.peak_power_kw == pytest.approx(r_raw.peak_power_kw * cf, rel=1e-3)
    assert r_sae.peak_torque_nm == pytest.approx(r_raw.peak_torque_nm * cf, rel=1e-3)
    # unknown baro / intake: no correction
    none_b = Bench(vehicle)
    none_b.feed(pull, baro=None, intake=None)
    assert none_b.dyno.runs[-1].correction == 1.0


def test_signal_loss_ends_the_pull_without_raising(vehicle):
    b = Bench(vehicle)
    b.dyno.arm()
    ticks = synth_pull(vehicle, gear=3, wot_s=1.0, tail_s=0.0)
    b.feed(ticks)
    est = b.gears.estimate
    b.dyno.update(b.t + DT, None, None, None, est)
    assert b.dyno.state == "rejected"
    b.dyno.update(b.t + 2 * DT, None, None, None, est)  # still fine


def test_ordinal():
    assert [ordinal(n) for n in (1, 2, 3, 4, 11, 12, 13, 21, 22)] == [
        "1st",
        "2nd",
        "3rd",
        "4th",
        "11th",
        "12th",
        "13th",
        "21st",
        "22nd",
    ]
