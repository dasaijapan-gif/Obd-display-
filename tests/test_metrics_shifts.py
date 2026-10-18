"""Tests for obddash.metrics.shifts (shift detection and rev-match scoring).

Tick sequences are built from the sim_sportscar gearing and fed through the real
GearEstimator, so these also cover the estimator / analyzer hand-off.
"""

from __future__ import annotations

import pytest

from obddash.config import CONFIG_DIR, RevMatchConfig, load_vehicle
from obddash.metrics.gears import GearEstimator
from obddash.metrics.shifts import (
    ShiftAnalyzer,
    grade_for,
    match_score,
    score_downshift,
    timing_score,
)

DT = 0.02


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


class Rig:
    """GearEstimator + ShiftAnalyzer driven at 50 Hz."""

    def __init__(self, vehicle, cfg: RevMatchConfig | None = None):
        self.v = vehicle
        self.cfg = cfg or RevMatchConfig()
        self.gears = GearEstimator(vehicle)
        self.an = ShiftAnalyzer(self.cfg, vehicle)
        self.t = 0.0
        self.results = []
        self.states: list[str] = []

    def rpk(self, gear):
        return self.v.rpm_per_kph(gear)

    def tick(self, rpm, kph, thr=0.0, brake=False, clutch=None, lon=None, n=1, brake_bar=None):
        for _ in range(n):
            self.t += DT
            est = self.gears.update(self.t, rpm, kph, clutch, None)
            r = self.an.update(self.t, rpm, kph, thr, brake, clutch, est, lon, brake_bar=brake_bar)
            self.states.append(self.an.state)
            if r is not None:
                self.results.append(r)

    def cruise(self, gear, kph, secs, **kw):
        self.tick(kph * self.rpk(gear), kph, n=round(secs / DT), **kw)

    def downshift(
        self,
        frm=4,
        to=3,
        kph=100.0,
        *,
        factor=1.0,
        dur=0.4,
        blip=True,
        brake=True,
        switch=True,
        lon=None,
        spike=0.0,
        after=1.5,
    ):
        """Cruise in ``frm``, clutch in for ``dur`` s with the engine taken to
        ``factor * target`` (rising on a blip at 5000 rpm/s and then held, or
        stepped there without a blip), then re-engage in ``to`` (rpm = target)."""
        bar = 40.0 if brake else 0.0
        self.cruise(frm, kph, 1.0, brake=brake, brake_bar=bar, lon=lon)
        target = kph * self.rpk(to)
        start = kph * self.rpk(frm)
        final = target * factor
        n = round(dur / DT)
        clutch = True if switch else None
        rpm = start
        for _ in range(n):
            if blip and rpm < final:
                rpm = min(final, rpm + 5000.0 * DT)
                thr = 60.0
            else:
                rpm = final
                thr = 15.0 if blip else 0.0
            self.tick(rpm, kph, thr=thr, brake=brake, clutch=clutch, lon=lon, brake_bar=bar)
        self.t_eng = self.t + DT
        released = False if switch else None
        for i in range(round(after / DT)):
            lon_i = None if lon is None else lon - (spike if i < 2 else 0.0)
            self.tick(target, kph, brake=brake, clutch=released, lon=lon_i, brake_bar=bar)
        return target


def scored(rig):
    return [r for r in rig.results if r.kind == "down"]


# --------------------------------------------------------------------------------------
# scoring helpers
# --------------------------------------------------------------------------------------
def test_score_components():
    cfg = RevMatchConfig()
    assert match_score(0.0, cfg) == 100.0
    assert match_score(2.0, cfg) == 100.0
    assert match_score(15.0, cfg) == 0.0
    assert match_score(8.5, cfg) == pytest.approx(50.0)
    # under-rev is weighted x1.5: -8 % counts like 12 %
    assert match_score(-8.0, cfg) == pytest.approx(match_score(12.0, cfg))
    assert timing_score(0.3, cfg) == 5.0
    assert timing_score(2.0, cfg) == -10.0
    assert timing_score(1.0, cfg) == pytest.approx(-2.5)
    assert score_downshift(0.0, 0.3, True, None, cfg) == 100.0
    assert score_downshift(5.0, 0.3, False, None, cfg) == pytest.approx(
        score_downshift(5.0, 0.3, True, None, cfg) - 15.0
    )
    assert score_downshift(5.0, 0.3, True, 0.5, cfg) == pytest.approx(
        score_downshift(5.0, 0.3, True, None, cfg) - 14.0
    )
    assert [grade_for(s) for s in (99, 95, 90, 75, 55, 10)] == ["S", "S", "A", "B", "C", "D"]


# --------------------------------------------------------------------------------------
# downshifts
# --------------------------------------------------------------------------------------
def test_perfect_heel_toe_downshift_scores_s(vehicle):
    rig = Rig(vehicle)
    target = rig.downshift(4, 3, 100.0, factor=1.0, dur=0.4, blip=True, brake=True)
    [r] = scored(rig)
    assert (r.from_gear, r.to_gear, r.kind) == (4, 3, "down")
    assert r.target_rpm == pytest.approx(target, abs=0.5)
    assert r.actual_rpm == pytest.approx(target, abs=0.5)
    assert r.error_pct == pytest.approx(0.0, abs=0.05)
    assert r.duration_s == pytest.approx(0.4, abs=0.025)
    assert r.blip and r.heel_toe
    assert r.score >= 95 and r.grade == "S"
    assert r.t == pytest.approx(rig.t_eng, abs=1e-6)
    s = rig.an.session
    assert (s.count, s.downshifts, s.upshifts, s.heel_toe_count) == (1, 1, 0, 1)
    assert rig.an.last is r


def test_blip_without_brake_is_not_heel_toe(vehicle):
    rig = Rig(vehicle)
    rig.downshift(4, 3, 100.0, brake=False)
    [r] = scored(rig)
    assert r.blip and not r.heel_toe


def test_under_rev_scores_lower_than_over_rev(vehicle):
    under, over = Rig(vehicle), Rig(vehicle)
    under.downshift(4, 3, 100.0, factor=0.92)
    over.downshift(4, 3, 100.0, factor=1.08)
    [ru], [ro] = scored(under), scored(over)
    assert ru.error_pct == pytest.approx(-8.0, abs=0.1)
    assert ro.error_pct == pytest.approx(8.0, abs=0.1)
    assert ru.score < ro.score


def test_no_blip_costs_15_points(vehicle):
    with_blip, no_blip = Rig(vehicle), Rig(vehicle)
    with_blip.downshift(4, 3, 100.0, factor=1.05, blip=True)
    no_blip.downshift(4, 3, 100.0, factor=1.05, blip=False)
    [a], [b] = scored(with_blip), scored(no_blip)
    assert a.blip and not b.blip and not b.heel_toe
    assert a.error_pct == pytest.approx(b.error_pct, abs=0.01)
    assert a.score - b.score == pytest.approx(15.0, abs=0.11)


def test_slow_shift_is_penalised(vehicle):
    quick, slow = Rig(vehicle), Rig(vehicle)
    quick.downshift(4, 3, 100.0, factor=1.05, dur=0.4)
    slow.downshift(4, 3, 100.0, factor=1.05, dur=2.0)
    [a], [b] = scored(quick), scored(slow)
    assert b.duration_s == pytest.approx(2.0, abs=0.025)
    assert a.score - b.score == pytest.approx(15.0, abs=0.11)  # +5 vs -10


def test_ratio_only_shift_without_clutch_switch(vehicle):
    rig = Rig(vehicle)
    rig.downshift(4, 3, 100.0, factor=1.0, switch=False)
    [r] = scored(rig)
    assert (r.from_gear, r.to_gear) == (4, 3)
    assert abs(r.error_pct) < 1.0
    assert r.blip


def test_clutch_drag_is_walked_back_to_the_bite(vehicle):
    """Engine held 12 % over, then dragged onto the target by the slipping clutch:
    the error is the rpm at the bite, not the half-synced rpm just before the lock."""
    rig = Rig(vehicle)
    kph = 100.0
    rig.cruise(4, kph, 1.0, brake=True)
    target = kph * rig.rpk(3)
    for i in range(10):  # clutch in, blip up to +12 %
        rig.tick(target * (0.75 + 0.037 * i), kph, thr=60, brake=True, clutch=True)
    rig.tick(target * 1.12, kph, thr=15, brake=True, clutch=True, n=10)
    for k in range(8):  # pedal up: the clutch pulls the engine down to the target
        rig.tick(target * (1.12 - 0.016 * (k + 1)), kph, thr=15, brake=True, clutch=False)
    rig.tick(target, kph, brake=True, clutch=False, n=80)
    [r] = scored(rig)
    assert r.error_pct == pytest.approx(12.0, abs=0.5)


def test_jolt_penalty_and_brake_release_is_not_a_jolt(vehicle):
    calm, shunt = Rig(vehicle), Rig(vehicle)
    calm.downshift(4, 3, 100.0, factor=1.05, lon=-0.5)
    shunt.downshift(4, 3, 100.0, factor=1.05, lon=-0.5, spike=0.5)
    [a], [b] = scored(calm), scored(shunt)
    assert a.jolt_g == pytest.approx(0.0, abs=1e-6)
    assert b.jolt_g == pytest.approx(0.5, abs=1e-6)
    assert a.score - b.score == pytest.approx(14.0, abs=0.11)  # min(15, 40 * (0.5 - 0.15))

    # releasing the brake 0.1 s after engagement moves lon_g a lot, but is not a shunt
    rig = Rig(vehicle)
    kph = 100.0
    target = kph * rig.rpk(3)
    rig.cruise(4, kph, 1.0, brake=True, brake_bar=40, lon=-0.5)
    rig.tick(target, kph, thr=40, brake=True, clutch=True, lon=-0.5, brake_bar=40, n=20)
    rig.tick(target, kph, brake=True, clutch=False, lon=-0.5, brake_bar=40, n=5)
    rig.tick(target, kph, brake=False, clutch=False, lon=0.0, brake_bar=0, n=70)
    [r] = scored(rig)
    assert r.jolt_g is not None and r.jolt_g < 0.15


def test_no_imu_means_no_jolt(vehicle):
    rig = Rig(vehicle)
    rig.downshift(4, 3, 100.0, lon=None)
    [r] = scored(rig)
    assert r.jolt_g is None


def test_states_during_a_shift(vehicle):
    rig = Rig(vehicle)
    rig.downshift(4, 3, 100.0, dur=0.6)
    seen = rig.states
    assert seen[0] == "idle"
    assert "blip" in seen and "disengaged" in seen and "engaged" in seen
    assert seen[-1] == "idle"
    first_engaged = seen.index("engaged")
    assert seen.index("blip") < seen.index("disengaged") < first_engaged


def test_result_returned_once_after_post_window_with_bounded_traces(vehicle):
    rig = Rig(vehicle)
    rig.cruise(4, 100.0, 2.0)
    rig.downshift(4, 3, 100.0, after=3.0)
    assert len(rig.results) == 1
    r = rig.results[0]
    for tr in (r.rpm_trace, r.throttle_trace, r.brake_trace):
        assert 10 < len(tr) <= 120
        assert all(len(p) == 2 for p in tr)
    t0 = r.t - r.duration_s
    assert r.rpm_trace[0][0] >= t0 - 1.5 - 1e-6
    assert r.rpm_trace[0][0] <= t0 - 1.4
    assert r.rpm_trace[-1][0] <= r.t + 1.0 + 1e-6
    assert max(p[1] for p in r.brake_trace) == pytest.approx(40.0)  # brake trace in bar


def test_back_to_back_downshifts_finalize_early(vehicle):
    rig = Rig(vehicle)
    rig.downshift(5, 4, 110.0, after=0.3)  # next shift starts inside the 1 s window
    rig.downshift(4, 3, 100.0, after=1.5)
    assert [(r.from_gear, r.to_gear) for r in scored(rig)] == [(5, 4), (4, 3)]


# --------------------------------------------------------------------------------------
# upshifts, auto, filters
# --------------------------------------------------------------------------------------
def test_upshifts_counted_not_scored(vehicle):
    rig = Rig(vehicle)
    kph = 6000 / rig.rpk(3)
    rig.cruise(3, kph, 1.0, thr=100)
    rig.tick(5000, kph, thr=0, clutch=True, n=15)
    rig.cruise(4, kph, 1.5, thr=60, clutch=False)
    [r] = rig.results
    assert r.kind == "up" and (r.from_gear, r.to_gear) == (3, 4)
    assert r.score == 0.0 and r.grade == "-"
    s = rig.an.session
    assert (s.count, s.upshifts, s.downshifts) == (1, 1, 0)
    assert s.avg_score == 0.0 and s.best_score == 0.0 and s.history == []
    assert rig.an.last is None


def test_auto_transmission_counts_but_never_scores(vehicle):
    auto = vehicle.model_copy(update={"transmission": "auto"})
    rig = Rig(auto)
    rig.downshift(4, 3, 100.0)
    assert set(rig.states) == {"idle"}
    [r] = rig.results
    assert r.kind == "down" and r.grade == "-" and r.score == 0.0
    s = rig.an.session
    assert s.downshifts == 1 and s.history == [] and s.avg_score == 0.0
    assert rig.an.last is None
    assert rig.an.target_rpm is None


def test_session_averages(vehicle):
    rig = Rig(vehicle)
    for factor in (1.0, 1.06, 0.95):
        rig.downshift(4, 3, 100.0, factor=factor)
    scores = [r.score for r in scored(rig)]
    assert len(scores) == 3
    s = rig.an.session
    # the rig jumps straight back to 4th between downshifts: that counts as an upshift
    assert s.downshifts == 3 and s.upshifts == 2 and s.count == 5
    assert s.avg_score == pytest.approx(sum(scores) / 3, abs=0.01)
    assert s.best_score == max(scores)
    assert s.history == scores
    assert s.heel_toe_count == 3
    snap = rig.an.session_snapshot()
    assert snap.history == scores and snap.history is not s.history


def test_history_is_capped(vehicle):
    rig = Rig(vehicle, RevMatchConfig(history=2))
    for _ in range(3):
        rig.downshift(4, 3, 100.0)
    assert len(rig.an.session.history) == 2
    assert rig.an.session.downshifts == 3


def test_shifts_below_min_speed_are_ignored(vehicle):
    rig = Rig(vehicle, RevMatchConfig(min_speed_kph=50.0))
    rig.downshift(4, 3, 40.0)
    assert rig.results == []
    assert rig.an.session.count == 0


def test_long_neutral_coast_is_not_a_shift(vehicle):
    rig = Rig(vehicle)
    rig.downshift(4, 3, 100.0, dur=4.5, blip=False)
    assert rig.results == []


def test_reset(vehicle):
    rig = Rig(vehicle)
    rig.downshift(4, 3, 100.0)
    rig.an.reset()
    assert rig.an.last is None and rig.an.session.count == 0 and rig.an.state == "idle"


# --------------------------------------------------------------------------------------
# live hint
# --------------------------------------------------------------------------------------
def test_live_target_and_shift_light(vehicle):
    rig = Rig(vehicle)
    rig.cruise(4, 100.0, 0.2)
    assert rig.an.target_gear == 3
    assert rig.an.target_rpm == pytest.approx(100.0 * vehicle.rpm_per_kph(3), abs=0.1)
    assert not rig.an.shift_light
    # 3rd at 150 km/h: 2nd would be far past the redline
    rig2 = Rig(vehicle)
    rig2.cruise(3, 150.0, 0.2)
    assert rig2.an.target_rpm is None and rig2.an.target_gear is None
    # 1st: nothing lower
    rig3 = Rig(vehicle)
    rig3.cruise(1, 30.0, 0.2)
    assert rig3.an.target_gear is None
    # shift light
    rig4 = Rig(vehicle)
    rig4.cruise(2, 7100 / vehicle.rpm_per_kph(2), 0.2)
    assert rig4.an.shift_light
