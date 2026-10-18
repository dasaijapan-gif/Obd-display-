"""Tests for obddash.metrics.brakes (BrakeAnalyzer)."""

from __future__ import annotations

import pytest

from obddash.config import BrakeConfig, BrakeMetricsConfig
from obddash.metrics.brakes import BrakeAnalyzer

DT = 0.02


class Rig:
    def __init__(self, cfg: BrakeMetricsConfig | None = None, brake_cfg: BrakeConfig | None = None):
        self.cfg = cfg or BrakeMetricsConfig()
        self.bc = brake_cfg or BrakeConfig(max_pressure_bar=120.0)
        self.an = BrakeAnalyzer(self.cfg, self.bc)
        self.t = 0.0
        self.events = []

    def tick(self, p=None, sw=None, lon=None, lat=None, kph=100.0, n=1, front=None, rear=None):
        for _ in range(n):
            self.t += DT
            ev = self.an.update(self.t, p, front, rear, sw, lon, lat, kph)
            if ev is not None:
                self.events.append(ev)


def trail_braking_stop(rig: Rig, peak=80.0, v0=140.0, v1=60.0):
    """0.2 s ramp to ``peak``, 1.5 s hold (lateral g for the second half), 0.3 s
    release, while speed falls linearly v0 -> v1 over the 2 s of braking."""
    rig.tick(p=0.0, kph=v0, lon=0.0, lat=0.0, n=25)
    seq = []
    for i in range(10):
        seq.append((peak * (i + 1) / 10, 0.0))
    for i in range(75):
        seq.append((peak, 0.0 if i < 38 else 0.5))
    for i in range(15):
        seq.append((peak * (1 - (i + 1) / 15), 0.5))
    n = len(seq)
    for i, (p, lat) in enumerate(seq):
        kph = v0 + (v1 - v0) * (i + 1) / n
        rig.tick(p=p, kph=kph, lon=-p / 80.0, lat=lat)
    rig.tick(p=0.0, kph=v1, lon=0.0, lat=0.0, n=25)


def test_can_pressure_event_with_trail_braking():
    rig = Rig()
    trail_braking_stop(rig)
    assert len(rig.events) == 1
    ev = rig.events[0]
    assert ev.entry_kph == pytest.approx(140.0, abs=1.0)
    # released when the pressure fell below 2 bar, ~1 tick before the end of the ramp
    assert ev.exit_kph == pytest.approx(61.0, abs=1.5)
    assert ev.peak_bar == pytest.approx(80.0)
    assert ev.duration_s == pytest.approx(1.98, abs=0.05)
    # lateral g above 0.25 on the second half of the hold and the release
    active = 9 + 75 + 14  # samples >= 2 bar
    trail = 37 + 14
    assert ev.trail_pct == pytest.approx(100.0 * trail / active, abs=1.5)
    assert ev.avg_decel_g == pytest.approx(0.9, abs=0.05)
    assert 0.0 <= ev.smoothness <= 100.0
    snap = rig.an.snapshot()
    assert snap.source == "can"
    assert snap.count == 1 and snap.events == [ev]
    assert snap.peak_bar == pytest.approx(80.0)
    assert not snap.active and snap.pressure_bar == 0.0
    expected = 0.6 * ev.smoothness + 0.4 * min(100.0, 80.0 / 120.0 * 130.0)
    assert snap.score == pytest.approx(expected, abs=0.1)


def test_live_values_during_braking():
    rig = Rig()
    rig.tick(p=60.0, front=40.0, rear=20.0, lon=-0.7, lat=0.4, n=10)
    snap = rig.an.snapshot()
    assert snap.active and snap.source == "can"
    assert snap.pressure_bar == 60.0
    assert snap.pressure_pct == pytest.approx(50.0)
    assert snap.front_bar == 40.0 and snap.rear_bar == 20.0
    assert snap.trail_pct == pytest.approx(100.0)
    rig.tick(p=150.0)
    assert rig.an.snapshot().pressure_pct == 100.0  # clamped


def test_short_tap_is_discarded_and_short_gap_merges():
    rig = Rig()
    rig.tick(p=0.0, n=5)
    rig.tick(p=20.0, n=10)  # 0.2 s < event_min_s
    rig.tick(p=0.0, n=20)
    assert rig.events == []
    # a 0.1 s dip below threshold does not split an event
    rig.tick(p=30.0, n=30)
    rig.tick(p=0.0, n=5)
    rig.tick(p=30.0, n=30)
    rig.tick(p=0.0, n=20)
    assert len(rig.events) == 1
    assert rig.events[0].duration_s == pytest.approx(1.3, abs=0.05)


def test_estimate_mode_from_decel_and_brake_switch():
    rig = Rig(brake_cfg=BrakeConfig(max_pressure_bar=100.0, estimate_bar_per_g=60.0))
    rig.tick(sw=False, lon=0.0, n=10)
    snap = rig.an.snapshot()
    assert snap.source == "estimate" and not snap.active and snap.pressure_bar == 0.0
    rig.tick(sw=True, lon=-0.8, kph=120.0, n=40)
    snap = rig.an.snapshot()
    assert snap.active
    assert snap.pressure_bar == pytest.approx(48.0)  # 60 bar/g * 0.8 g
    assert snap.pressure_pct == pytest.approx(48.0)
    rig.tick(sw=False, lon=0.0, kph=90.0, n=20)
    [ev] = rig.events
    assert ev.peak_bar == pytest.approx(48.0)
    assert ev.avg_decel_g == pytest.approx(0.8)
    assert ev.exit_kph == pytest.approx(90.0)
    # switch on but no lon_g at all: active, pressure unknown -> 0
    rig.tick(sw=True, lon=None, n=5)
    snap = rig.an.snapshot()
    assert snap.active and snap.pressure_bar == 0.0


def test_no_signals_means_source_none():
    rig = Rig()
    rig.tick(lon=-0.9, n=10)
    snap = rig.an.snapshot()
    assert snap.source == "none" and not snap.active and snap.pressure_bar == 0.0
    assert rig.events == []


def test_peak_reset_and_full_reset():
    rig = Rig()
    trail_braking_stop(rig, peak=80.0)
    assert rig.an.snapshot().peak_bar == pytest.approx(80.0)
    rig.an.reset_peak()
    assert rig.an.snapshot().peak_bar == 0.0
    assert rig.an.snapshot().count == 1  # events kept
    rig.tick(p=30.0, n=3)
    assert rig.an.snapshot().peak_bar == pytest.approx(30.0)
    rig.an.reset()
    snap = rig.an.snapshot()
    assert snap.peak_bar == 0.0 and snap.count == 0 and snap.events == [] and snap.trace == []


def test_trace_is_bounded_and_25hz():
    rig = Rig()
    for i in range(1000):  # 20 s
        rig.tick(p=float(i % 50))
    trace = rig.an.snapshot().trace
    assert len(trace) <= int(8.0 / 0.04) + 4
    span = trace[-1][0] - trace[0][0]
    assert 7.5 <= span <= 8.0 + 1e-6
    gaps = [b[0] - a[0] for a, b in zip(trace, trace[1:], strict=False)]
    assert min(gaps) >= 0.04 - 1e-6
    assert all(len(p) == 2 for p in trace)


def test_events_capped_most_recent_last_and_score_uses_last_ten():
    rig = Rig(BrakeMetricsConfig(max_events=5))
    for k in range(12):
        rig.tick(p=0.0, n=15)
        rig.tick(p=20.0 + k, n=25)
    rig.tick(p=0.0, n=15)
    snap = rig.an.snapshot()
    assert snap.count == 12
    assert len(snap.events) == 5
    assert [round(e.peak_bar) for e in snap.events] == [27, 28, 29, 30, 31]
    last10 = rig.events[-10:]
    expected = sum(0.6 * e.smoothness + 0.4 * min(100, e.peak_bar / 120 * 130) for e in last10) / 10
    assert snap.score == pytest.approx(expected, abs=0.1)


def test_smoothness_prefers_steady_pressure():
    steady, jerky = Rig(), Rig()
    steady.tick(p=0.0, n=5)
    steady.tick(p=50.0, n=50)
    steady.tick(p=0.0, n=20)
    jerky.tick(p=0.0, n=5)
    for i in range(50):
        jerky.tick(p=50.0 if i % 4 < 2 else 20.0)
    jerky.tick(p=0.0, n=20)
    assert steady.events[0].smoothness > jerky.events[0].smoothness
