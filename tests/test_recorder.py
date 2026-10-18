"""Tests for the JSONL recorder / replayer (obddash.server.recorder)."""

from __future__ import annotations

import json
import time
from datetime import datetime

import pytest

from obddash import __version__
from obddash.model import RawState, Sig
from obddash.server.recorder import (
    Recorder,
    Replayer,
    make_recording_path,
    parse_line,
    read_meta,
    read_recording,
)


class FakeClock:
    def __init__(self, t: float = 100.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def _lines(path):
    return [json.loads(x) for x in path.read_text().splitlines()]


def _record(path, steps, *, keepalive_s=1.0, meta=None):
    """Drive a Recorder with ``[(t, {sig: value}), ...]`` on a fake clock."""
    clock = FakeClock()
    raw = RawState(clock=clock)
    with Recorder(path, meta=meta, keepalive_s=keepalive_s) as rec:
        for t, values in steps:
            clock.t = t
            if values:
                raw.update(values)
            rec.sample(raw, t)
    return rec


# --------------------------------------------------------------------------------------
# Recorder
# --------------------------------------------------------------------------------------
def test_round_trip_and_header(tmp_path):
    path = tmp_path / "rec.jsonl"
    _record(
        path,
        [
            (10.0, {Sig.RPM: 850.0, Sig.SPEED_KPH: 0.0, Sig.COOLANT_C: 88.0}),
            (10.02, {Sig.RPM: 900.5}),
            (10.04, {Sig.RPM: 950.25, Sig.SPEED_KPH: 3.0}),
        ],
        meta={"vehicle": "Test car", "source": "sim"},
    )
    lines = _lines(path)
    meta = lines[0]["meta"]
    assert meta["vehicle"] == "Test car"
    assert meta["version"] == __version__
    assert meta["format"] == 1
    assert meta["started"] == pytest.approx(time.time(), abs=60)
    assert read_meta(path) == meta

    data = read_recording(path)
    assert data == [
        (0.0, {Sig.RPM: 850.0, Sig.SPEED_KPH: 0.0, Sig.COOLANT_C: 88.0}),
        (0.02, {Sig.RPM: 900.5}),
        (0.04, {Sig.RPM: 950.25, Sig.SPEED_KPH: 3.0}),
    ]
    # integers are written without a fraction to keep files small
    assert '"rpm":850,' in path.read_text()


def test_delta_encoding_skips_unchanged_values_and_ticks(tmp_path):
    path = tmp_path / "rec.jsonl"
    rec = _record(
        path,
        [
            (0.00, {Sig.RPM: 1000.0, Sig.COOLANT_C: 90.0}),
            (0.02, {Sig.RPM: 1000.0, Sig.COOLANT_C: 90.0}),  # refreshed, unchanged
            (0.04, {}),  # nothing new at all
            (0.06, {Sig.RPM: 1010.0, Sig.COOLANT_C: 90.0}),
        ],
    )
    data = read_recording(path)
    assert data == [(0.0, {Sig.RPM: 1000.0, Sig.COOLANT_C: 90.0}), (0.06, {Sig.RPM: 1010.0})]
    assert rec.lines == 2


def test_keepalive_rewrites_refreshed_values_but_not_stale_ones(tmp_path):
    """A constant but still-arriving value is re-written once per keepalive so it
    stays fresh on replay; a value that stopped arriving is not."""
    path = tmp_path / "rec.jsonl"
    steps = [(0.0, {Sig.COOLANT_C: 92.0, Sig.OIL_TEMP_C: 100.0})]
    t = 0.0
    for _ in range(60):  # 1.2 s at 50 Hz; coolant keeps arriving, oil temp went silent
        t = round(t + 0.02, 2)
        steps.append((t, {Sig.COOLANT_C: 92.0}))
    _record(path, steps, keepalive_s=0.5)
    data = read_recording(path)
    coolant_lines = [t for t, v in data if Sig.COOLANT_C in v]
    oil_lines = [t for t, v in data if Sig.OIL_TEMP_C in v]
    assert oil_lines == [0.0]
    assert coolant_lines == [0.0, 0.5, 1.0]


def test_time_starts_at_the_first_data_line(tmp_path):
    """Ticks before any signal arrived do not shift the time base."""
    path = tmp_path / "rec.jsonl"
    _record(path, [(5.0, {}), (5.02, {}), (5.04, {Sig.RPM: 800.0}), (5.1, {Sig.RPM: 900.0})])
    assert read_recording(path) == [(0.0, {Sig.RPM: 800.0}), (0.06, {Sig.RPM: 900.0})]


def test_non_finite_values_are_not_recorded(tmp_path):
    path = tmp_path / "rec.jsonl"
    _record(path, [(0.0, {Sig.RPM: float("nan"), Sig.SPEED_KPH: 5.0})])
    assert read_recording(path) == [(0.0, {Sig.SPEED_KPH: 5.0})]


def test_sample_after_close_is_a_noop(tmp_path):
    path = tmp_path / "rec.jsonl"
    raw = RawState()
    rec = Recorder(path)
    rec.close()
    raw.set(Sig.RPM, 1000.0)
    assert rec.sample(raw, 1.0) is False
    assert rec.closed
    rec.close()  # idempotent


def test_write_failure_disables_recorder_without_raising(tmp_path):
    path = tmp_path / "rec.jsonl"
    raw = RawState()
    rec = Recorder(path, flush_s=0.0)

    class Broken:
        def write(self, _s):
            raise OSError(28, "No space left on device")

        def flush(self):
            raise OSError(28, "No space left on device")

        def close(self):
            pass

    rec._fh = Broken()
    raw.set(Sig.RPM, 1000.0)
    assert rec.sample(raw, 0.0) is False
    assert rec.closed
    assert rec.sample(raw, 0.1) is False


def test_recorder_flushes_periodically(tmp_path):
    path = tmp_path / "rec.jsonl"
    clock = FakeClock()
    raw = RawState(clock=clock)
    rec = Recorder(path, flush_s=1.0)
    raw.set(Sig.RPM, 1000.0)
    rec.sample(raw, 0.0)
    before = path.read_text()
    assert "1000" not in before  # still buffered
    raw.set(Sig.RPM, 1100.0)
    rec.sample(raw, 1.0)
    assert "1100" in path.read_text()
    rec.close()


def test_make_recording_path_never_overwrites(tmp_path):
    now = datetime(2025, 10, 9, 8, 53, 20)
    p1 = make_recording_path(tmp_path, now)
    assert p1.name == "obddash-20251009-085320.jsonl"
    p1.write_text("")
    p2 = make_recording_path(tmp_path, now)
    assert p2.name == "obddash-20251009-085320-1.jsonl"
    with pytest.raises(FileExistsError):
        Recorder(p1)


def test_parse_line_rejects_garbage():
    assert parse_line('{"meta": {}}') is None
    assert parse_line("") is None
    assert parse_line('{"t": 1.0, "v": {"rpm": 9') is None  # truncated
    assert parse_line('{"t": "x"}') is None
    assert parse_line('{"t": 1.5, "v": {"rpm": 900, "bad": "str", "ok": true}}') == (
        1.5,
        {"rpm": 900.0, "ok": 1.0},
    )


# --------------------------------------------------------------------------------------
# Replayer
# --------------------------------------------------------------------------------------
def _ramp_recording(path, seconds=1.0, hz=50):
    steps = []
    n = int(seconds * hz)
    for i in range(n + 1):
        t = round(i / hz, 3)
        steps.append((t, {Sig.RPM: 1000.0 + 10 * i, Sig.SPEED_KPH: float(i // 5)}))
    steps[0][1][Sig.COOLANT_C] = 85.0
    _record(path, steps, meta={"vehicle": "Ramp"})
    return steps


def test_replayer_restores_final_values(tmp_path):
    path = tmp_path / "rec.jsonl"
    steps = _ramp_recording(path)
    raw = RawState()
    rp = Replayer(path, raw, speed=50.0)
    assert rp.duration == pytest.approx(1.0)
    assert rp.meta["vehicle"] == "Ramp"
    t0 = time.monotonic()
    rp.start()
    assert rp.wait(5.0)
    elapsed = time.monotonic() - t0
    assert elapsed < 1.0  # 1 s of data at x50 is ~20 ms
    assert rp.finished and not rp.running
    assert rp.position == pytest.approx(1.0)
    assert rp.passes == 1
    assert rp.lines == len(steps)
    assert raw.get(Sig.RPM) == steps[-1][1][Sig.RPM]
    assert raw.get(Sig.SPEED_KPH) == steps[-1][1][Sig.SPEED_KPH]
    assert raw.get(Sig.COOLANT_C) == 85.0
    # values are fresh: stamped with the replay time, not the recording time
    assert raw.age(Sig.RPM) < 1.0


def test_replayer_keeps_original_timing(tmp_path):
    path = tmp_path / "rec.jsonl"
    _ramp_recording(path, seconds=0.3)
    rp = Replayer(path, RawState(), speed=1.0)
    t0 = time.monotonic()
    rp.start()
    assert rp.wait(5.0)
    assert time.monotonic() - t0 == pytest.approx(0.3, abs=0.15)


def test_replayer_loop_and_stop(tmp_path):
    path = tmp_path / "rec.jsonl"
    _ramp_recording(path, seconds=0.2)
    rp = Replayer(path, RawState(), speed=20.0, loop=True)
    rp.start()
    deadline = time.monotonic() + 5.0
    while rp.passes < 3 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert rp.passes >= 3
    assert rp.running
    rp.stop()
    assert not rp.running


def test_replayer_tolerates_truncated_last_line(tmp_path):
    path = tmp_path / "rec.jsonl"
    _ramp_recording(path, seconds=0.5)
    with open(path, "a", encoding="utf-8") as fh:
        fh.write('{"t": 0.52, "v": {"rpm": 12')  # power cut mid-write
    assert read_recording(path)[-1][0] == pytest.approx(0.5)
    raw = RawState()
    rp = Replayer(path, raw, speed=50.0)
    assert rp.duration == pytest.approx(0.5)
    rp.start()
    assert rp.wait(5.0)
    assert raw.get(Sig.RPM) == 1000.0 + 10 * 25


def test_replayer_rejects_bad_arguments(tmp_path):
    with pytest.raises(FileNotFoundError):
        Replayer(tmp_path / "missing.jsonl", RawState())
    path = tmp_path / "rec.jsonl"
    _ramp_recording(path, seconds=0.1)
    with pytest.raises(ValueError):
        Replayer(path, RawState(), speed=0)


def test_replayer_empty_recording_does_not_spin(tmp_path):
    path = tmp_path / "rec.jsonl"
    Recorder(path).close()  # header only
    rp = Replayer(path, RawState(), loop=True)
    assert rp.duration == 0.0
    rp.start()
    assert rp.wait(2.0)
