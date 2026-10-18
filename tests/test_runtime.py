"""Tests for obddash.server.runtime: tick / broadcast loops, fan-out, sources.

The metrics engine is replaced by a FakeEngine everywhere, so these tests exercise
only the runtime's own wiring.  The "can" / "sim" / "replay" source tests use the
real bus reader, simulator and replayer on per-test virtual CAN channels.
"""

from __future__ import annotations

import asyncio
import json
import re
import threading
import time
import uuid
from types import SimpleNamespace

import can
import pytest

from obddash.config import CONFIG_DIR, AppConfig, load_vehicle
from obddash.model import Event, RawState, Sig, Telemetry
from obddash.server.recorder import Recorder, read_meta, read_recording
from obddash.server.runtime import CLIENT_QUEUE_SIZE, Runtime, _next_deadline


class FakeEngine:
    """Stands in for MetricsEngine: counts ticks, echoes rpm, records commands."""

    def __init__(self, fail_every: int = 0):
        self.fail_every = fail_every
        self.calls = 0
        self.ticks = 0
        self.conn = None
        self.commands: list[tuple[str, dict]] = []
        self.events: list[Event] = []

    def set_connection(self, conn):
        self.conn = conn

    def tick(self, raw, t):
        self.calls += 1
        if self.fail_every and self.calls % self.fail_every == 0:
            raise RuntimeError("boom")
        self.ticks += 1
        tel = Telemetry(t=float(self.ticks), ts=time.time(), conn=self.conn)
        tel.veh.rpm = raw.get(Sig.RPM, 0.0) or 0.0
        return tel

    def command(self, cmd, args=None):
        if cmd == "explode":
            raise ValueError("bad command")
        self.commands.append((cmd, args))
        return cmd != "nope"

    def drain_events(self):
        out, self.events = self.events, []
        return out


def _cfg(tick_hz: float = 100.0, ws_hz: float = 50.0, vehicle=None) -> AppConfig:
    cfg = AppConfig()
    cfg.metrics.tick_hz = tick_hz
    cfg.server.ws_rate_hz = ws_hz
    if vehicle is not None:
        cfg.vehicle = vehicle
    return cfg


@pytest.fixture(scope="module")
def sim_vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


@pytest.fixture
def channel():
    return f"obddash-test-rt-{uuid.uuid4().hex}"


def _wait(cond, timeout=5.0, step=0.01) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(step)
    return cond()


async def _await(cond, timeout=5.0) -> None:
    deadline = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < deadline, "condition not reached in time"
        await asyncio.sleep(0.005)


# --------------------------------------------------------------------------------------
# loops in a real event loop
# --------------------------------------------------------------------------------------
async def test_none_source_ticks_and_broadcasts_telemetry():
    engine = FakeEngine()
    rt = Runtime(_cfg(), source="none", engine=engine)
    q = rt.subscribe()
    task = asyncio.create_task(rt.run())
    await _await(lambda: engine.ticks >= 10 and not q.empty())
    assert rt.health()["ok"] is True
    rt.stop()
    assert await asyncio.wait_for(task, 2.0) is None  # stop() makes run() return normally
    assert not rt.running
    assert rt.health()["ok"] is False

    msg = json.loads(q.get_nowait())
    assert msg["type"] == "telemetry"
    data = msg["data"]
    assert data["conn"] == {
        "bus": False,
        "ecu": False,
        "imu": False,
        "source": "none",
        "pid_hz": 0.0,
        "frame_hz": 0.0,
    }
    assert set(data) >= {"t", "veh", "g", "brake", "shift", "dyno", "handling", "session"}
    assert rt.stats.ticks >= 10
    assert engine.conn.source == "none"


async def test_slow_consumer_frames_are_dropped_not_blocking():
    engine = FakeEngine()
    rt = Runtime(_cfg(tick_hz=100, ws_hz=100), source="none", engine=engine)
    slow = rt.subscribe()  # never read
    fast = rt.subscribe()
    received: list[dict] = []

    async def consume():
        while True:
            received.append(json.loads(await fast.get()))

    consumer = asyncio.create_task(consume())
    task = asyncio.create_task(rt.run())
    await _await(lambda: engine.ticks >= 30 and len(received) >= 10)
    rt.stop()
    await asyncio.wait_for(task, 2.0)
    await asyncio.sleep(0.02)  # let the fast consumer drain
    consumer.cancel()
    await asyncio.gather(consumer, return_exceptions=True)

    assert slow.qsize() == CLIENT_QUEUE_SIZE
    assert rt.stats.dropped >= len(received) - CLIENT_QUEUE_SIZE - 1
    # the slow client holds the NEWEST frames: the oldest ones were discarded
    held = [json.loads(slow.get_nowait())["data"]["t"] for _ in range(CLIENT_QUEUE_SIZE)]
    assert held[-1] == received[-1]["data"]["t"]
    assert held[0] < held[1]
    assert held[0] > received[0]["data"]["t"]


async def test_engine_errors_never_stop_the_tick_loop():
    engine = FakeEngine(fail_every=2)
    rt = Runtime(_cfg(), source="none", engine=engine)
    task = asyncio.create_task(rt.run())
    await _await(lambda: engine.calls >= 12)
    rt.stop()
    await asyncio.wait_for(task, 2.0)
    assert rt.stats.tick_errors >= 6
    assert rt.stats.ticks >= 12
    assert rt.latest.t == float(engine.ticks)  # last good snapshot is kept


async def test_tick_rate_follows_config():
    engine = FakeEngine()
    rt = Runtime(_cfg(tick_hz=100), source="none", engine=engine)
    task = asyncio.create_task(rt.run())
    await asyncio.sleep(0.5)
    rt.stop()
    await asyncio.wait_for(task, 2.0)
    assert 35 <= engine.ticks <= 56  # ~50 expected; loose for loaded CI machines


async def test_stop_from_another_thread_ends_run():
    rt = Runtime(_cfg(), source="none", engine=FakeEngine())
    task = asyncio.create_task(rt.run())
    await _await(lambda: rt.running and rt.stats.ticks > 2)
    await asyncio.get_running_loop().run_in_executor(None, rt.stop)
    assert await asyncio.wait_for(task, 2.0) is None


async def test_cancelling_the_run_task_propagates():
    rt = Runtime(_cfg(), source="none", engine=FakeEngine())
    task = asyncio.create_task(rt.run())
    await _await(lambda: rt.stats.ticks > 2)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert not rt.running
    rt.stop()


# --------------------------------------------------------------------------------------
# deterministic pieces (no running loop needed)
# --------------------------------------------------------------------------------------
def test_next_deadline_is_drift_free_and_skips_when_far_behind():
    assert _next_deadline(0.0, 0.02, 0.005) == (0.02, 0)  # on time
    assert _next_deadline(0.0, 0.02, 0.03) == (0.02, 0)  # late < 1 period: run now
    deadline, skipped = _next_deadline(0.0, 0.02, 0.105)  # 4 slots behind
    assert skipped == 4
    assert deadline == pytest.approx(0.10)
    # many on-time cycles land exactly on the grid (no accumulated jitter)
    d = 0.0
    for i in range(1000):
        d, _ = _next_deadline(d, 0.02, d + 0.001 * (i % 7))
    assert d == pytest.approx(20.0)


def test_publish_discards_oldest_when_full():
    rt = Runtime(_cfg(), source="none", engine=FakeEngine())
    q = rt.subscribe()
    for i in range(5):
        rt.publish(str(i))
    assert [q.get_nowait() for _ in range(q.qsize())] == ["3", "4"]
    assert rt.stats.dropped == 3
    rt.unsubscribe(q)
    rt.publish("x")
    assert q.empty()
    assert rt.client_count == 0


def test_events_go_out_as_separate_messages_before_telemetry():
    engine = FakeEngine()
    rt = Runtime(_cfg(), source="none", engine=engine)
    q = rt.subscribe()
    engine.events = [
        Event("shift.scored", {"score": 86.0, "bad": float("nan"), "gears": (4, 3)}, t=1.5)
    ]
    rt.step()
    assert rt.broadcast_once() == 2
    ev = json.loads(q.get_nowait())
    tel = json.loads(q.get_nowait())
    assert ev == {
        "type": "event",
        "name": "shift.scored",
        "data": {"score": 86.0, "bad": None, "gears": [4, 3]},
        "t": 1.5,
    }
    assert tel["type"] == "telemetry"
    # nothing new: nothing sent
    assert rt.broadcast_once() == 0
    assert q.empty()


def test_events_are_drained_even_without_clients():
    engine = FakeEngine()
    rt = Runtime(_cfg(), source="none", engine=engine)
    engine.events = [Event("alert", {"msg": "COOLANT HIGH"})]
    assert rt.broadcast_once() == 0
    assert engine.events == []


def test_telemetry_is_serialised_once_per_snapshot():
    rt = Runtime(_cfg(), source="none", engine=FakeEngine())
    q1, q2 = rt.subscribe(), rt.subscribe()
    rt.step()
    a = rt.telemetry_json()
    assert rt.telemetry_json() is a
    assert rt.snapshot_dict() is rt.snapshot_dict()
    rt.broadcast_once()
    assert q1.get_nowait() is q2.get_nowait() is a
    rt.step()
    assert rt.telemetry_json() is not a
    assert json.loads(rt.telemetry_json())["data"]["t"] == 2.0


def test_step_feeds_engine_and_survives_recorder_failure():
    rt = Runtime(_cfg(), source="none", engine=FakeEngine())
    rt.raw.set(Sig.RPM, 3210.0)

    class BadRecorder:
        def sample(self, raw, t):
            raise OSError("disk gone")

        def close(self):
            pass

    rt.recorder = BadRecorder()
    tel = rt.step()
    assert tel.veh.rpm == 3210.0
    assert rt.recorder is None
    assert rt.snapshot_dict()["veh"]["rpm"] == 3210.0


def test_command_passthrough():
    engine = FakeEngine()
    rt = Runtime(_cfg(), source="none", engine=engine)
    assert rt.command("dyno.arm") is True
    assert rt.command("session.reset", {"keep": 1}) is True
    assert rt.command("nope") is False
    assert rt.command("explode") is False  # engine raised: logged, False
    assert rt.command("") is False
    assert engine.commands == [("dyno.arm", {}), ("session.reset", {"keep": 1}), ("nope", {})]


def test_connection_reflects_reader_imu_and_replayer():
    rt = Runtime(_cfg(), source="can", engine=FakeEngine())
    rt.reader = SimpleNamespace(
        stats=SimpleNamespace(bus_ok=True, ecu_ok=False, pid_hz=47.96, frame_hz=210.04)
    )
    rt.imu = SimpleNamespace(available=True)
    conn = rt.connection()
    assert (conn.bus, conn.ecu, conn.imu, conn.source) == (True, False, True, "can")
    assert (conn.pid_hz, conn.frame_hz) == (48.0, 210.0)
    rt.reader = None
    rt.imu = None
    rt.replayer = SimpleNamespace(running=True)
    conn = rt.connection()
    assert conn.bus and conn.ecu and not conn.imu


def test_bad_arguments():
    with pytest.raises(ValueError):
        Runtime(_cfg(), source="usb", engine=FakeEngine())
    with pytest.raises(ValueError):
        Runtime(_cfg(), source="replay", engine=FakeEngine())


def test_stop_is_idempotent_and_safe_without_start():
    rt = Runtime(_cfg(), source="none", engine=FakeEngine())
    rt.stop()
    rt.stop()
    rt2 = Runtime(_cfg(), source="none", engine=FakeEngine())
    rt2.start()
    rt2.start()
    rt2.stop()
    rt2.stop()


# --------------------------------------------------------------------------------------
# sources and recording
# --------------------------------------------------------------------------------------
def test_recording_writes_a_named_jsonl_file(tmp_path, sim_vehicle):
    cfg = _cfg(vehicle=sim_vehicle)
    cfg.recording.enabled = True
    cfg.recording.dir = str(tmp_path / "recordings")
    rt = Runtime(cfg, source="none", engine=FakeEngine())
    rt.start()
    assert rt.recorder is not None
    rt.raw.set(Sig.RPM, 1000.0)
    rt.step(10.0)
    rt.step(10.02)  # unchanged: no line
    rt.raw.set(Sig.RPM, 1100.0)
    rt.step(10.04)
    assert rt.health()["recording"] == str(rt.recorder.path)
    rt.stop()
    assert rt.recorder is None
    files = list((tmp_path / "recordings").glob("*.jsonl"))
    assert len(files) == 1
    assert re.fullmatch(r"obddash-\d{8}-\d{6}\.jsonl", files[0].name)
    assert read_meta(files[0])["vehicle"] == sim_vehicle.name
    assert read_recording(files[0]) == [(0.0, {Sig.RPM: 1000.0}), (0.04, {Sig.RPM: 1100.0})]


def test_can_source_with_injected_virtual_bus(channel, sim_vehicle):
    cfg = _cfg(vehicle=sim_vehicle)
    cfg.bus.obd_enabled = False  # nothing answers on this channel
    bus = can.Bus(interface="virtual", channel=channel)
    peer = can.Bus(interface="virtual", channel=channel)
    rt = Runtime(cfg, source="can", bus=bus, engine=FakeEngine())
    try:
        rt.start()
        assert _wait(lambda: rt.connection().bus)
        # a broadcast frame from "the car": 0x203 byte 0 = gear (int8)
        peer.send(
            can.Message(
                arbitration_id=0x203, data=bytes([3, 0, 0, 0, 0, 0, 0, 0]), is_extended_id=False
            )
        )
        assert _wait(lambda: rt.raw.get(Sig.GEAR) == 3.0)
        conn = rt.step().conn
        assert conn.bus and not conn.ecu and conn.source == "can"
    finally:
        rt.stop()
        peer.shutdown()
    assert not rt.reader.running


def test_sim_source_end_to_end(channel, sim_vehicle):
    rt = Runtime(
        _cfg(vehicle=sim_vehicle),
        source="sim",
        engine=FakeEngine(),
        sim_channel=channel,
        sim_script="lap",
        sim_seed=3,
    )
    try:
        rt.start()
        assert rt.sim is not None and rt.reader is not None
        # OBD-polled (rpm, speed) and broadcast (brake pressure, accel) signals arrive
        assert _wait(
            lambda: all(
                rt.raw.has(s)
                for s in (Sig.RPM, Sig.SPEED_KPH, Sig.BRAKE_PRESSURE_BAR, Sig.ACC_LON_G)
            ),
            timeout=8.0,
        )
        assert _wait(lambda: rt.connection().ecu, timeout=5.0)
        conn = rt.step().conn
        assert conn.bus and conn.ecu and conn.source == "sim" and not conn.imu
    finally:
        rt.stop()
    assert not rt.sim.running
    assert not rt.reader.running
    assert rt._sim_buses == []
    threads = [t.name for t in threading.enumerate()]
    assert "sim-physics" not in threads


def test_replay_source(tmp_path):
    path = tmp_path / "drive.jsonl"
    raw_values = [(0.0, 1000.0), (0.1, 2000.0), (0.2, 3000.0)]
    clock_t = [0.0]
    raw = RawState(clock=lambda: clock_t[0])
    with Recorder(path, meta={"vehicle": "x"}) as rec:
        for t, rpm in raw_values:
            clock_t[0] = t
            raw.set(Sig.RPM, rpm)
            rec.sample(raw, t)

    rt = Runtime(_cfg(), source="replay", replay_path=path, replay_speed=20.0, engine=FakeEngine())
    try:
        rt.start()
        assert rt.replayer.wait(5.0)
        assert rt.raw.get(Sig.RPM) == 3000.0
        assert rt.step().veh.rpm == 3000.0
        h = rt.health()
        assert h["replay"]["finished"] is True
        assert h["replay"]["duration_s"] == pytest.approx(0.2)
        assert rt.connection().bus is False  # replay over: link down
    finally:
        rt.stop()


def test_failed_start_cleans_up(tmp_path):
    rt = Runtime(
        _cfg(), source="replay", replay_path=tmp_path / "missing.jsonl", engine=FakeEngine()
    )
    with pytest.raises(FileNotFoundError):
        rt.start()
    rt.stop()
