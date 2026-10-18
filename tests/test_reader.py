"""obddash.bus.reader: CanReader against a fake ECU on python-can's in-process virtual bus."""

from __future__ import annotations

import threading
import time
import uuid

import can
import pytest

from obddash.bus import obd
from obddash.bus.reader import ECU_ALIVE_S, CanReader, RateMeter
from obddash.config import BusConfig, PidRequest, VehicleConfig
from obddash.model import RawState, Sig

VIN = "WAUZZZ8V9KA123456"
REQ_ID, RESP_ID, PHYS_ID = 0x7DF, 0x7E8, 0x7E0

# raw bytes the fake ECU answers with
ECU_VALUES = {
    0x0C: bytes([0x1A, 0xF8]),  # 1726 rpm
    0x0D: bytes([0x50]),  # 80 km/h
    0x11: bytes([0x80]),  # 50.2 % throttle
    0x05: bytes([0x82]),  # 90 degC coolant
    0x04: bytes([0x40]),  # load
}


class FakeEcu(threading.Thread):
    """Answers mode 01 / mode 09 requests on a virtual channel like a real engine ECU:
    single frames when the answer fits, ISO-TP first frame + flow-control wait + consecutive
    frames otherwise."""

    def __init__(
        self,
        channel: str,
        *,
        supported=(0x0C, 0x0D, 0x11, 0x05, 0x04),
        answer_discovery: bool = True,
        answer_multi: bool = True,
        vin: str | None = VIN,
    ):
        super().__init__(name="fake-ecu", daemon=True)
        self.bus = can.Bus(interface="virtual", channel=channel)
        self.supported = set(supported)
        self.answer_discovery = answer_discovery
        self.answer_multi = answer_multi
        self.vin = vin
        self.requests: list[tuple[int, list[int]]] = []
        self.fc_received = 0
        self._quit = threading.Event()
        self._deferred: list[can.Message] = []

    # -- lifecycle --
    def stop(self):
        self._quit.set()
        self.join(timeout=2.0)
        self.bus.shutdown()

    def broadcast(self, arbitration_id: int, data: bytes):
        self.bus.send(can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False))

    # -- loop --
    def run(self):
        while not self._quit.is_set():
            if self._deferred:
                msg = self._deferred.pop(0)
            else:
                msg = self.bus.recv(0.02)
            if msg is None:
                continue
            if msg.arbitration_id in (REQ_ID, PHYS_ID) and (msg.data[0] >> 4) == 0:
                self._answer(msg)

    def _answer(self, msg: can.Message):
        n = msg.data[0] & 0x0F
        mode = msg.data[1]
        pids = list(msg.data[2 : 1 + n])
        self.requests.append((mode, pids))
        if mode == 0x01:
            if len(pids) > 1 and not self.answer_multi:
                return
            payload = bytearray([0x41])
            for pid in pids:
                if pid == 0x00:
                    if self.answer_discovery:
                        payload += b"\x00" + obd.encode_supported(0x00, self.supported)
                elif pid in self.supported:
                    payload += bytes([pid]) + ECU_VALUES[pid]
            if len(payload) > 1:
                self._send_payload(bytes(payload))
        elif mode == 0x09 and pids == [0x02] and self.vin:
            self._send_payload(bytes([0x49, 0x02, 0x01]) + self.vin.encode())

    def _send_payload(self, payload: bytes):
        frames = obd.split_multi_frame(payload)
        self.bus.send(can.Message(arbitration_id=RESP_ID, data=frames[0], is_extended_id=False))
        if len(frames) == 1:
            return
        # wait for the tester's flow control (30 xx xx) on our physical id
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            m = self.bus.recv(0.02)
            if m is None:
                continue
            if m.arbitration_id == PHYS_ID and (m.data[0] >> 4) == 0x3:
                self.fc_received += 1
                break
            self._deferred.append(m)
        else:
            return  # no flow control: a real ECU gives up too
        for f in frames[1:]:
            self.bus.send(can.Message(arbitration_id=RESP_ID, data=f, is_extended_id=False))


def wait_until(pred, timeout=3.0, step=0.02) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return pred()


@pytest.fixture
def channel():
    return f"obddash-test-{uuid.uuid4().hex}"


@pytest.fixture
def ecu(channel):
    e = FakeEcu(channel)
    e.start()
    yield e
    e.stop()


def make_reader(channel, raw=None, *, bus_cfg=None, vehicle=None):
    raw = raw or RawState()
    bus_cfg = bus_cfg or BusConfig(interface="virtual", channel=channel)
    vehicle = vehicle or VehicleConfig()
    bus = can.Bus(interface="virtual", channel=channel)
    reader = CanReader(bus_cfg, vehicle, raw, bus=bus)
    return reader, raw, bus


def obddash_threads():
    return [t for t in threading.enumerate() if t.name.startswith("obddash-")]


# --------------------------------------------------------------------------------------
def test_rate_meter():
    m = RateMeter(now=0.0)
    assert m.rate(0.1) == 0.0
    for i in range(25):
        m.tick(i * 0.02)  # 25 ticks in the first half second
    assert m.rate(0.5) == pytest.approx(50.0, rel=0.1)  # provisional estimate
    for i in range(25, 50):
        m.tick(i * 0.02)  # 50 ticks in the first second
    assert m.rate(1.0) == pytest.approx(50.0, rel=0.05)
    assert m.rate(1.5) == pytest.approx(50.0, rel=0.05)  # held during the next window
    assert m.rate(3.0) == 0.0  # source stopped
    # an idle start does not count as a window
    m2 = RateMeter(now=0.0)
    m2.tick(10.0)
    assert m2.rate(10.5) == pytest.approx(2.0)


def test_reader_polls_ecu_and_fills_raw(channel, ecu):
    reader, raw, _bus = make_reader(channel)
    reader.start()
    try:
        assert wait_until(
            lambda: raw.has(Sig.RPM) and raw.has(Sig.SPEED_KPH) and raw.has(Sig.THROTTLE_PCT), 2.0
        )
        assert raw.get(Sig.RPM) == 1726.0
        assert raw.get(Sig.SPEED_KPH) == 80.0
        assert raw.get(Sig.THROTTLE_PCT) == pytest.approx(128 / 2.55)
        assert wait_until(lambda: raw.has(Sig.COOLANT_C), 2.0)  # slow PID gets its turn too
        st = reader.stats
        assert st.bus_ok and st.ecu_ok and st.discovered
        assert st.supported_pids == ecu.supported
        assert wait_until(lambda: reader.stats.pid_hz > 0, 2.0)
        assert reader.stats.frame_hz > 0
        assert reader.stats.pid_responses > 5
        assert wait_until(lambda: reader.stats.vin == VIN, 1.0)  # multi-frame + flow control
        assert ecu.fc_received >= 1
        # only configured PIDs the ECU supports are polled
        polled = {pid for mode, pids in ecu.requests if mode == 1 for pid in pids}
        assert 0x5C not in polled and 0x0B not in polled
        assert {0x0C, 0x0D, 0x11, 0x05, 0x04} <= polled
        assert set(reader.stats.active_pids) == {0x0C, 0x0D, 0x11, 0x05, 0x04}
        # fast PIDs are requested far more often than slow ones
        counts = {}
        for _mode, pids in ecu.requests:
            for p in pids:
                counts[p] = counts.get(p, 0) + 1
        assert counts[0x0C] > counts[0x05]
    finally:
        reader.stop()


def test_passive_signals_decode_into_raw(channel, ecu):
    vehicle = VehicleConfig(
        signals=[
            {
                "name": Sig.BRAKE_PRESSURE_BAR,
                "id": 0x200,
                "byte": 0,
                "length": 16,
                "scale": 0.01,
                "max": 250,
            },
            {"name": Sig.BRAKE_SWITCH, "id": 0x200, "start_bit": 48, "length": 1},
            {
                "name": Sig.STEERING_DEG,
                "id": 0x201,
                "byte": 0,
                "length": 16,
                "signed": True,
                "scale": 0.1,
            },
        ]
    )
    cfg = BusConfig(interface="virtual", channel=channel, obd_enabled=False, log_unknown_ids=True)
    reader, raw, _bus = make_reader(channel, bus_cfg=cfg, vehicle=vehicle)
    reader.start()
    try:
        time.sleep(0.1)
        for _ in range(5):
            ecu.broadcast(0x200, bytes.fromhex("04D2000000000100"))
            ecu.broadcast(0x201, bytes.fromhex("FF88000000000000"))
            ecu.broadcast(0x333, bytes.fromhex("0102030405060708"))  # unknown id
            time.sleep(0.02)
        assert wait_until(
            lambda: raw.has(Sig.BRAKE_PRESSURE_BAR) and raw.has(Sig.STEERING_DEG), 2.0
        )
        assert raw.get(Sig.BRAKE_PRESSURE_BAR) == pytest.approx(12.34)
        assert raw.get(Sig.BRAKE_SWITCH) == 1.0
        assert raw.get(Sig.STEERING_DEG) == pytest.approx(-12.0)
        st = reader.stats
        assert st.frames >= 15
        assert st.unknown_ids.get(0x333) == 5
        assert 0x200 not in st.unknown_ids
        assert not st.ecu_ok and st.pid_responses == 0
        assert ecu.requests == []  # obd disabled: nothing was asked
        assert not raw.has(Sig.RPM)
    finally:
        reader.stop()


def test_stop_is_clean_and_idempotent(channel, ecu):
    reader, raw, bus = make_reader(channel)
    reader.start()
    assert wait_until(lambda: raw.has(Sig.RPM), 2.0)
    assert obddash_threads()
    t0 = time.monotonic()
    reader.stop()
    assert time.monotonic() - t0 < 2.0
    assert wait_until(lambda: not obddash_threads(), 1.0)
    assert not reader.running
    assert getattr(bus, "_is_shutdown", True)
    assert not reader.stats.bus_ok
    reader.stop()  # second stop is a no-op
    # no requests arrive after stop
    n = len(ecu.requests)
    time.sleep(0.3)
    assert len(ecu.requests) == n


def test_missing_interface_never_raises(channel):
    cfg = BusConfig(interface="socketcan", channel="obddash_no_such_if0")
    reader = CanReader(cfg, VehicleConfig(), RawState())
    reader.start()  # must not raise even though the interface cannot be opened
    try:
        assert wait_until(lambda: reader.stats.last_error is not None, 2.0)
        assert not reader.stats.bus_ok
        assert "obddash_no_such_if0" in reader.stats.last_error
        assert reader.running  # keeps retrying in the background
    finally:
        t0 = time.monotonic()
        reader.stop()
        assert time.monotonic() - t0 < 2.0


def test_multi_pid_requests_are_packed(channel, ecu):
    cfg = BusConfig(interface="virtual", channel=channel, obd_multi_pid=True)
    reader, raw, _bus = make_reader(channel, bus_cfg=cfg)
    reader.start()
    try:
        assert wait_until(
            lambda: raw.has(Sig.RPM) and raw.has(Sig.SPEED_KPH) and raw.has(Sig.THROTTLE_PCT), 2.0
        )
        assert wait_until(lambda: any(len(p) == 3 for m, p in ecu.requests if m == 1), 1.0)
        assert reader.stats.multi_pid
        assert ecu.fc_received >= 1  # the 8-byte multi-PID answer is a multi-frame response
        assert raw.get(Sig.RPM) == 1726.0 and raw.get(Sig.SPEED_KPH) == 80.0
        assert reader.stats.pid_responses > 3
    finally:
        reader.stop()


def test_multi_pid_falls_back_to_single(channel):
    ecu = FakeEcu(channel, answer_multi=False)
    ecu.start()
    cfg = BusConfig(interface="virtual", channel=channel, obd_multi_pid=True, obd_timeout_s=0.15)
    reader, raw, _bus = make_reader(channel, bus_cfg=cfg)
    reader.start()
    try:
        assert wait_until(lambda: not reader.stats.multi_pid, 3.0)
        assert wait_until(lambda: raw.has(Sig.RPM) and raw.has(Sig.SPEED_KPH), 2.0)
        multi = [p for m, p in ecu.requests if m == 1 and len(p) > 1]
        assert 1 <= len(multi) <= 3  # gave up after two unanswered packed requests
        assert reader.stats.timeouts >= 2
    finally:
        reader.stop()
        ecu.stop()


def test_no_discovery_answer_polls_configured_list(channel):
    ecu = FakeEcu(channel, answer_discovery=False, supported=(0x0C, 0x0D))
    ecu.start()
    vehicle = VehicleConfig(
        pids=[
            PidRequest(pid=0x0C, rate="fast"),
            PidRequest(pid=0x0D, rate="fast"),
            PidRequest(pid=0x5C, rate="slow"),
        ]
    )
    cfg = BusConfig(interface="virtual", channel=channel, obd_timeout_s=0.1)
    reader, raw, _bus = make_reader(channel, bus_cfg=cfg, vehicle=vehicle)
    reader.start()
    try:
        # discovery: 2 tries x 0.3 s, then the configured PIDs are polled anyway
        assert wait_until(lambda: raw.has(Sig.RPM) and raw.has(Sig.SPEED_KPH), 3.0)
        st = reader.stats
        assert not st.discovered and st.supported_pids == set()
        assert st.ecu_ok
        assert set(st.active_pids) == {0x0C, 0x0D, 0x5C}
        discovery = [p for m, p in ecu.requests if m == 1 and p == [0x00]]
        assert len(discovery) == 2
        # the unsupported slow PID was tried, timed out, and is now backed off
        assert wait_until(lambda: any(p == [0x5C] for m, p in ecu.requests), 2.0)
        assert reader.stats.timeouts >= 1
    finally:
        reader.stop()
        ecu.stop()


def test_ecu_ok_decays_when_ecu_goes_silent(channel):
    ecu = FakeEcu(channel)
    ecu.start()
    cfg = BusConfig(interface="virtual", channel=channel, obd_timeout_s=0.05)
    vehicle = VehicleConfig(pids=[PidRequest(pid=0x0C, rate="fast")])
    reader, raw, _bus = make_reader(channel, bus_cfg=cfg, vehicle=vehicle)
    reader.start()
    try:
        assert wait_until(lambda: raw.has(Sig.RPM), 2.0)
        assert reader.stats.ecu_ok
        ecu.stop()  # ignition off
        assert wait_until(lambda: not reader.stats.ecu_ok, ECU_ALIVE_S + 1.0)
        assert reader.stats.bus_ok  # the interface itself is still fine
    finally:
        reader.stop()


def test_pid_without_decoder_is_not_polled(channel, ecu):
    vehicle = VehicleConfig(
        pids=[PidRequest(pid=0x0C, rate="fast"), PidRequest(pid=0xE5, rate="fast")]
    )
    reader, raw, _bus = make_reader(channel, vehicle=vehicle)
    reader.start()
    try:
        assert wait_until(lambda: raw.has(Sig.RPM), 2.0)
        assert 0xE5 not in {p for m, ps in ecu.requests for p in ps}
    finally:
        reader.stop()
