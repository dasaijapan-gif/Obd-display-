"""Tests for the fake ECU (sim.ecu): OBD-II responder, ISO-TP, broadcast frames."""

from __future__ import annotations

import itertools
import time

import can
import pytest

from obddash.config import CONFIG_DIR, load_vehicle
from obddash.sim import Simulator
from obddash.sim.ecu import (
    ID_BRAKE,
    ID_CHASSIS,
    ID_GEAR,
    ID_WHEELS,
    OBD_FUNCTIONAL_ID,
    OBD_PHYSICAL_ID,
    OBD_RESPONSE_ID,
    VIN,
    FakeEcu,
    encode_pid,
    frame_brake,
    frame_chassis,
    pack_be,
    supported_mask,
)
from obddash.sim.vehicle import Controls, VehicleModel, VehicleState

_chan = itertools.count()


@pytest.fixture(scope="module")
def vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


@pytest.fixture
def buses():
    """(tester bus, ecu bus) on a fresh virtual channel, like the runtime wires it."""
    name = f"obddash-test-{next(_chan)}"
    tester = can.Bus(interface="virtual", channel=name)
    ecu = can.Bus(interface="virtual", channel=name)
    yield tester, ecu
    tester.shutdown()
    ecu.shutdown()


@pytest.fixture
def rolling_model(vehicle):
    m = VehicleModel(vehicle, seed=1)
    m.state.speed_mps = 90.0 / 3.6
    m.state.rpm = 90.0 * vehicle.rpm_per_kph(3)
    for _ in range(50):
        m.step(0.01, Controls(throttle_pct=40.0, gear=3, brake_bar=0.0))
    return m


def _recv_id(bus: can.BusABC, arb_id: int, timeout: float = 1.0) -> can.Message | None:
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is not None and msg.arbitration_id == arb_id:
            return msg


def _request(bus: can.BusABC, payload: bytes, arb_id: int = OBD_FUNCTIONAL_ID) -> None:
    data = bytes([len(payload)]) + payload + bytes(7 - len(payload))
    bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))


def _recv_isotp(bus: can.BusABC, timeout: float = 1.0) -> bytes:
    """Tester-side ISO-TP receive from 0x7E8, sending flow control when needed."""
    first = _recv_id(bus, OBD_RESPONSE_ID, timeout)
    assert first is not None, "no response from ECU"
    d = bytes(first.data)
    pci = d[0] >> 4
    if pci == 0:
        return d[1 : 1 + (d[0] & 0x0F)]
    assert pci == 1, f"unexpected PCI {d[0]:02x}"
    length = ((d[0] & 0x0F) << 8) | d[1]
    out = bytearray(d[2:8])
    bus.send(
        can.Message(
            arbitration_id=OBD_PHYSICAL_ID,
            data=bytes([0x30, 0, 0, 0, 0, 0, 0, 0]),
            is_extended_id=False,
        )
    )
    seq = 1
    while len(out) < length:
        cf = _recv_id(bus, OBD_RESPONSE_ID, timeout)
        assert cf is not None, "consecutive frame missing"
        assert cf.data[0] == (0x20 | (seq & 0x0F))
        out += bytes(cf.data[1:8])
        seq += 1
    return bytes(out[:length])


# --------------------------------------------------------------------------------------
# pure helpers
# --------------------------------------------------------------------------------------
def test_pack_be_unsigned_signed_and_clamping():
    assert pack_be(12.34, 0.01, 16) == 1234
    assert pack_be(-12.3, 0.1, 16, signed=True) == (-123) & 0xFFFF
    assert pack_be(0.5, 0.001, 16, signed=True) == 500
    assert pack_be(1e9, 0.01, 16) == 0xFFFF
    assert pack_be(-5, 1.0, 8) == 0
    assert pack_be(200, 1.0, 8, signed=True) == 127
    assert pack_be(-1, 1.0, 8, signed=True) == 0xFF
    assert pack_be(float("nan"), 1.0, 8) == 0
    assert pack_be(50.0, 1.0, 8, offset=-40.0) == 90  # temperature style A-40


def test_supported_masks_declare_exactly_the_implemented_pids():
    def bits(mask: bytes, base: int) -> set[int]:
        val = int.from_bytes(mask, "big")
        return {base + k for k in range(1, 33) if val & (1 << (32 - k))}

    s = VehicleState()
    m00 = bits(encode_pid(0x00, s), 0x00)
    assert {0x04, 0x05, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x20} <= m00
    assert 0x01 not in m00 and 0x1F not in m00
    m20 = bits(encode_pid(0x20, s), 0x20)
    assert m20 == {0x2F, 0x33, 0x40}
    m40 = bits(encode_pid(0x40, s), 0x40)
    assert m40 == {0x42, 0x44, 0x46, 0x49, 0x5C, 0x5E, 0x60}
    m60 = bits(encode_pid(0x60, s), 0x60)
    assert m60 == {0x62, 0x63}
    assert supported_mask(0x00, {0x02}) == bytes([0x40, 0, 0, 0])


def test_pid_encodings_are_the_inverse_of_j1979():
    s = VehicleState(
        rpm=4210.25,
        speed_mps=87.3 / 3.6,
        throttle_pct=64.0,
        load_pct=71.0,
        coolant_c=92.0,
        intake_c=31.0,
        maf_gps=41.27,
        map_kpa=88.0,
        baro_kpa=101.0,
        fuel_pct=48.0,
        batt_v=14.123,
        timing_adv_deg=22.5,
        lambda_=0.85,
        oil_temp_c=105.0,
        fuel_rate_lph=12.35,
        engine_torque_nm=125.0,
    )
    a, b = encode_pid(0x0C, s)
    assert (a * 256 + b) / 4 == pytest.approx(4210.25)
    assert encode_pid(0x0D, s)[0] == 87
    assert encode_pid(0x11, s)[0] * 100 / 255 == pytest.approx(64.0, abs=0.3)
    assert encode_pid(0x04, s)[0] * 100 / 255 == pytest.approx(71.0, abs=0.3)
    assert encode_pid(0x05, s)[0] - 40 == 92
    assert encode_pid(0x0F, s)[0] - 40 == 31
    assert encode_pid(0x5C, s)[0] - 40 == 105
    a, b = encode_pid(0x10, s)
    assert (a * 256 + b) / 100 == pytest.approx(41.27)
    assert encode_pid(0x0B, s)[0] == 88 and encode_pid(0x33, s)[0] == 101
    assert encode_pid(0x2F, s)[0] * 100 / 255 == pytest.approx(48.0, abs=0.3)
    a, b = encode_pid(0x42, s)
    assert (a * 256 + b) / 1000 == pytest.approx(14.123)
    assert encode_pid(0x0E, s)[0] / 2 - 64 == pytest.approx(22.5)
    a, b, _, _ = encode_pid(0x44, s)
    assert (a * 256 + b) * 2 / 65536 == pytest.approx(0.85, abs=0.001)
    a, b = encode_pid(0x5E, s)
    assert (a * 256 + b) / 20 == pytest.approx(12.35)
    pct = encode_pid(0x62, s)[0] - 125
    a, b = encode_pid(0x63, s)
    assert (a * 256 + b) * pct / 100 == pytest.approx(125.0, abs=2.5)
    assert encode_pid(0x01, s) is None and encode_pid(0xFF, s) is None


def test_broadcast_frames_match_the_vehicle_yaml_layout():
    s = VehicleState(
        brake_bar=74.2,
        brake_front_bar=74.2,
        brake_rear_bar=39.95,
        clutch=0.8,
        steering_deg=-12.3,
        yaw_rate_dps=-4.56,
        lat_g=-0.312,
        lon_g=0.221,
    )
    f = frame_brake(s, 5)
    assert int.from_bytes(f[0:2], "big") * 0.01 == pytest.approx(74.2)
    assert int.from_bytes(f[2:4], "big") * 0.01 == pytest.approx(74.2)
    assert int.from_bytes(f[4:6], "big") * 0.01 == pytest.approx(39.95)
    assert f[6] & 0x01 == 1 and f[6] & 0x02 == 2  # brake + clutch switch (bits 48, 49)
    assert f[7] == 5
    assert frame_brake(VehicleState(), 0)[6] == 0
    c = frame_chassis(s)
    assert int.from_bytes(c[0:2], "big", signed=True) * 0.1 == pytest.approx(-12.3)
    assert int.from_bytes(c[2:4], "big", signed=True) * 0.01 == pytest.approx(-4.56)
    assert int.from_bytes(c[4:6], "big", signed=True) * 0.001 == pytest.approx(-0.312)
    assert int.from_bytes(c[6:8], "big", signed=True) * 0.001 == pytest.approx(0.221)


# --------------------------------------------------------------------------------------
# on a virtual bus
# --------------------------------------------------------------------------------------
def test_single_pid_rpm_request(buses, vehicle, rolling_model):
    tester, ecu_bus = buses
    ecu = FakeEcu(vehicle, ecu_bus, lambda: rolling_model.state, latency_s=0.002, broadcast=False)
    ecu.start()
    try:
        _request(tester, bytes([0x01, 0x0C]))
        msg = _recv_id(tester, OBD_RESPONSE_ID)
        assert msg is not None and not msg.is_extended_id
        d = bytes(msg.data)
        assert d[0] == 0x04 and d[1] == 0x41 and d[2] == 0x0C
        assert (d[3] * 256 + d[4]) / 4 == pytest.approx(rolling_model.state.rpm, abs=0.25)
        assert len(d) == 8  # padded
        assert ecu.request_count == 1 and ecu.response_count == 1
    finally:
        ecu.stop()


def test_physical_request_speed_and_supported_mask(buses, vehicle, rolling_model):
    tester, ecu_bus = buses
    ecu = FakeEcu(vehicle, ecu_bus, lambda: rolling_model.state, latency_s=0.0, broadcast=False)
    ecu.start()
    try:
        _request(tester, bytes([0x01, 0x0D]), arb_id=OBD_PHYSICAL_ID)
        d = _recv_isotp(tester)
        assert d[:2] == bytes([0x41, 0x0D])
        assert d[2] == int(round(rolling_model.state.speed_kph))
        _request(tester, bytes([0x01, 0x00]))
        d = _recv_isotp(tester)
        assert d[:2] == bytes([0x41, 0x00])
        mask = int.from_bytes(d[2:6], "big")
        for pid in (0x0C, 0x0D, 0x11):
            assert mask & (1 << (32 - pid)), f"PID {pid:#x} not declared"
    finally:
        ecu.stop()


def test_multi_pid_request_is_concatenated_and_multiframe(buses, vehicle, rolling_model):
    tester, ecu_bus = buses
    ecu = FakeEcu(vehicle, ecu_bus, lambda: rolling_model.state, latency_s=0.0, broadcast=False)
    ecu.start()
    try:
        # short enough for a single frame: 41 0C A B 0D A = 6 bytes
        _request(tester, bytes([0x01, 0x0C, 0x0D]))
        d = _recv_isotp(tester)
        s = rolling_model.state
        assert d[0] == 0x41 and d[1] == 0x0C and d[4] == 0x0D
        assert (d[2] * 256 + d[3]) / 4 == pytest.approx(s.rpm, abs=0.25)
        assert d[5] == int(round(s.speed_kph))
        # six PIDs incl. an unsupported one -> multi-frame with flow control
        _request(tester, bytes([0x01, 0x0C, 0x0D, 0x11, 0x04, 0x01, 0x10]))
        d = _recv_isotp(tester)
        assert d[0] == 0x41
        i = 1
        seen = []
        lengths = {0x0C: 2, 0x0D: 1, 0x11: 1, 0x04: 1, 0x10: 2}
        while i < len(d):
            pid = d[i]
            seen.append(pid)
            i += 1 + lengths[pid]
        assert seen == [0x0C, 0x0D, 0x11, 0x04, 0x10]  # 0x01 skipped
        assert ecu.response_count == 2
    finally:
        ecu.stop()


def test_vin_via_multiframe_with_flow_control(buses, vehicle):
    tester, ecu_bus = buses
    state = VehicleState()
    ecu = FakeEcu(vehicle, ecu_bus, lambda: state, latency_s=0.0, broadcast=False)
    ecu.start()
    try:
        _request(tester, bytes([0x09, 0x02]))
        d = _recv_isotp(tester)
        assert d[:3] == bytes([0x49, 0x02, 0x01])
        assert d[3:].decode("ascii") == VIN
        assert len(VIN) == 17
        _request(tester, bytes([0x09, 0x00]))
        d = _recv_isotp(tester)
        assert d == bytes([0x49, 0x00, 0x40, 0, 0, 0])
    finally:
        ecu.stop()


def test_multiframe_honours_block_size_and_aborts_without_flow_control(buses, vehicle):
    tester, ecu_bus = buses
    state = VehicleState()
    ecu = FakeEcu(vehicle, ecu_bus, lambda: state, latency_s=0.0, broadcast=False)
    ecu.FC_TIMEOUT_S = 0.2
    ecu.start()
    try:
        # block size 1: a flow control is needed before every consecutive frame
        _request(tester, bytes([0x09, 0x02]))
        first = _recv_id(tester, OBD_RESPONSE_ID)
        assert first.data[0] >> 4 == 1
        out = bytearray(first.data[2:8])
        length = ((first.data[0] & 0x0F) << 8) | first.data[1]
        while len(out) < length:
            tester.send(
                can.Message(
                    arbitration_id=OBD_PHYSICAL_ID,
                    data=bytes([0x30, 1, 0, 0, 0, 0, 0, 0]),
                    is_extended_id=False,
                )
            )
            cf = _recv_id(tester, OBD_RESPONSE_ID, 0.5)
            assert cf is not None
            out += bytes(cf.data[1:8])
            # no second frame must arrive before the next FC
            assert _recv_id(tester, OBD_RESPONSE_ID, 0.05) is None
        assert bytes(out[3:length]).decode() == VIN
        # no flow control at all: only the first frame, then the ECU gives up
        _request(tester, bytes([0x09, 0x02]))
        assert _recv_id(tester, OBD_RESPONSE_ID) is not None
        assert _recv_id(tester, OBD_RESPONSE_ID, 0.4) is None
        # ...and it still answers afterwards
        _request(tester, bytes([0x01, 0x0D]))
        assert _recv_isotp(tester)[:2] == bytes([0x41, 0x0D])
    finally:
        ecu.stop()


def test_unknown_mode_gets_negative_response_and_unknown_pid_silence(buses, vehicle):
    tester, ecu_bus = buses
    ecu = FakeEcu(vehicle, ecu_bus, VehicleState, latency_s=0.0, broadcast=False)
    ecu.start()
    try:
        _request(tester, bytes([0x22, 0xF1, 0x90]))
        d = _recv_isotp(tester)
        assert d == bytes([0x7F, 0x22, 0x11])
        _request(tester, bytes([0x03]))
        assert _recv_isotp(tester) == bytes([0x43, 0x00])
        _request(tester, bytes([0x01, 0x01]))
        assert _recv_id(tester, OBD_RESPONSE_ID, 0.2) is None
        # frames for other ids are ignored
        tester.send(
            can.Message(
                arbitration_id=0x123,
                data=bytes([0x02, 0x01, 0x0C, 0, 0, 0, 0, 0]),
                is_extended_id=False,
            )
        )
        assert _recv_id(tester, OBD_RESPONSE_ID, 0.2) is None
    finally:
        ecu.stop()


def test_latency_is_honoured(buses, vehicle):
    tester, ecu_bus = buses
    ecu = FakeEcu(vehicle, ecu_bus, VehicleState, latency_s=0.05, broadcast=False)
    ecu.start()
    try:
        t0 = time.monotonic()
        _request(tester, bytes([0x01, 0x0C]))
        assert _recv_id(tester, OBD_RESPONSE_ID) is not None
        assert time.monotonic() - t0 >= 0.045
    finally:
        ecu.stop()


def test_broadcast_0x200_decodes_to_model_brake_pressure(buses, vehicle):
    tester, ecu_bus = buses
    m = VehicleModel(vehicle, seed=1)
    m.state.speed_mps = 100.0 / 3.6
    for _ in range(30):
        m.step(0.01, Controls(brake_bar=63.7, clutch=1.0, gear=3, steering_deg=-20.0))
    ecu = FakeEcu(vehicle, ecu_bus, lambda: m.state, latency_s=0.0)
    ecu.start()
    try:
        msg = _recv_id(tester, ID_BRAKE)
        assert msg is not None
        d = bytes(msg.data)
        assert int.from_bytes(d[0:2], "big") * 0.01 == pytest.approx(m.state.brake_bar, abs=0.01)
        assert int.from_bytes(d[2:4], "big") * 0.01 == pytest.approx(
            m.state.brake_front_bar, abs=0.01
        )
        assert int.from_bytes(d[4:6], "big") * 0.01 == pytest.approx(
            m.state.brake_rear_bar, abs=0.01
        )
        assert d[6] & 0x01 == 1 and d[6] & 0x02 == 2
        ch = _recv_id(tester, ID_CHASSIS)
        assert int.from_bytes(ch.data[0:2], "big", signed=True) * 0.1 == pytest.approx(-20.0)
        lon = int.from_bytes(ch.data[6:8], "big", signed=True) * 0.001
        assert lon == pytest.approx(m.state.lon_g, abs=0.001)
        wh = _recv_id(tester, ID_WHEELS)
        fl = int.from_bytes(wh.data[0:2], "big") * 0.01
        assert fl == pytest.approx(m.state.wheel_speeds_kph[0], abs=0.01)
        g = _recv_id(tester, ID_GEAR)
        assert int.from_bytes(g.data[0:1], "big", signed=True) == 3
    finally:
        ecu.stop()


def test_broadcast_rates_and_noise_ids(buses, vehicle):
    tester, ecu_bus = buses
    ecu = FakeEcu(vehicle, ecu_bus, VehicleState, latency_s=0.0)
    ecu.start()
    try:
        counts: dict[int, int] = {}
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = tester.recv(timeout=0.1)
            if msg is not None:
                counts[msg.arbitration_id] = counts.get(msg.arbitration_id, 0) + 1
    finally:
        ecu.stop()
    assert 30 <= counts[ID_BRAKE] <= 60
    assert 30 <= counts[ID_CHASSIS] <= 60
    assert 10 <= counts[ID_GEAR] <= 25
    assert counts[0x0A5] > counts[ID_BRAKE]
    assert counts[0x3C2] >= 5
    assert ecu.broadcast_count > 50


def test_simulator_end_to_end_on_virtual_bus(buses, vehicle):
    tester, ecu_bus = buses
    sim = Simulator(vehicle, ecu_bus, script="dyno", seed=2, realtime=True, rate_hz=100.0)
    sim.speed = 3.0
    sim.start()
    try:
        time.sleep(0.5)
        _request(tester, bytes([0x01, 0x0C, 0x0D]))
        d = _recv_isotp(tester)
        assert d[0] == 0x41
        assert sim.t > 0.5
        assert sim.running
        gear = _recv_id(tester, ID_GEAR)
        assert gear is not None
    finally:
        sim.stop()
    assert not sim.running
    assert sim.ecu.response_count >= 1
