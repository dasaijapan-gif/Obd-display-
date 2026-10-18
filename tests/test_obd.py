"""obddash.bus.obd: PID decoders, request builder, ISO-TP assembler, response parsing."""

from __future__ import annotations

import pytest

from obddash.bus import obd
from obddash.config import DEFAULT_PIDS
from obddash.model import Sig

# --------------------------------------------------------------------------------------
# PID table
# --------------------------------------------------------------------------------------
DECODE_CASES = [
    # pid, data, expected, sig
    (0x04, "80", 128 / 2.55, Sig.ENGINE_LOAD_PCT),
    (0x05, "82", 90.0, Sig.COOLANT_C),
    (0x0B, "64", 100.0, Sig.MAP_KPA),
    (0x0C, "1AF8", 1726.0, Sig.RPM),
    (0x0C, "0000", 0.0, Sig.RPM),
    (0x0C, "FFFF", 16383.75, Sig.RPM),
    (0x0D, "50", 80.0, Sig.SPEED_KPH),
    (0x0E, "A0", 160 / 2 - 64, Sig.TIMING_ADV_DEG),
    (0x0F, "28", 0.0, Sig.INTAKE_C),
    (0x10, "1019", 0x1019 / 100, Sig.MAF_GPS),
    (0x11, "FF", 100.0, Sig.THROTTLE_PCT),
    (0x2F, "7F", 127 / 2.55, Sig.FUEL_LEVEL_PCT),
    (0x33, "65", 101.0, Sig.BARO_KPA),
    (0x42, "3714", 14.1, Sig.CONTROL_MODULE_V),
    (0x44, "8000", 1.0, Sig.LAMBDA),
    (0x46, "35", 13.0, Sig.AMBIENT_C),
    (0x49, "40", 64 / 2.55, Sig.ACCEL_PEDAL_PCT),
    (0x5C, "96", 110.0, Sig.OIL_TEMP_C),
    (0x5E, "00C8", 10.0, Sig.FUEL_RATE_LPH),
    (0x62, "A0", 35.0, Sig.ENGINE_TORQUE_PCT),
    (0x62, "00", -125.0, Sig.ENGINE_TORQUE_PCT),
    (0x63, "00FA", 250.0, Sig.ENGINE_REF_TORQUE_NM),
]


@pytest.mark.parametrize("pid,data,expected,sig", DECODE_CASES)
def test_decoders(pid, data, expected, sig):
    pdef = obd.PID_TABLE[pid]
    raw = bytes.fromhex(data)
    assert len(raw) == pdef.nbytes
    assert pdef.decode(raw) == pytest.approx(expected)
    assert pdef.sig == sig
    assert obd.PID_BY_SIG[sig] == pid


def test_table_covers_default_pids_and_sigs():
    for req in DEFAULT_PIDS:
        assert req.pid in obd.PID_TABLE, f"PID 0x{req.pid:02X} missing"
        assert obd.PID_TABLE[req.pid].sig is not None
    # every standard-PID Sig has a decoder
    for base in obd.SUPPORTED_PID_BASES:
        assert obd.PID_TABLE[base].nbytes == 4
    for name in (Sig.RPM, Sig.SPEED_KPH, Sig.THROTTLE_PCT, Sig.LAMBDA, Sig.ENGINE_REF_TORQUE_NM):
        assert name in obd.PID_BY_SIG


def test_extra_decoders():
    assert obd.PID_TABLE[0x06].decode(bytes([0x80])) == pytest.approx(0.0)  # fuel trim 0 %
    assert obd.PID_TABLE[0x1F].decode(bytes([0x01, 0x2C])) == 300.0
    assert obd.PID_TABLE[0x23].decode(bytes([0x00, 0x0A])) == 100.0
    assert obd.PID_TABLE[0x5D].decode(bytes([0x69, 0x00])) == pytest.approx(0x6900 / 128 - 210)
    assert obd.PID_TABLE[0x01].decode(bytes([0x81, 0, 0, 0])) == 1.0  # MIL on


def test_pid_length_known_and_unknown():
    assert obd.pid_length(0x0C) == 2
    assert obd.pid_length(0x14) == 2  # O2 sensor, length only
    assert obd.pid_length(0x24) == 4
    assert obd.pid_length(0xE5) is None


# --------------------------------------------------------------------------------------
# supported-PID bitmasks
# --------------------------------------------------------------------------------------
def test_decode_supported_classic_example():
    # BE 1F A8 13 is the textbook example from the OBD-II PID wiki page
    s = obd.decode_supported(0x00, bytes.fromhex("BE1FA813"))
    assert s == {
        0x01,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x0C,
        0x0D,
        0x0E,
        0x0F,
        0x10,
        0x11,
        0x13,
        0x15,
        0x1C,
        0x1F,
        0x20,
    }
    assert obd.next_range_supported(0x00, s)  # bit 0 = PID 0x20 -> 0x20 mask exists
    assert not obd.next_range_supported(0x20, s)


def test_decode_supported_offsets_and_round_trip():
    assert obd.decode_supported(0x20, bytes.fromhex("80000001")) == {0x21, 0x40}
    assert obd.decode_supported(0x40, bytes.fromhex("00000000")) == set()
    assert obd.decode_supported(0x00, b"\x80") == set()  # truncated -> nothing
    pids = {0x41, 0x44, 0x49, 0x5C, 0x60}
    assert obd.decode_supported(0x40, obd.encode_supported(0x40, pids)) == pids
    # PIDs outside the range are ignored by the encoder
    assert obd.encode_supported(0x40, {0x0C, 0x61}) == b"\x00\x00\x00\x00"


# --------------------------------------------------------------------------------------
# request builder
# --------------------------------------------------------------------------------------
def test_build_request_single_pid_layout():
    msg = obd.build_request([0x0C])
    assert msg.arbitration_id == 0x7DF
    assert not msg.is_extended_id
    assert bytes(msg.data) == bytes.fromhex("02010C0000000000")
    assert msg.dlc == 8


def test_build_request_multi_pid_and_mode():
    msg = obd.build_request([0x0C, 0x0D, 0x11], mode=1)
    assert bytes(msg.data) == bytes.fromhex("04010C0D11000000")
    six = obd.build_request([1, 2, 3, 4, 5, 6])
    assert bytes(six.data) == bytes.fromhex("0701010203040506")
    vin = obd.build_request([0x02], mode=0x09)
    assert bytes(msg.data[:2]) == b"\x04\x01"
    assert bytes(vin.data) == bytes.fromhex("0209020000000000")
    with pytest.raises(ValueError):
        obd.build_request([1, 2, 3, 4, 5, 6, 7])
    with pytest.raises(ValueError):
        obd.build_request([])


def test_build_request_extended_and_custom_id():
    ext = obd.build_request([0x0C], extended=True)
    assert ext.arbitration_id == 0x18DB33F1
    assert ext.is_extended_id
    phys = obd.build_request([0x0C], request_id=0x7E0)
    assert phys.arbitration_id == 0x7E0
    ext_phys = obd.build_request([0x0C], extended=True, request_id=0x18DA10F1)
    assert ext_phys.arbitration_id == 0x18DA10F1


def test_flow_control_addressing():
    fc = obd.build_flow_control(0x7E8)
    assert fc.arbitration_id == 0x7E0
    assert not fc.is_extended_id
    assert bytes(fc.data) == bytes.fromhex("3000000000000000")
    assert obd.flow_control_target(0x7EB) == 0x7E3
    fc29 = obd.build_flow_control(0x18DAF110, extended=True)
    assert fc29.arbitration_id == 0x18DA10F1
    assert fc29.is_extended_id
    assert obd.flow_control_target(0x18DAF117, extended=True) == 0x18DA17F1
    assert obd.physical_request_id(0x7E8) == 0x7E0


def test_is_obd_response_id():
    assert obd.is_obd_response_id(0x7E8)
    assert obd.is_obd_response_id(0x7EF)
    assert not obd.is_obd_response_id(0x7E0)
    assert not obd.is_obd_response_id(0x7F0)
    assert obd.is_obd_response_id(0x18DAF110, extended=True)
    assert obd.is_obd_response_id(0x18DAF1FF, extended=True)
    assert not obd.is_obd_response_id(0x18DA10F1, extended=True)


# --------------------------------------------------------------------------------------
# mode 01 / 09 parsing
# --------------------------------------------------------------------------------------
def test_parse_mode01_single_and_multi():
    assert obd.parse_mode01(bytes.fromhex("410C1AF8")) == [(0x0C, Sig.RPM, 1726.0)]
    multi = obd.parse_mode01(bytes.fromhex("410C1AF80D501180"))
    assert [(p, n) for p, n, _ in multi] == [
        (0x0C, Sig.RPM),
        (0x0D, Sig.SPEED_KPH),
        (0x11, Sig.THROTTLE_PCT),
    ]
    assert multi[0][2] == 1726.0
    assert multi[1][2] == 80.0
    assert multi[2][2] == pytest.approx(128 / 2.55)


def test_parse_mode01_stops_at_unknown_pid_and_truncation():
    # unknown PID 0xE5 after rpm: its length is unknown, so parsing stops there
    assert obd.parse_mode01(bytes.fromhex("410C1AF8E5010D50")) == [(0x0C, Sig.RPM, 1726.0)]
    # truncated: rpm needs 2 bytes, only 1 present
    assert obd.parse_mode01(bytes.fromhex("410C1A")) == []
    # PID with a known length but no decoder (O2 sensor 0x14) is skipped, parsing continues
    out = obd.parse_mode01(bytes.fromhex("4114AABB0D50"))
    assert out == [(0x0D, Sig.SPEED_KPH, 80.0)]


def test_parse_mode01_rejects_negative_and_wrong_service():
    assert obd.parse_mode01(bytes.fromhex("7F0112")) == []
    assert obd.parse_mode01(bytes.fromhex("49020157")) == []
    assert obd.parse_mode01(b"") == []
    assert obd.parse_negative(bytes.fromhex("7F0978")) == (0x09, 0x78)
    assert obd.parse_negative(bytes.fromhex("410C1AF8")) is None


def test_parse_mode01_supported_mask_value():
    out = obd.parse_mode01(bytes.fromhex("4100BE1FA813"))
    assert out[0][0] == 0x00
    assert int(out[0][2]) == 0xBE1FA813


def test_parse_mode09_vin():
    vin = "WAUZZZ8V9KA123456"
    payload = bytes([0x49, 0x02, 0x01]) + vin.encode()
    assert obd.parse_mode09_vin(payload) == vin
    # without the ISO 15765-4 count byte
    assert obd.parse_mode09_vin(bytes([0x49, 0x02]) + vin.encode()) == vin
    # leading NUL padding (some ECUs)
    assert obd.parse_mode09_vin(bytes([0x49, 0x02, 0x01, 0x00, 0x00, 0x00]) + vin.encode()) == vin
    assert obd.parse_mode09_vin(bytes([0x49, 0x02, 0x01]) + b"SHORT") is None
    assert obd.parse_mode09_vin(bytes([0x49, 0x04, 0x01]) + vin.encode()) is None
    assert obd.parse_mode09_vin(bytes.fromhex("7F0912")) is None


def test_parse_response_classification():
    r = obd.parse_response(0x7E8, bytes.fromhex("4100BE1FA8130C1AF8"))
    assert r is not None and r.mode == 1 and r.negative is None
    assert r.supported[0x00] == obd.decode_supported(0, bytes.fromhex("BE1FA813"))
    assert r.values == [(0x0C, Sig.RPM, 1726.0)]
    assert r.pids == {0x00, 0x0C}
    neg = obd.parse_response(0x7E8, bytes.fromhex("7F0112"))
    assert neg is not None and neg.negative == 0x12 and neg.mode == 1
    vin = obd.parse_response(0x7E8, bytes([0x49, 0x02, 0x01]) + b"WAUZZZ8V9KA123456")
    assert vin is not None and vin.vin == "WAUZZZ8V9KA123456" and vin.pids == {0x02}
    assert obd.parse_response(0x7E8, bytes.fromhex("010C")) is None  # a request, not a response
    assert obd.parse_response(0x7E8, b"") is None


# --------------------------------------------------------------------------------------
# ISO-TP
# --------------------------------------------------------------------------------------
VIN = b"WAUZZZ8V9KA123456"
VIN_PAYLOAD = bytes([0x49, 0x02, 0x01]) + VIN  # 20 bytes -> FF + 2 CF


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def test_isotp_single_frame():
    asm = obd.IsoTpAssembler()
    assert asm.feed(0x7E8, bytes.fromhex("04410C1AF8000000")) == (0x7E8, bytes.fromhex("410C1AF8"))
    assert asm.feed(0x7E8, bytes.fromhex("0041")) is None  # length 0 invalid on classic CAN
    assert asm.feed(0x7E8, bytes.fromhex("0541")) is None  # claims 5 bytes, has 1
    assert asm.feed(0x7E8, b"") is None


def test_isotp_multi_frame_vin_with_flow_control_callback():
    fc_requests: list[int] = []
    asm = obd.IsoTpAssembler(on_first_frame=fc_requests.append)
    frames = obd.split_multi_frame(VIN_PAYLOAD)
    assert len(frames) == 3
    assert frames[0][:2] == bytes([0x10, 20])
    assert frames[1][0] == 0x21 and frames[2][0] == 0x22
    assert asm.feed(0x7E8, frames[0]) is None
    assert fc_requests == [0x7E8]  # the reader now sends 30 00 00 to 0x7E0
    assert asm.in_progress == 1
    assert asm.feed(0x7E8, frames[1]) is None
    done = asm.feed(0x7E8, frames[2])
    assert done == (0x7E8, VIN_PAYLOAD)
    assert obd.parse_mode09_vin(done[1]) == VIN.decode()
    assert asm.in_progress == 0


def test_isotp_flow_control_queue_without_callback():
    asm = obd.IsoTpAssembler()
    frames = obd.split_multi_frame(VIN_PAYLOAD)
    asm.feed(0x7E9, frames[0])
    assert asm.pop_flow_control_requests() == [0x7E9]
    assert asm.pop_flow_control_requests() == []


@pytest.mark.parametrize("n", [1, 7, 8, 13, 20, 27, 100, 200])
def test_isotp_split_and_reassemble_round_trip(n):
    payload = bytes((i * 7 + 3) & 0xFF for i in range(n))
    asm = obd.IsoTpAssembler(on_first_frame=lambda _arb: None)
    result = None
    for frame in obd.split_multi_frame(payload):
        assert len(frame) == 8
        result = asm.feed(0x7E8, frame)
    assert result == (0x7E8, payload)
    # sequence numbers wrap 0xF -> 0 on long messages
    if n > 6 + 7 * 15:
        frames = obd.split_multi_frame(payload)
        assert frames[16][0] == 0x20


def test_isotp_out_of_order_sequence_drops_message():
    asm = obd.IsoTpAssembler(on_first_frame=lambda _arb: None)
    frames = obd.split_multi_frame(VIN_PAYLOAD)
    asm.feed(0x7E8, frames[0])
    assert asm.feed(0x7E8, frames[2]) is None  # seq 2 before seq 1
    assert asm.dropped == 1
    assert asm.in_progress == 0
    assert asm.feed(0x7E8, frames[1]) is None  # stray CF after the drop: ignored
    # a fresh FF starts over cleanly
    asm.feed(0x7E8, frames[0])
    asm.feed(0x7E8, frames[1])
    assert asm.feed(0x7E8, frames[2]) == (0x7E8, VIN_PAYLOAD)


def test_isotp_stray_consecutive_frame_ignored():
    asm = obd.IsoTpAssembler()
    assert asm.feed(0x7E8, bytes.fromhex("2141424344454647")) is None
    assert asm.in_progress == 0


def test_isotp_per_source_state_interleaved():
    asm = obd.IsoTpAssembler(on_first_frame=lambda _arb: None)
    a = bytes([0x49, 0x02, 0x01]) + b"AAAAAAAAAAAAAAAAA"
    b = bytes([0x49, 0x02, 0x01]) + b"BBBBBBBBBBBBBBBBB"
    fa, fb = obd.split_multi_frame(a), obd.split_multi_frame(b)
    assert asm.feed(0x7E8, fa[0]) is None
    assert asm.feed(0x7E9, fb[0]) is None
    assert asm.in_progress == 2
    assert asm.feed(0x7E9, fb[1]) is None
    assert asm.feed(0x7E8, fa[1]) is None
    assert asm.feed(0x7E8, fa[2]) == (0x7E8, a)
    assert asm.feed(0x7E9, fb[2]) == (0x7E9, b)


def test_isotp_timeout_drops_stale_partial():
    clock = Clock()
    asm = obd.IsoTpAssembler(timeout_s=1.0, on_first_frame=lambda _arb: None, clock=clock)
    frames = obd.split_multi_frame(VIN_PAYLOAD)
    asm.feed(0x7E8, frames[0])
    clock.t = 0.5
    asm.feed(0x7E8, frames[1])
    clock.t = 1.6  # > 1 s since the last frame of this message
    assert asm.feed(0x7E8, frames[2]) is None
    assert asm.dropped == 1
    assert asm.in_progress == 0
    # explicit purge works too
    asm.feed(0x7E8, frames[0])
    clock.t = 5.0
    assert asm.purge() == 1


def test_isotp_single_frame_supersedes_partial():
    asm = obd.IsoTpAssembler(on_first_frame=lambda _arb: None)
    asm.feed(0x7E8, obd.split_multi_frame(VIN_PAYLOAD)[0])
    assert asm.feed(0x7E8, bytes.fromhex("04410C1AF8000000")) == (0x7E8, bytes.fromhex("410C1AF8"))
    assert asm.in_progress == 0 and asm.dropped == 1


def test_isotp_first_frame_callback_errors_are_contained():
    def boom(_arb):
        raise RuntimeError("no bus")

    asm = obd.IsoTpAssembler(on_first_frame=boom)
    frames = obd.split_multi_frame(VIN_PAYLOAD)
    asm.feed(0x7E8, frames[0])  # must not raise
    asm.feed(0x7E8, frames[1])
    assert asm.feed(0x7E8, frames[2]) == (0x7E8, VIN_PAYLOAD)


def test_build_single_frame_helper():
    msg = obd.build_single_frame(bytes.fromhex("410C1AF8"), 0x7E8)
    assert msg.arbitration_id == 0x7E8 and not msg.is_extended_id
    assert bytes(msg.data) == bytes.fromhex("04410C1AF8000000")
    with pytest.raises(ValueError):
        obd.build_single_frame(bytes(8), 0x7E8)
