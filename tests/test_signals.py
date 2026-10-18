"""obddash.bus.signals: DBC bit extraction, SignalDecoder, DBC import."""

from __future__ import annotations

import random

import pytest

from obddash.bus import signals as sg
from obddash.bus.signals import SignalDecoder, decode_signal, encode_signal, extract_bits, pack_bits
from obddash.config import CONFIG_DIR, SignalConfig, load_vehicle
from obddash.model import Sig

BE, LE = "big_endian", "little_endian"


def _frame(hexstr: str) -> bytes:
    return bytes.fromhex(hexstr.replace(" ", ""))


# --------------------------------------------------------------------------------------
# extract_bits: DBC semantics, hand-computed
# --------------------------------------------------------------------------------------
def test_big_endian_16bit_at_start_bit_7_is_bytes_0_1():
    data = _frame("04 D2 00 00 00 00 00 00")
    assert extract_bits(data, 7, 16, BE, False) == 0x04D2 == 1234


def test_big_endian_16bit_at_start_bit_23_is_bytes_2_3():
    data = _frame("00 00 12 34 00 00 00 00")
    assert extract_bits(data, 23, 16, BE, False) == 0x1234
    assert extract_bits(data, 7, 16, BE, False) == 0


def test_big_endian_unaligned_crosses_byte_boundary():
    # start_bit 3 (byte 0, bit 3) length 8: byte0 bits 3..0 (high nibble of the value)
    # then byte1 bits 7..4 (low nibble)
    data = _frame("0A B0 00 00 00 00 00 00")  # byte0 low nibble = 0xA, byte1 high nibble = 0xB
    assert extract_bits(data, 3, 8, BE, False) == 0xAB
    # 12-bit at start_bit 15 (byte 1 bit 7): byte1 fully, then byte2 bits 7..4
    data = _frame("00 FE D0 00 00 00 00 00")
    assert extract_bits(data, 15, 12, BE, False) == 0xFED


def test_big_endian_single_bits_in_byte_6():
    data = _frame("00 00 00 00 00 00 02 00")  # byte 6 = 0b0000_0010
    assert extract_bits(data, 48, 1, BE, False) == 0  # byte 6 bit 0
    assert extract_bits(data, 49, 1, BE, False) == 1  # byte 6 bit 1
    assert extract_bits(data, 48, 1, LE, False) == 0  # 1-bit: byte order irrelevant
    assert extract_bits(data, 49, 1, LE, False) == 1
    data = _frame("00 00 00 00 00 00 01 00")
    assert extract_bits(data, 48, 1, BE, False) == 1
    assert extract_bits(data, 55, 1, BE, False) == 0  # byte 6 bit 7
    data = _frame("00 00 00 00 00 00 80 00")
    assert extract_bits(data, 55, 1, BE, False) == 1


def test_little_endian_cases():
    data = _frame("D2 04 00 00 00 00 00 00")
    assert extract_bits(data, 0, 16, LE, False) == 0x04D2  # LSB in byte 0
    data = _frame("00 00 00 34 12 00 00 00")
    assert extract_bits(data, 24, 16, LE, False) == 0x1234
    # 12-bit at start_bit 4: byte0 bits 7..4 (low nibble of value), byte1 bits 7..0 (high byte)
    data = _frame("50 AB 00 00 00 00 00 00")
    assert extract_bits(data, 4, 12, LE, False) == 0xAB5
    # straddling three bytes: 20 bits at start 6
    data = _frame("C0 FF FF 03 00 00 00 00")
    assert extract_bits(data, 6, 20, LE, False) == 0xFFFFF
    assert extract_bits(data, 6, 20, LE, True) == -1


def test_signed_two_complement():
    data = _frame("FF 88 FE 66 00 00 00 00")
    assert extract_bits(data, 7, 16, BE, True) == -120
    assert extract_bits(data, 23, 16, BE, True) == -410
    assert extract_bits(data, 7, 16, BE, False) == 0xFF88
    assert extract_bits(_frame("FF"), 7, 8, BE, True) == -1
    assert extract_bits(_frame("80"), 7, 8, BE, True) == -128
    assert extract_bits(_frame("7F"), 7, 8, BE, True) == 127
    assert extract_bits(_frame("08"), 3, 4, LE, True) == 1  # 4-bit field at bits 3..6: LSB set
    assert extract_bits(_frame("40"), 3, 4, LE, True) == -8  # bit 6 = sign bit of that field
    assert extract_bits(_frame("00 80"), 0, 16, LE, True) == -32768


def test_extract_bounds_and_bad_order():
    with pytest.raises(ValueError):
        extract_bits(bytes(2), 7, 24, BE, False)
    with pytest.raises(ValueError):
        extract_bits(bytes(2), 8, 16, LE, False)
    with pytest.raises(ValueError):
        extract_bits(bytes(8), 7, 0, BE, False)
    with pytest.raises(ValueError):
        extract_bits(bytes(8), 7, 8, "middle_endian", False)


# --------------------------------------------------------------------------------------
# pack_bits
# --------------------------------------------------------------------------------------
def test_pack_bits_known_layouts():
    assert pack_bits(bytes(8), 1234, 7, 16, BE) == _frame("04 D2 00 00 00 00 00 00")
    assert pack_bits(bytes(8), 0x1234, 23, 16, BE) == _frame("00 00 12 34 00 00 00 00")
    assert pack_bits(bytes(8), 0x1234, 0, 16, LE) == _frame("34 12 00 00 00 00 00 00")
    assert pack_bits(bytes(8), -120, 7, 16, BE, signed=True) == _frame("FF 88 00 00 00 00 00 00")
    assert pack_bits(bytes(8), 1, 49, 1, BE) == _frame("00 00 00 00 00 00 02 00")
    # packing leaves the other bits alone and clears the field first
    base = _frame("FF FF FF FF FF FF FF FF")
    assert pack_bits(base, 0, 23, 16, BE) == _frame("FF FF 00 00 FF FF FF FF")
    assert pack_bits(base, 0, 49, 1, BE) == _frame("FF FF FF FF FF FF FD FF")
    with pytest.raises(ValueError):
        pack_bits(bytes(8), 256, 7, 8, BE)
    with pytest.raises(ValueError):
        pack_bits(bytes(8), 128, 7, 8, BE, signed=True)
    with pytest.raises(ValueError):
        pack_bits(bytes(8), -1, 7, 8, BE)


def test_pack_extract_round_trip_random():
    rng = random.Random(1234)
    for _ in range(600):
        order = rng.choice([BE, LE])
        length = rng.randint(1, 32)
        signed = rng.random() < 0.5 and length > 1
        if order == LE:
            start = rng.randint(0, 64 - length)
        else:
            # choose a stream position that fits, convert back to a DBC start bit (MSB)
            pos = rng.randint(0, 64 - length)
            byte, off = divmod(pos, 8)
            start = byte * 8 + (7 - off)
        if signed:
            value = rng.randint(-(1 << (length - 1)), (1 << (length - 1)) - 1)
        else:
            value = rng.randint(0, (1 << length) - 1)
        background = bytes(rng.randrange(256) for _ in range(8))
        packed = pack_bits(background, value, start, length, order, signed)
        assert extract_bits(packed, start, length, order, signed) == value, (
            order,
            start,
            length,
            signed,
            value,
        )


# --------------------------------------------------------------------------------------
# sim_sportscar.yaml signals through SignalDecoder
# --------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def sim_vehicle():
    return load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")


@pytest.fixture(scope="module")
def sim_decoder(sim_vehicle):
    return SignalDecoder(sim_vehicle.signals)


def test_sim_decoder_ids(sim_decoder):
    assert sim_decoder.ids == {0x200, 0x201, 0x202, 0x203}
    assert len(sim_decoder) == 14
    assert sim_decoder.decode(0x7E8, bytes(8)) == {}


def test_sim_brake_frame(sim_decoder):
    # 12.34 bar system, 0.01 front, 1.00 rear, byte 6 = 0b10 -> brake off, clutch in
    out = sim_decoder.decode(0x200, _frame("04 D2 00 01 00 64 02 00"))
    assert out[Sig.BRAKE_PRESSURE_BAR] == pytest.approx(12.34)
    assert out[Sig.BRAKE_PRESSURE_FRONT_BAR] == pytest.approx(0.01)
    assert out[Sig.BRAKE_PRESSURE_REAR_BAR] == pytest.approx(1.00)
    assert out[Sig.BRAKE_SWITCH] == 0
    assert out[Sig.CLUTCH_SWITCH] == 1
    out = sim_decoder.decode(0x200, _frame("00 00 00 00 00 00 01 00"))
    assert out[Sig.BRAKE_SWITCH] == 1 and out[Sig.CLUTCH_SWITCH] == 0
    assert out[Sig.BRAKE_PRESSURE_BAR] == 0.0


def test_sim_brake_out_of_range_dropped(sim_decoder):
    out = sim_decoder.decode(0x200, _frame("04 D2 00 01 FF FF 00 00"))  # rear = 655.35 > max 250
    assert Sig.BRAKE_PRESSURE_REAR_BAR not in out
    assert Sig.BRAKE_PRESSURE_BAR in out and Sig.BRAKE_PRESSURE_FRONT_BAR in out


def test_sim_chassis_frame_signed(sim_decoder):
    # steering -12.0 deg (-120 raw), yaw -4.1 (-410), lat +0.5 g (500), lon -1.0 g (-1000)
    out = sim_decoder.decode(0x201, _frame("FF 88 FE 66 01 F4 FC 18"))
    assert out[Sig.STEERING_DEG] == pytest.approx(-12.0)
    assert out[Sig.YAW_RATE_DPS] == pytest.approx(-4.1)
    assert out[Sig.ACC_LAT_G] == pytest.approx(0.5)
    assert out[Sig.ACC_LON_G] == pytest.approx(-1.0)
    out = sim_decoder.decode(0x201, _frame("7F FF 80 00 00 00 00 00"))
    assert out[Sig.STEERING_DEG] == pytest.approx(3276.7)
    assert out[Sig.YAW_RATE_DPS] == pytest.approx(-327.68)


def test_sim_wheel_speed_and_gear(sim_decoder):
    out = sim_decoder.decode(0x202, _frame("27 10 27 11 26 FF 00 00"))
    assert out[Sig.WHEEL_SPEED_FL_KPH] == pytest.approx(100.0)
    assert out[Sig.WHEEL_SPEED_FR_KPH] == pytest.approx(100.01)
    assert out[Sig.WHEEL_SPEED_RL_KPH] == pytest.approx(99.83)
    assert out[Sig.WHEEL_SPEED_RR_KPH] == 0.0
    assert sim_decoder.decode(0x203, _frame("FF 00 00 00 00 00 00 00"))[Sig.GEAR] == -1
    assert sim_decoder.decode(0x203, _frame("06"))[Sig.GEAR] == 6
    assert sim_decoder.decode(0x203, _frame("00"))[Sig.GEAR] == 0


def test_sim_signals_encode_decode_round_trip(sim_vehicle, sim_decoder):
    frame = bytearray(8)
    values = {
        Sig.BRAKE_PRESSURE_BAR: 42.5,
        Sig.BRAKE_PRESSURE_FRONT_BAR: 50.0,
        Sig.BRAKE_PRESSURE_REAR_BAR: 30.25,
        Sig.BRAKE_SWITCH: 1,
        Sig.CLUTCH_SWITCH: 0,
    }
    for cfg in sim_vehicle.signals:
        if cfg.id == 0x200:
            frame = encode_signal(cfg, values[cfg.name], frame)
    out = sim_decoder.decode(0x200, bytes(frame))
    for name, v in values.items():
        assert out[name] == pytest.approx(v)
    # per-signal helper agrees
    for cfg in sim_vehicle.signals:
        if cfg.id == 0x200:
            assert decode_signal(cfg, bytes(frame)) == pytest.approx(values[cfg.name])


def test_short_frame_skips_signals_that_do_not_fit(sim_decoder):
    out = sim_decoder.decode(0x200, _frame("04 D2 00"))  # DLC 3
    assert out == {Sig.BRAKE_PRESSURE_BAR: pytest.approx(12.34)}


# --------------------------------------------------------------------------------------
# SignalDecoder features on hand-built configs
# --------------------------------------------------------------------------------------
def test_little_endian_signal_with_scale_offset_and_limits():
    cfg = SignalConfig(
        name="coolant_c",
        id=0x3A0,
        start_bit=8,
        length=16,
        byte_order=LE,
        scale=0.1,
        offset=-40,
        min=-40,
        max=150,
    )
    dec = SignalDecoder([cfg])
    assert dec.decode(0x3A0, _frame("00 E8 03 00 00 00 00 00"))["coolant_c"] == pytest.approx(
        60.0
    )  # 1000*0.1-40
    assert dec.decode(0x3A0, _frame("00 FF FF 00 00 00 00 00")) == {}  # 6513.5 > max
    assert dec.decode(0x3A0, _frame("00 00 00 00 00 00 00 00"))["coolant_c"] == pytest.approx(
        -40.0
    )  # == min ok


def test_mux_byte_gates_decoding():
    a = SignalConfig(name="a", id=0x100, byte=1, length=8, mux_byte=0, mux_value=0x01)
    b = SignalConfig(name="b", id=0x100, byte=1, length=8, mux_byte=0, mux_value=0x02)
    plain = SignalConfig(name="c", id=0x100, byte=2, length=8)
    dec = SignalDecoder([a, b, plain])
    assert dec.decode(0x100, _frame("01 11 22")) == {"a": 0x11, "c": 0x22}
    assert dec.decode(0x100, _frame("02 11 22")) == {"b": 0x11, "c": 0x22}
    assert dec.decode(0x100, _frame("03 11 22")) == {"c": 0x22}
    assert decode_signal(a, _frame("02 11 22")) is None
    # encode_signal sets the mux byte too
    assert encode_signal(b, 0x99, bytes(8))[:2] == _frame("02 99")


def test_byte_shortcut_resolution_matches_start_bit():
    be = SignalConfig(name="x", id=1, byte=2, length=16)
    assert be.start_bit == 23
    le = SignalConfig(name="x", id=1, byte=2, length=16, byte_order=LE)
    assert le.start_bit == 16
    data = _frame("00 00 12 34 00 00 00 00")
    assert SignalDecoder([be]).decode(1, data)["x"] == 0x1234
    assert SignalDecoder([le]).decode(1, data)["x"] == 0x3412


def test_decoder_handles_multiple_ids_and_hex_strings():
    cfgs = [
        SignalConfig(name="s1", id="0x200", byte=0, length=8),
        SignalConfig(name="s2", id=0x201, byte=0, length=8, signed=True),
    ]
    dec = SignalDecoder(cfgs)
    assert dec.ids == {0x200, 0x201}
    assert dec.decode(0x200, _frame("FF"))["s1"] == 255
    assert dec.decode(0x201, _frame("FF"))["s2"] == -1
    assert dec.names == ["s1", "s2"]


# --------------------------------------------------------------------------------------
# DBC import
# --------------------------------------------------------------------------------------
def _cantools_available() -> bool:
    try:
        import cantools  # noqa: F401
    except ImportError:
        return False
    return True


@pytest.mark.skipif(
    _cantools_available(), reason="cantools installed; the ImportError path is not reachable"
)
def test_load_dbc_without_cantools_raises_clear_error(tmp_path):
    p = tmp_path / "x.dbc"
    p.write_text("")
    with pytest.raises(ImportError, match="cantools"):
        sg.load_dbc(p, {"BrakePressure": Sig.BRAKE_PRESSURE_BAR})


DBC_TEXT = """VERSION ""

NS_ :

BS_:

BU_: ECU

BO_ 512 BRAKE: 8 ECU
 SG_ BrakePressure : 7|16@0+ (0.01,0) [0|250] "bar" Vector__XXX
 SG_ BrakeSwitch : 48|1@1+ (1,0) [0|1] "" Vector__XXX

BO_ 513 CHASSIS: 8 ECU
 SG_ SteerAngle : 0|16@1- (0.1,0) [-780|780] "deg" Vector__XXX

"""


@pytest.mark.skipif(not _cantools_available(), reason="cantools not installed")
def test_load_dbc_with_cantools(tmp_path):
    p = tmp_path / "car.dbc"
    p.write_text(DBC_TEXT)
    cfgs = sg.load_dbc(
        p,
        {
            "BrakePressure": Sig.BRAKE_PRESSURE_BAR,
            "BRAKE.BrakeSwitch": Sig.BRAKE_SWITCH,
            "SteerAngle": Sig.STEERING_DEG,
        },
    )
    by_name = {c.name: c for c in cfgs}
    assert set(by_name) == {Sig.BRAKE_PRESSURE_BAR, Sig.BRAKE_SWITCH, Sig.STEERING_DEG}
    bp = by_name[Sig.BRAKE_PRESSURE_BAR]
    assert (bp.id, bp.start_bit, bp.length, bp.byte_order, bp.scale, bp.max) == (
        0x200,
        7,
        16,
        BE,
        0.01,
        250,
    )
    st = by_name[Sig.STEERING_DEG]
    assert (st.id, st.start_bit, st.byte_order, st.signed, st.scale) == (0x201, 0, LE, True, 0.1)
    dec = SignalDecoder(cfgs)
    assert dec.decode(0x200, _frame("04 D2 00 00 00 00 01 00")) == {
        Sig.BRAKE_PRESSURE_BAR: pytest.approx(12.34),
        Sig.BRAKE_SWITCH: 1,
    }
    assert dec.decode(0x201, _frame("88 FF 00 00 00 00 00 00"))[Sig.STEERING_DEG] == pytest.approx(
        -12.0
    )
