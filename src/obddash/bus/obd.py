"""OBD-II over CAN: ISO 15765-4 transport (ISO-TP) and ISO 15031-5 / SAE J1979 PIDs.

Everything here is a pure function or a small state machine with no I/O, so the
protocol can be unit-tested byte for byte and reused by the simulator's fake ECU.

Transport (ISO 15765-2 "ISO-TP") in one paragraph
-------------------------------------------------
A request is a *functional* (broadcast) frame to ``0x7DF`` (11-bit) or ``0x18DB33F1``
(29-bit).  Every ECU that supports the PID answers from its *physical* id,
``0x7E8..0x7EF`` (11-bit, engine ECU is usually ``0x7E8``) or ``0x18DAF1xx`` (29-bit,
``xx`` = ECU address).  The first data byte of each frame is the PCI (protocol
control information):

====  ==========  =================================================================
PCI   frame       meaning
====  ==========  =================================================================
0x0L  single      L = payload length (1..7), payload follows
0x1L  first       L + next byte = 12-bit total length, 6 payload bytes follow;
                  the receiver must answer with a flow-control frame
0x2N  consecutive N = sequence number 1,2,..,F,0,1,.. ; 7 payload bytes
0x3S  flow ctrl   S = 0 continue / 1 wait / 2 overflow; then block size, STmin
====  ==========  =================================================================

Flow control for a response from ``0x7E8`` goes to that ECU's physical request id
``0x7E0`` (``response_id - 8``); for 29-bit ``0x18DAF1xx`` it goes to ``0x18DAxxF1``.

Application layer (SAE J1979)
-----------------------------
Mode 01 request payload is ``[0x01, pid, ...]`` (up to 6 PIDs per request on ECUs
that allow it); the positive response is ``[0x41, pid, data..., pid, data..., ...]``
where each PID's data length is fixed by the standard (:data:`PID_TABLE`).  Negative
responses are ``[0x7F, mode, nrc]``.  PIDs ``0x00, 0x20, 0x40, ...`` return a 32-bit
bitmask of the next 32 supported PIDs (MSB = ``base + 1``).
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

import can

from obddash.model import Sig

log = logging.getLogger(__name__)

# --------------------------------------------------------------------------------------
# Addressing
# --------------------------------------------------------------------------------------
OBD_REQUEST_ID_11 = 0x7DF  # functional request, 11-bit
OBD_RESPONSE_IDS_11 = tuple(range(0x7E8, 0x7F0))  # 0x7E8 .. 0x7EF
OBD_REQUEST_ID_29 = 0x18DB33F1  # functional request, 29-bit (tester = 0xF1)
OBD_RESPONSE_PREFIX_29 = 0x18DAF100  # responses are 0x18DAF1xx (xx = ECU address)
OBD_RESPONSE_MASK_29 = 0x1FFFFF00

MAX_PIDS_PER_REQUEST = 6  # ISO 15031-5: a single frame holds mode + 6 PIDs
PADDING_BYTE = 0x00

# Negative response codes we care about
NRC_SERVICE_NOT_SUPPORTED = 0x11
NRC_SUBFUNCTION_NOT_SUPPORTED = 0x12
NRC_REQUEST_OUT_OF_RANGE = 0x31
NRC_RESPONSE_PENDING = 0x78  # "requestCorrectlyReceived-ResponsePending": wait longer


def is_obd_response_id(arbitration_id: int, extended: bool = False) -> bool:
    """True for a physical ECU response id (11-bit ``0x7E8..0x7EF`` or 29-bit ``0x18DAF1xx``)."""
    if extended:
        return (arbitration_id & OBD_RESPONSE_MASK_29) == OBD_RESPONSE_PREFIX_29
    return 0x7E8 <= arbitration_id <= 0x7EF


def flow_control_target(response_id: int, extended: bool = False) -> int:
    """Arbitration id a flow-control frame must be sent to, for a first frame received
    from ``response_id`` (the ECU's physical request id)."""
    if extended:
        ecu = response_id & 0xFF
        return 0x18DA0000 | (ecu << 8) | 0xF1
    return response_id - 8


def physical_request_id(response_id: int, extended: bool = False) -> int:
    """Alias of :func:`flow_control_target`: the id to address one ECU directly."""
    return flow_control_target(response_id, extended)


# --------------------------------------------------------------------------------------
# PID table (mode 01)
# --------------------------------------------------------------------------------------
@dataclass(frozen=True)
class PidDef:
    """One mode-01 PID: its fixed data length and the formula turning bytes into a value."""

    pid: int
    name: str
    nbytes: int
    decode: Callable[[bytes], float]
    unit: str = ""
    sig: str | None = None  # canonical model.Sig name, if the metrics use it


def _a(d: bytes) -> int:
    return d[0]


def _ab(d: bytes) -> int:
    return (d[0] << 8) | d[1]


def _abcd(d: bytes) -> float:
    return float(int.from_bytes(d[:4], "big"))


def _pct(d: bytes) -> float:  # A * 100 / 255
    return d[0] / 2.55


def _temp(d: bytes) -> float:  # A - 40 (deg C)
    return float(d[0] - 40)


def _trim(d: bytes) -> float:  # fuel trim: A/1.28 - 100  (%)
    return d[0] / 1.28 - 100.0


SUPPORTED_PID_BASES = (0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0)

_DEFS: list[PidDef] = [
    # supported-PID bitmasks: 4 bytes, bit 31 (MSB of A) = base+1 ... bit 0 = base+0x20
    *[
        PidDef(base, f"pids_supported_{base + 1:02X}_{base + 0x20:02X}", 4, _abcd, "bitmask")
        for base in SUPPORTED_PID_BASES
    ],
    PidDef(0x01, "mil_on", 4, lambda d: float((d[0] >> 7) & 1), "bool"),
    PidDef(0x03, "fuel_system_status", 2, lambda d: float(d[0]), "enum"),
    PidDef(0x04, "engine_load_pct", 1, _pct, "%", Sig.ENGINE_LOAD_PCT),
    PidDef(0x05, "coolant_c", 1, _temp, "degC", Sig.COOLANT_C),
    PidDef(0x06, "stft_b1_pct", 1, _trim, "%"),
    PidDef(0x07, "ltft_b1_pct", 1, _trim, "%"),
    PidDef(0x08, "stft_b2_pct", 1, _trim, "%"),
    PidDef(0x09, "ltft_b2_pct", 1, _trim, "%"),
    PidDef(0x0A, "fuel_pressure_kpa", 1, lambda d: float(3 * d[0]), "kPa"),
    PidDef(0x0B, "map_kpa", 1, lambda d: float(d[0]), "kPa", Sig.MAP_KPA),
    PidDef(0x0C, "rpm", 2, lambda d: _ab(d) / 4.0, "rpm", Sig.RPM),
    PidDef(0x0D, "speed_kph", 1, lambda d: float(d[0]), "km/h", Sig.SPEED_KPH),
    PidDef(0x0E, "timing_adv_deg", 1, lambda d: d[0] / 2.0 - 64.0, "deg", Sig.TIMING_ADV_DEG),
    PidDef(0x0F, "intake_c", 1, _temp, "degC", Sig.INTAKE_C),
    PidDef(0x10, "maf_gps", 2, lambda d: _ab(d) / 100.0, "g/s", Sig.MAF_GPS),
    PidDef(0x11, "throttle_pct", 1, _pct, "%", Sig.THROTTLE_PCT),
    PidDef(0x1C, "obd_standard", 1, lambda d: float(d[0]), "enum"),
    PidDef(0x1F, "run_time_s", 2, lambda d: float(_ab(d)), "s"),
    PidDef(0x21, "distance_mil_km", 2, lambda d: float(_ab(d)), "km"),
    PidDef(0x23, "fuel_rail_kpa", 2, lambda d: float(10 * _ab(d)), "kPa"),
    PidDef(0x2C, "egr_pct", 1, _pct, "%"),
    PidDef(0x2F, "fuel_level_pct", 1, _pct, "%", Sig.FUEL_LEVEL_PCT),
    PidDef(0x30, "warmups_since_clear", 1, lambda d: float(d[0]), ""),
    PidDef(0x31, "distance_since_clear_km", 2, lambda d: float(_ab(d)), "km"),
    PidDef(0x33, "baro_kpa", 1, lambda d: float(d[0]), "kPa", Sig.BARO_KPA),
    PidDef(0x42, "control_module_v", 2, lambda d: _ab(d) / 1000.0, "V", Sig.CONTROL_MODULE_V),
    PidDef(0x43, "abs_load_pct", 2, lambda d: _ab(d) / 2.55, "%"),
    PidDef(0x44, "lambda", 2, lambda d: _ab(d) / 32768.0, "", Sig.LAMBDA),
    PidDef(0x45, "rel_throttle_pct", 1, _pct, "%"),
    PidDef(0x46, "ambient_c", 1, _temp, "degC", Sig.AMBIENT_C),
    PidDef(0x47, "abs_throttle_b_pct", 1, _pct, "%"),
    PidDef(0x49, "accel_pedal_pct", 1, _pct, "%", Sig.ACCEL_PEDAL_PCT),
    PidDef(0x4A, "accel_pedal_e_pct", 1, _pct, "%"),
    PidDef(0x4C, "throttle_actuator_pct", 1, _pct, "%"),
    PidDef(0x51, "fuel_type", 1, lambda d: float(d[0]), "enum"),
    PidDef(0x52, "ethanol_pct", 1, _pct, "%"),
    PidDef(0x5A, "rel_accel_pedal_pct", 1, _pct, "%"),
    PidDef(0x5B, "hybrid_batt_pct", 1, _pct, "%"),
    PidDef(0x5C, "oil_temp_c", 1, _temp, "degC", Sig.OIL_TEMP_C),
    PidDef(0x5D, "fuel_inj_timing_deg", 2, lambda d: _ab(d) / 128.0 - 210.0, "deg"),
    PidDef(0x5E, "fuel_rate_lph", 2, lambda d: _ab(d) / 20.0, "L/h", Sig.FUEL_RATE_LPH),
    PidDef(0x61, "driver_demand_torque_pct", 1, lambda d: float(d[0] - 125), "%"),
    PidDef(0x62, "engine_torque_pct", 1, lambda d: float(d[0] - 125), "%", Sig.ENGINE_TORQUE_PCT),
    PidDef(
        0x63, "engine_ref_torque_nm", 2, lambda d: float(_ab(d)), "Nm", Sig.ENGINE_REF_TORQUE_NM
    ),
]

PID_TABLE: dict[int, PidDef] = {p.pid: p for p in _DEFS}
PID_BY_SIG: dict[str, int] = {p.sig: p.pid for p in _DEFS if p.sig}

# Standard data lengths of mode-01 PIDs we do not decode, so a multi-PID response can
# be walked past them.  Anything missing here AND in PID_TABLE stops the parser.
_STANDARD_LENGTHS: dict[int, int] = {
    0x02: 2,
    0x12: 1,
    0x13: 1,
    **{p: 2 for p in range(0x14, 0x1C)},  # O2 sensor voltage / trim
    0x1D: 1,
    0x1E: 1,
    0x22: 2,
    **{p: 4 for p in range(0x24, 0x2C)},  # O2 sensor lambda / voltage
    0x2D: 1,
    0x2E: 1,
    0x32: 2,
    **{p: 4 for p in range(0x34, 0x3C)},  # O2 sensor lambda / current
    **{p: 2 for p in range(0x3C, 0x40)},  # catalyst temperatures
    0x41: 4,
    0x48: 1,
    0x4B: 1,
    0x4D: 2,
    0x4E: 2,
    0x4F: 4,
    0x50: 4,
    0x53: 2,
    0x54: 2,
    **{p: 2 for p in range(0x55, 0x59)},  # secondary O2 trims
    0x59: 2,
    0x5F: 1,
    0x64: 5,
    0x65: 2,
    0x66: 5,
    0x67: 3,
}


def pid_length(pid: int) -> int | None:
    """Data length in bytes of a mode-01 PID, or None if unknown."""
    p = PID_TABLE.get(pid)
    if p is not None:
        return p.nbytes
    return _STANDARD_LENGTHS.get(pid)


def decode_supported(pid_base: int, data: bytes) -> set[int]:
    """Decode a supported-PID bitmask (PID 0x00/0x20/...) into the set of PIDs it flags.

    Bit 31 (MSB of byte A) is ``pid_base + 1``; bit 0 (LSB of byte D) is ``pid_base + 0x20``.
    """
    if len(data) < 4:
        return set()
    mask = int.from_bytes(data[:4], "big")
    return {pid_base + 32 - i for i in range(32) if (mask >> i) & 1}


def encode_supported(pid_base: int, pids: set[int]) -> bytes:
    """Inverse of :func:`decode_supported` (for simulators and tests)."""
    mask = 0
    for pid in pids:
        offset = pid - pid_base
        if 1 <= offset <= 32:
            mask |= 1 << (32 - offset)
    return mask.to_bytes(4, "big")


def next_range_supported(pid_base: int, supported: set[int]) -> bool:
    """A mask's last bit (``base + 0x20``) says whether the next bitmask PID exists."""
    return (pid_base + 0x20) in supported


# --------------------------------------------------------------------------------------
# Request builder
# --------------------------------------------------------------------------------------
def build_request(
    pids: list[int],
    mode: int = 1,
    *,
    extended: bool = False,
    request_id: int = OBD_REQUEST_ID_11,
) -> can.Message:
    """Build a single-frame OBD request: ``[PCI=len] [mode] [pid...]`` padded to 8 bytes.

    Up to :data:`MAX_PIDS_PER_REQUEST` (6) PIDs fit in one frame.  With ``extended``
    the default functional id becomes ``0x18DB33F1`` unless ``request_id`` was given.
    """
    if not 1 <= len(pids) <= MAX_PIDS_PER_REQUEST:
        raise ValueError(f"1..{MAX_PIDS_PER_REQUEST} PIDs per request, got {len(pids)}")
    if extended and request_id == OBD_REQUEST_ID_11:
        request_id = OBD_REQUEST_ID_29
    payload = bytes([mode & 0xFF, *(p & 0xFF for p in pids)])
    frame = bytes([len(payload)]) + payload
    frame = frame + bytes([PADDING_BYTE]) * (8 - len(frame))
    return can.Message(arbitration_id=request_id, data=frame, is_extended_id=extended)


def build_flow_control(
    response_id: int,
    *,
    extended: bool = False,
    block_size: int = 0,
    st_min: int = 0,
) -> can.Message:
    """``30 BS STmin`` "continue to send" frame addressed to the ECU that sent a first
    frame from ``response_id``.  BS=0 means "send everything", STmin=0 no gap."""
    target = flow_control_target(response_id, extended)
    frame = bytes([0x30, block_size & 0xFF, st_min & 0xFF]) + bytes([PADDING_BYTE]) * 5
    return can.Message(arbitration_id=target, data=frame, is_extended_id=extended)


def build_single_frame(
    payload: bytes, arbitration_id: int, *, extended: bool = False
) -> can.Message:
    """Wrap a <=7 byte payload in an ISO-TP single frame (used by simulators/tests)."""
    if not 1 <= len(payload) <= 7:
        raise ValueError("single frame payload must be 1..7 bytes")
    frame = bytes([len(payload)]) + payload
    frame += bytes([PADDING_BYTE]) * (8 - len(frame))
    return can.Message(arbitration_id=arbitration_id, data=frame, is_extended_id=extended)


def split_multi_frame(payload: bytes) -> list[bytes]:
    """Split a payload into ISO-TP frames (single frame if it fits, else FF + CFs).

    Pure helper for the sender side (simulators / tests); the frames are 8-byte padded.
    """
    if len(payload) <= 7:
        frame = bytes([len(payload)]) + payload
        return [frame + bytes([PADDING_BYTE]) * (8 - len(frame))]
    if len(payload) > 0xFFF:
        raise ValueError("ISO-TP payload > 4095 bytes not supported")
    total = len(payload)
    frames = [bytes([0x10 | (total >> 8), total & 0xFF]) + payload[:6]]
    pos, seq = 6, 1
    while pos < total:
        chunk = payload[pos : pos + 7]
        frame = bytes([0x20 | seq]) + chunk
        frames.append(frame + bytes([PADDING_BYTE]) * (8 - len(frame)))
        pos += 7
        seq = (seq + 1) & 0x0F
    return frames


# --------------------------------------------------------------------------------------
# ISO-TP reassembly
# --------------------------------------------------------------------------------------
@dataclass
class _Partial:
    total: int
    buf: bytearray
    next_seq: int
    t_last: float


class IsoTpAssembler:
    """Reassemble ISO-TP messages from raw frames, one state per source id.

    ``feed(arbitration_id, data)`` returns ``(arbitration_id, payload)`` when a message
    completes, else ``None``.  When a first frame arrives the ECU is waiting for a flow
    control frame: ``on_first_frame(arbitration_id)`` is called (the reader sends
    :func:`build_flow_control` there); without a callback the id is queued and can be
    fetched with :meth:`pop_flow_control_requests`.

    Partial messages that see no frame for ``timeout_s`` are dropped.  Out-of-sequence
    consecutive frames abort the partial message (the ECU will not resend; the next
    poll gets a fresh answer).
    """

    def __init__(
        self,
        *,
        timeout_s: float = 1.0,
        on_first_frame: Callable[[int], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.timeout_s = timeout_s
        self.on_first_frame = on_first_frame
        self._clock = clock
        self._partials: dict[int, _Partial] = {}
        self._fc_due: list[int] = []
        self.dropped = 0  # partial messages abandoned (timeouts, bad sequence)

    # -- public -------------------------------------------------------------------------
    def feed(
        self, arbitration_id: int, data: bytes, now: float | None = None
    ) -> tuple[int, bytes] | None:
        if not data:
            return None
        if now is None:
            now = self._clock()
        if self._partials:
            self.purge(now)
        pci_type = data[0] >> 4
        if pci_type == 0x0:
            return self._single(arbitration_id, data)
        if pci_type == 0x1:
            self._first(arbitration_id, data, now)
            return None
        if pci_type == 0x2:
            return self._consecutive(arbitration_id, data, now)
        # 0x3 = flow control (we only receive those if we were the sender); others invalid
        return None

    def pop_flow_control_requests(self) -> list[int]:
        """Ids that sent a first frame since the last call (only used without a callback)."""
        due, self._fc_due = self._fc_due, []
        return due

    def purge(self, now: float | None = None) -> int:
        """Drop partial messages older than ``timeout_s``; returns how many were dropped."""
        if now is None:
            now = self._clock()
        stale = [k for k, p in self._partials.items() if now - p.t_last > self.timeout_s]
        for k in stale:
            del self._partials[k]
            self.dropped += 1
        return len(stale)

    @property
    def in_progress(self) -> int:
        return len(self._partials)

    # -- frame types ----------------------------------------------------------------------
    def _single(self, arb: int, data: bytes) -> tuple[int, bytes] | None:
        length = data[0] & 0x0F
        if length == 0 or length > len(data) - 1:
            return None  # length 0 is the CAN-FD escape form; not valid on classic CAN
        if arb in self._partials:  # a new message supersedes an unfinished one
            del self._partials[arb]
            self.dropped += 1
        return arb, bytes(data[1 : 1 + length])

    def _first(self, arb: int, data: bytes, now: float) -> None:
        if len(data) < 3:
            return
        total = ((data[0] & 0x0F) << 8) | data[1]
        if total <= 7:
            return  # must have been a single frame; malformed
        if arb in self._partials:
            self.dropped += 1
        self._partials[arb] = _Partial(total, bytearray(data[2:8]), 1, now)
        if self.on_first_frame is not None:
            try:
                self.on_first_frame(arb)
            except Exception:  # never let a transport callback kill the rx thread
                log.exception("flow-control callback failed for 0x%X", arb)
        else:
            self._fc_due.append(arb)

    def _consecutive(self, arb: int, data: bytes, now: float) -> tuple[int, bytes] | None:
        part = self._partials.get(arb)
        if part is None:
            return None  # stray CF (we missed the FF, or it timed out)
        seq = data[0] & 0x0F
        if seq != part.next_seq:
            log.debug("ISO-TP 0x%X: expected seq %d got %d, dropping", arb, part.next_seq, seq)
            del self._partials[arb]
            self.dropped += 1
            return None
        part.buf += data[1:8]
        part.next_seq = (seq + 1) & 0x0F
        part.t_last = now
        if len(part.buf) >= part.total:
            del self._partials[arb]
            return arb, bytes(part.buf[: part.total])
        return None


# --------------------------------------------------------------------------------------
# Application-layer parsing
# --------------------------------------------------------------------------------------
def parse_negative(payload: bytes) -> tuple[int, int] | None:
    """``(mode, nrc)`` if ``payload`` is a negative response ``7F mode nrc``, else None."""
    if len(payload) >= 3 and payload[0] == 0x7F:
        return payload[1], payload[2]
    return None


def iter_mode01(payload: bytes) -> Iterator[tuple[int, bytes]]:
    """Yield ``(pid, data)`` pairs from a positive mode-01 response ``41 pid data ...``.

    Stops at the first PID whose length is unknown (the rest cannot be delimited) or
    when the data is truncated.  Non-mode-01 and negative responses yield nothing.
    """
    if len(payload) < 2 or payload[0] != 0x41:
        return
    i = 1
    n = len(payload)
    while i < n:
        pid = payload[i]
        length = pid_length(pid)
        if length is None:
            log.debug("mode 01 response: unknown PID 0x%02X, stopping", pid)
            return
        data = payload[i + 1 : i + 1 + length]
        if len(data) < length:
            return
        yield pid, bytes(data)
        i += 1 + length


def parse_mode01(payload: bytes) -> list[tuple[int, str, float]]:
    """Decode a mode-01 response into ``[(pid, name, value), ...]``.

    ``name`` is the canonical :class:`~obddash.model.Sig` name when the PID has one,
    else the PID's own name from :data:`PID_TABLE`.  Multi-PID responses yield one
    entry per PID.  PIDs with a known length but no decoder are skipped.
    """
    out: list[tuple[int, str, float]] = []
    for pid, data in iter_mode01(payload):
        pdef = PID_TABLE.get(pid)
        if pdef is None:
            continue
        try:
            value = float(pdef.decode(data))
        except Exception:
            log.debug("PID 0x%02X: decode failed on %s", pid, data.hex())
            continue
        out.append((pid, pdef.sig or pdef.name, value))
    return out


def parse_mode09_vin(payload: bytes) -> str | None:
    """Extract the VIN from a mode-09 PID-02 response ``49 02 [count] <17 ASCII bytes>``."""
    if len(payload) < 2 + 17 or payload[0] != 0x49 or payload[1] != 0x02:
        return None
    body = payload[2:]
    if len(body) >= 18 and body[0] <= 0x05:  # ISO 15765-4 responses carry a count byte
        body = body[1:]
    # some ECUs pad with NULs before/after the 17 characters
    chars = bytes(b for b in body if 0x20 < b < 0x7F)
    vin = chars[:17].decode("ascii")
    if len(vin) != 17 or not vin.isalnum():
        return None
    return vin.upper()


@dataclass
class ObdResponse:
    """Convenience wrapper used by the reader: one fully parsed positive/negative response."""

    arbitration_id: int
    mode: int  # requested mode (response byte - 0x40), or the mode in a 7F frame
    negative: int | None = None  # NRC when negative
    values: list[tuple[int, str, float]] = field(default_factory=list)  # mode 01
    supported: dict[int, set[int]] = field(default_factory=dict)  # base -> set, mode 01
    pids: set[int] = field(default_factory=set)  # every PID present in the response
    vin: str | None = None  # mode 09


def parse_response(arbitration_id: int, payload: bytes) -> ObdResponse | None:
    """Classify and decode any OBD response payload we know how to read."""
    if not payload:
        return None
    neg = parse_negative(payload)
    if neg is not None:
        return ObdResponse(arbitration_id, neg[0], negative=neg[1])
    sid = payload[0]
    if sid < 0x40:
        return None  # a request, not a response
    mode = sid - 0x40
    resp = ObdResponse(arbitration_id, mode)
    if mode == 0x01:
        for pid, data in iter_mode01(payload):
            resp.pids.add(pid)
            if pid in SUPPORTED_PID_BASES:
                resp.supported[pid] = decode_supported(pid, data)
                continue
            pdef = PID_TABLE.get(pid)
            if pdef is None:
                continue
            try:
                resp.values.append((pid, pdef.sig or pdef.name, float(pdef.decode(data))))
            except Exception:
                log.debug("PID 0x%02X: decode failed on %s", pid, data.hex())
    elif mode == 0x09:
        if len(payload) >= 2:
            resp.pids.add(payload[1])
        resp.vin = parse_mode09_vin(payload)
    return resp
