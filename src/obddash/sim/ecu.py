"""Fake ECU on a python-can bus: answers OBD-II requests and broadcasts the
vehicle-specific signals described in ``config/vehicles/sim_sportscar.yaml``.

Two independent parts, both fed by ``state_fn()`` (the live :class:`VehicleState`):

**OBD-II responder** (ISO 15765-4 / SAE J1979 over 11-bit CAN)

* Listens with a ``can.Notifier`` for requests on 0x7DF (functional) and 0x7E0
  (physical) and answers from 0x7E8.  Frames are parsed as ISO-TP: single
  frames (``0N ...``), first frames (``1N NN ...``) + consecutive frames for long
  requests, and flow control (``3S BS ST``) while *we* are sending a long reply.
* Requests are handed to a worker thread (so the notifier thread never blocks on
  flow control) which sleeps ``latency_s`` first: a real ECU takes a few ms, and
  that is what makes the reader's scheduler meaningful.
* Mode 01: the PIDs in :data:`PID_LENGTHS`, encoded with the inverse of the J1979
  formulas (rpm -> ``(A*256+B)/4`` etc.), plus the supported-PID bitmaps
  0x00/0x20/0x40/0x60 that declare exactly those PIDs.  Several PIDs in one
  request (up to 6) produce one concatenated ``41 P1 .. P2 ..`` reply; replies
  longer than 7 bytes go out as ISO-TP multi-frame, waiting for the tester's flow
  control on 0x7E0 (1 s timeout, honours block size and STmin).  Unsupported PIDs
  are skipped (and a request with none supported gets no answer - like a real
  ECU, so the reader learns to time out).
* Mode 09: PID 00 (supported), PID 02 VIN ``SIM0BDDASH0000001`` as multi-frame.
  Mode 03: "no stored DTCs".  Any other mode: negative response ``7F <mode> 11``.

**Broadcaster** - a thread sending, byte-exact to the vehicle yaml:

* ``0x200`` brake (50 Hz): system / front / rear pressure u16 big-endian 0.01 bar,
  byte 6 bit 0 brake switch, bit 1 clutch switch, byte 7 rolling counter;
* ``0x201`` chassis (50 Hz): steering 0.1 deg, yaw 0.01 deg/s, lat / lon 0.001 g,
  all signed 16-bit big-endian;
* ``0x202`` wheel speeds (50 Hz): four u16 at 0.01 km/h;
* ``0x203`` gear (20 Hz): signed int8 in byte 0;
* ``0x0A5`` (100 Hz) and ``0x3C2`` (10 Hz): meaningless counters so ``obddash scan``
  has unknown traffic to show.

The bus object is *not* owned here: the caller opens it (``can.Bus(interface=
"virtual", channel=...)``) and shuts it down.  ``receive_own_messages`` must stay
False or the ECU would see its own broadcasts.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable

import can

from obddash.config import VehicleConfig
from obddash.sim.vehicle import VehicleState

log = logging.getLogger(__name__)

OBD_FUNCTIONAL_ID = 0x7DF
OBD_PHYSICAL_ID = 0x7E0
OBD_RESPONSE_ID = 0x7E8
VIN = "SIM0BDDASH0000001"
PADDING = 0x00

# broadcast ids (see config/vehicles/sim_sportscar.yaml)
ID_BRAKE = 0x200
ID_CHASSIS = 0x201
ID_WHEELS = 0x202
ID_GEAR = 0x203
ID_NOISE_FAST = 0x0A5
ID_NOISE_SLOW = 0x3C2


# --------------------------------------------------------------------------------------
# bit packing helpers (local on purpose: obddash.bus is written concurrently)
# --------------------------------------------------------------------------------------
def pack_be(
    value: float, scale: float, nbits: int, signed: bool = False, offset: float = 0.0
) -> int:
    """Encode a physical value as an ``nbits`` raw integer: ``raw = (value - offset) /
    scale`` rounded and clamped to the field's range; two's complement if ``signed``.
    The result is the unsigned bit pattern ready to be written big-endian."""
    if value is None or value != value:  # None / NaN -> 0
        raw = 0
    else:
        raw = int(round((float(value) - offset) / scale))
    if signed:
        lo, hi = -(1 << (nbits - 1)), (1 << (nbits - 1)) - 1
        raw = max(lo, min(hi, raw))
        return raw & ((1 << nbits) - 1)
    return max(0, min((1 << nbits) - 1, raw))


def put_be(buf: bytearray, byte: int, nbits: int, raw: int) -> None:
    """Write ``raw`` big-endian into ``buf`` starting at ``byte`` (byte-aligned field,
    ``nbits`` a multiple of 8) - the layout the yaml's ``byte:`` shortcut describes."""
    nbytes = nbits // 8
    for i in range(nbytes):
        buf[byte + i] = (raw >> (8 * (nbytes - 1 - i))) & 0xFF


def _u8(x: float) -> bytes:
    return bytes([pack_be(x, 1.0, 8)])


def _u16(x: float) -> bytes:
    return pack_be(x, 1.0, 16).to_bytes(2, "big")


# --------------------------------------------------------------------------------------
# OBD-II mode 01 encoders (inverse of the SAE J1979 decode formulas)
# --------------------------------------------------------------------------------------
REF_TORQUE_NM = 250.0  # PID 0x63 engine reference torque


def _pct(x: float) -> bytes:  # A*100/255
    return _u8(x * 255.0 / 100.0)


def _temp(x: float) -> bytes:  # A-40
    return _u8(x + 40.0)


PID_ENCODERS: dict[int, Callable[[VehicleState], bytes]] = {
    0x04: lambda s: _pct(s.load_pct),
    0x05: lambda s: _temp(s.coolant_c),
    0x0B: lambda s: _u8(s.map_kpa),
    0x0C: lambda s: _u16(s.rpm * 4.0),  # (A*256+B)/4
    0x0D: lambda s: _u8(s.speed_mps * 3.6),
    0x0E: lambda s: _u8((s.timing_adv_deg + 64.0) * 2.0),  # A/2-64
    0x0F: lambda s: _temp(s.intake_c),
    0x10: lambda s: _u16(s.maf_gps * 100.0),  # (A*256+B)/100
    0x11: lambda s: _pct(s.throttle_pct),
    0x2F: lambda s: _pct(s.fuel_pct),
    0x33: lambda s: _u8(s.baro_kpa),
    0x42: lambda s: _u16(s.batt_v * 1000.0),  # (A*256+B)/1000
    # 0x44: equivalence ratio (A*256+B)*2/65536, then sensor voltage (C*256+D)/8192*8
    0x44: lambda s: _u16(s.lambda_ * 32768.0) + _u16(3.3 * 8192.0),
    0x46: lambda s: _temp(22.0),  # ambient
    0x49: lambda s: _pct(s.throttle_pct),  # accelerator pedal D ~ throttle on this car
    0x5C: lambda s: _temp(s.oil_temp_c),
    0x5E: lambda s: _u16(s.fuel_rate_lph * 20.0),  # (A*256+B)/20
    0x62: lambda s: _u8(s.engine_torque_nm / REF_TORQUE_NM * 100.0 + 125.0),  # A-125 %
    0x63: lambda s: _u16(REF_TORQUE_NM),
}

#: response payload length per PID (bytes after the PID byte), masks included
PID_LENGTHS: dict[int, int] = {
    0x00: 4, 0x20: 4, 0x40: 4, 0x60: 4,
    0x04: 1, 0x05: 1, 0x0B: 1, 0x0C: 2, 0x0D: 1, 0x0E: 1, 0x0F: 1, 0x10: 2, 0x11: 1,
    0x2F: 1, 0x33: 1, 0x42: 2, 0x44: 4, 0x46: 1, 0x49: 1, 0x5C: 1, 0x5E: 2, 0x62: 1, 0x63: 2,
}  # fmt: skip


def supported_mask(base: int, supported: set[int]) -> bytes:
    """4-byte bitmap for PID ``base`` (0x00/0x20/...): bit A7 = base+1 ... bit D0 = base+32.
    The next range's mask PID is flagged when anything beyond it is supported."""
    bits = 0
    for pid in range(base + 1, base + 0x21):
        if pid in supported or (pid % 0x20 == 0 and any(p > pid for p in supported)):
            bits |= 1 << (0x20 - (pid - base))
    return bits.to_bytes(4, "big")


def encode_pid(pid: int, state: VehicleState) -> bytes | None:
    """Mode-01 data bytes for ``pid`` (without the PID byte), or None if unsupported."""
    if pid in (0x00, 0x20, 0x40, 0x60):
        return supported_mask(pid, set(PID_ENCODERS))
    enc = PID_ENCODERS.get(pid)
    if enc is None:
        return None
    try:
        return enc(state)
    except Exception:  # a NaN or None in the state must not kill the responder
        log.exception("encoding PID 0x%02X failed", pid)
        return None


# --------------------------------------------------------------------------------------
# broadcast frame builders (pure, testable)
# --------------------------------------------------------------------------------------
def frame_brake(s: VehicleState, counter: int, *, switch_threshold_bar: float = 2.0) -> bytes:
    """0x200: u16 system / front / rear bar @0.01, byte 6 bit0 brake switch, bit1 clutch
    switch, byte 7 counter."""
    buf = bytearray(8)
    put_be(buf, 0, 16, pack_be(s.brake_bar, 0.01, 16))
    put_be(buf, 2, 16, pack_be(s.brake_front_bar, 0.01, 16))
    put_be(buf, 4, 16, pack_be(s.brake_rear_bar, 0.01, 16))
    flags = 0
    if s.brake_bar > switch_threshold_bar:
        flags |= 0x01  # DBC start_bit 48 = byte 6, bit 0
    if s.clutch > 0.5:
        flags |= 0x02  # DBC start_bit 49 = byte 6, bit 1
    buf[6] = flags
    buf[7] = counter & 0xFF
    return bytes(buf)


def frame_chassis(s: VehicleState) -> bytes:
    """0x201: i16 steering @0.1 deg, yaw @0.01 deg/s, lat @0.001 g, lon @0.001 g."""
    buf = bytearray(8)
    put_be(buf, 0, 16, pack_be(s.steering_deg, 0.1, 16, signed=True))
    put_be(buf, 2, 16, pack_be(s.yaw_rate_dps, 0.01, 16, signed=True))
    put_be(buf, 4, 16, pack_be(s.lat_g, 0.001, 16, signed=True))
    put_be(buf, 6, 16, pack_be(s.lon_g, 0.001, 16, signed=True))
    return bytes(buf)


def frame_wheels(s: VehicleState) -> bytes:
    """0x202: four u16 wheel speeds FL FR RL RR @0.01 km/h."""
    buf = bytearray(8)
    for i, w in enumerate(s.wheel_speeds_kph[:4]):
        put_be(buf, 2 * i, 16, pack_be(w, 0.01, 16))
    return bytes(buf)


def frame_gear(s: VehicleState, counter: int) -> bytes:
    """0x203: byte 0 gear as int8 (-1 R, 0 N, 1..6), byte 1 clutch %, byte 7 counter."""
    buf = bytearray(8)
    buf[0] = pack_be(s.gear, 1.0, 8, signed=True)
    buf[1] = pack_be(s.clutch * 100.0, 1.0, 8)
    buf[7] = counter & 0xFF
    return bytes(buf)


# --------------------------------------------------------------------------------------
# ISO-TP helpers
# --------------------------------------------------------------------------------------
def _sf(payload: bytes) -> bytes:
    """Single frame, padded to 8 bytes."""
    return bytes([len(payload)]) + payload + bytes([PADDING]) * (7 - len(payload))


def _stmin_seconds(st: int) -> float:
    if st <= 0x7F:
        return st / 1000.0
    if 0xF1 <= st <= 0xF9:
        return (st - 0xF0) / 10000.0
    return 0.0


class _RxAssembly:
    """State of a multi-frame request being received from the tester."""

    def __init__(self, length: int, first: bytes) -> None:
        self.length = length
        self.data = bytearray(first)
        self.seq = 1
        self.t = time.monotonic()


# --------------------------------------------------------------------------------------
# FakeEcu
# --------------------------------------------------------------------------------------
class FakeEcu:
    """OBD-II responder + signal broadcaster.  See the module docstring."""

    FC_TIMEOUT_S = 1.0
    BASE_TICK_S = 0.01  # broadcast scheduler base period (100 Hz)

    def __init__(
        self,
        vehicle: VehicleConfig,
        bus: can.BusABC,
        state_fn: Callable[[], VehicleState],
        *,
        latency_s: float = 0.004,
        broadcast: bool = True,
        noise_ids: bool = True,
    ) -> None:
        self.vehicle = vehicle
        self.bus = bus
        self.state_fn = state_fn
        self.latency_s = max(0.0, latency_s)
        self.broadcast = broadcast
        self.noise_ids = noise_ids
        self.request_count = 0
        self.response_count = 0
        self.broadcast_count = 0
        self._notifier: can.Notifier | None = None
        self._requests: queue.Queue[bytes] = queue.Queue()
        self._flow_control: queue.Queue[bytes] = queue.Queue()
        self._rx: _RxAssembly | None = None
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._bcast: threading.Thread | None = None
        self._send_lock = threading.Lock()

    # -- lifecycle ---------------------------------------------------------------------------
    def start(self) -> None:
        if self._worker is not None:
            return
        self._stop.clear()
        self._worker = threading.Thread(target=self._worker_loop, name="sim-ecu", daemon=True)
        self._worker.start()
        self._notifier = can.Notifier(self.bus, [self._on_frame], timeout=0.1)
        if self.broadcast:
            self._bcast = threading.Thread(
                target=self._broadcast_loop, name="sim-ecu-bcast", daemon=True
            )
            self._bcast.start()
        log.info("fake ECU up on %s (latency %.1f ms)", self.bus.channel_info, self.latency_s * 1e3)

    def stop(self) -> None:
        self._stop.set()
        if self._notifier is not None:
            try:
                self._notifier.stop(timeout=1.0)
            except Exception:  # bus may already be gone
                log.debug("notifier stop failed", exc_info=True)
            self._notifier = None
        self._requests.put(b"")  # wake the worker
        for th in (self._worker, self._bcast):
            if th is not None and th.is_alive():
                th.join(timeout=2.0)
        self._worker = None
        self._bcast = None

    # -- receive side ------------------------------------------------------------------------
    def _on_frame(self, msg: can.Message) -> None:
        """Notifier callback: route ISO-TP frames addressed to us."""
        if msg.is_extended_id or msg.is_error_frame or msg.is_remote_frame:
            return
        if msg.arbitration_id not in (OBD_FUNCTIONAL_ID, OBD_PHYSICAL_ID):
            return
        data = bytes(msg.data)
        if not data:
            return
        pci = data[0] >> 4
        if pci == 0x0:  # single frame
            n = data[0] & 0x0F
            if 0 < n <= 7:
                self._requests.put(data[1 : 1 + n])
        elif pci == 0x1:  # first frame of a long request: answer with flow control
            if len(data) < 2:
                return
            length = ((data[0] & 0x0F) << 8) | data[1]
            self._rx = _RxAssembly(length, data[2:8])
            self._send(bytes([0x30, 0x00, 0x00]) + bytes([PADDING]) * 5)
        elif pci == 0x2:  # consecutive frame
            rx = self._rx
            if rx is None or time.monotonic() - rx.t > self.FC_TIMEOUT_S:
                self._rx = None
                return
            if (data[0] & 0x0F) != (rx.seq & 0x0F):
                log.debug("ISO-TP sequence error on request, dropping")
                self._rx = None
                return
            rx.seq += 1
            rx.data += data[1:8]
            if len(rx.data) >= rx.length:
                self._requests.put(bytes(rx.data[: rx.length]))
                self._rx = None
        elif pci == 0x3:  # flow control for a reply we are sending
            if msg.arbitration_id == OBD_PHYSICAL_ID:
                self._flow_control.put(data)

    # -- worker ------------------------------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            try:
                req = self._requests.get(timeout=0.2)
            except queue.Empty:
                continue
            if not req or self._stop.is_set():
                continue
            self.request_count += 1
            try:
                if self.latency_s:
                    time.sleep(self.latency_s)
                self._handle(req)
            except Exception:
                log.exception("fake ECU failed on request %s", req.hex())

    def _handle(self, req: bytes) -> None:
        mode = req[0]
        state = self.state_fn()
        if mode == 0x01:
            out = bytearray([0x41])
            for pid in req[1:7]:  # up to 6 PIDs per request (ISO 15031-5)
                data = encode_pid(pid, state)
                if data is not None:
                    out.append(pid)
                    out += data
            if len(out) > 1:
                self._send_isotp(bytes(out))
            # all PIDs unsupported: stay silent, like a real ECU
        elif mode == 0x09:
            pid = req[1] if len(req) > 1 else 0x00
            if pid == 0x00:
                self._send_isotp(bytes([0x49, 0x00]) + supported_mask(0x00, {0x02}))
            elif pid == 0x02:
                self._send_isotp(bytes([0x49, 0x02, 0x01]) + VIN.encode("ascii"))
        elif mode == 0x03:
            self._send_isotp(bytes([0x43, 0x00]))  # no stored DTCs
        else:
            self._send_isotp(bytes([0x7F, mode, 0x11]))  # service not supported

    # -- transmit side -------------------------------------------------------------------------
    def _send(self, data: bytes, arbitration_id: int = OBD_RESPONSE_ID) -> None:
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        with self._send_lock:
            self.bus.send(msg)

    def _send_isotp(self, payload: bytes) -> None:
        """Send ``payload`` as a single frame or as FF + (flow control) + CFs."""
        self.response_count += 1
        if len(payload) <= 7:
            self._send(_sf(payload))
            return
        # drain stale flow-control frames
        while True:
            try:
                self._flow_control.get_nowait()
            except queue.Empty:
                break
        length = len(payload)
        self._send(bytes([0x10 | (length >> 8), length & 0xFF]) + payload[:6])
        pos = 6
        seq = 1
        block_size, stmin = self._await_flow_control()
        if block_size is None:
            return
        sent_in_block = 0
        while pos < length:
            chunk = payload[pos : pos + 7]
            self._send(bytes([0x20 | (seq & 0x0F)]) + chunk + bytes([PADDING]) * (7 - len(chunk)))
            pos += 7
            seq += 1
            sent_in_block += 1
            if pos >= length:
                break
            if block_size and sent_in_block >= block_size:
                block_size, stmin = self._await_flow_control()
                if block_size is None:
                    return
                sent_in_block = 0
            if stmin:
                time.sleep(stmin)

    def _await_flow_control(self) -> tuple[int | None, float]:
        """Wait for the tester's FC.  Returns (block_size, stmin_s); block_size None = abort."""
        deadline = time.monotonic() + self.FC_TIMEOUT_S
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or self._stop.is_set():
                log.debug("no flow control from tester; aborting multi-frame response")
                return None, 0.0
            try:
                fc = self._flow_control.get(timeout=remaining)
            except queue.Empty:
                continue
            status = fc[0] & 0x0F
            if status == 0x0:  # continue to send
                bs = fc[1] if len(fc) > 1 else 0
                st = _stmin_seconds(fc[2]) if len(fc) > 2 else 0.0
                return bs, st
            if status == 0x1:  # wait: tester asks for more time
                deadline = time.monotonic() + self.FC_TIMEOUT_S
                continue
            return None, 0.0  # overflow / abort

    # -- broadcaster -------------------------------------------------------------------------
    def _broadcast_loop(self) -> None:
        tick = 0
        counter = 0
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += self.BASE_TICK_S
            now = time.monotonic()
            if next_t < now - 0.5:  # fell badly behind (suspended?) -> resync, don't burst
                next_t = now
            delay = next_t - now
            if delay > 0:
                time.sleep(delay)
            try:
                s = self.state_fn()
                if tick % 2 == 0:  # 50 Hz
                    thr = self.vehicle.brake.active_threshold_bar
                    self._send(frame_brake(s, counter, switch_threshold_bar=thr), ID_BRAKE)
                    self._send(frame_chassis(s), ID_CHASSIS)
                    self._send(frame_wheels(s), ID_WHEELS)
                    counter += 1
                if tick % 5 == 0:  # 20 Hz
                    self._send(frame_gear(s, counter), ID_GEAR)
                if self.noise_ids:
                    self._send(_noise_fast(tick), ID_NOISE_FAST)  # 100 Hz
                    if tick % 10 == 0:
                        self._send(_noise_slow(tick), ID_NOISE_SLOW)  # 10 Hz
                self.broadcast_count += 1
            except Exception:
                if self._stop.is_set():
                    break
                log.debug("broadcast failed (bus closed?)", exc_info=True)
                time.sleep(0.1)
            tick += 1


def _noise_fast(tick: int) -> bytes:
    """0x0A5: a 16-bit counter, a sawtooth and a checksum - typical unknown traffic."""
    saw = (tick * 7) & 0xFF
    body = bytes([(tick >> 8) & 0xFF, tick & 0xFF, saw, 0x80 | (tick & 0x0F), 0x00, 0x00, 0x3F])
    return body + bytes([sum(body) & 0xFF])


def _noise_slow(tick: int) -> bytes:
    """0x3C2: slowly changing bytes with a few constant ones."""
    n = tick // 10
    return bytes([0x12, n & 0xFF, (n >> 8) & 0xFF, 0xA0, 0x00, 0x00, 0x00, (n * 3) & 0xFF])
