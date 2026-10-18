"""DBC-style decoding of vehicle broadcast CAN frames.

Cars broadcast hundreds of frames per second that standard OBD-II never exposes (brake
pressure, steering angle, wheel speeds, ...).  A :class:`~obddash.config.SignalConfig`
describes where one such value sits inside a frame using the DBC conventions that
``cantools`` and every DBC editor use:

* **little_endian (Intel)**: ``start_bit`` is the position of the signal's least
  significant bit, bits numbered ``byte*8 + bit`` (bit 0 = LSB of byte 0, bit 8 = LSB
  of byte 1).  The signal grows upward through increasing bit numbers.
* **big_endian (Motorola)**: ``start_bit`` is the position of the signal's MOST
  significant bit in the same numbering (bit 7 = MSB of byte 0).  The signal grows
  *downward* inside the byte, then continues at the MSB (bit 7) of the next byte:
  a 16-bit signal at ``start_bit 7`` occupies byte 0 (MSB) and byte 1 (LSB); at
  ``start_bit 23`` bytes 2-3.

Both are implemented by turning the frame into one big integer: for Intel the frame
read little-endian puts bit ``n`` of the frame at bit ``n`` of the integer, so the
signal is a plain shift+mask.  For Motorola the frame read big-endian is a bit stream
from the MSB of byte 0 downward, and the start bit maps to a position in that stream.

:class:`SignalDecoder` compiles a list of signals into per-id decode plans, applies
scale/offset, range filtering and the optional byte multiplexer, and returns
``{Sig name: physical value}`` ready for ``RawState.update``.  It is called from the
python-can notifier thread for every passive frame, so it avoids allocation beyond the
result dict.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

from obddash.config import SignalConfig

log = logging.getLogger(__name__)

LITTLE = "little_endian"
BIG = "big_endian"


# --------------------------------------------------------------------------------------
# Bit-level primitives
# --------------------------------------------------------------------------------------
def _be_stream_pos(start_bit: int) -> int:
    """Map a Motorola start bit (MSB position in sawtooth numbering) to its index in the
    big-endian bit stream (0 = MSB of byte 0, 7 = LSB of byte 0, 8 = MSB of byte 1...)."""
    return (start_bit // 8) * 8 + (7 - start_bit % 8)


def _check(start_bit: int, length: int) -> None:
    if start_bit < 0:
        raise ValueError("start_bit must be >= 0")
    if not 1 <= length <= 64:
        raise ValueError("length must be 1..64 bits")


def _end_bit(start_bit: int, length: int, byte_order: str) -> int:
    """One past the highest frame-stream index the signal touches (for bounds checks)."""
    if byte_order == LITTLE:
        return start_bit + length
    return _be_stream_pos(start_bit) + length


def extract_bits(data: bytes, start_bit: int, length: int, byte_order: str, signed: bool) -> int:
    """Return the raw integer of a signal with exact DBC semantics (see module doc).

    Raises ``ValueError`` when the signal does not fit in ``data``.
    """
    _check(start_bit, length)
    nbits = len(data) * 8
    mask = (1 << length) - 1
    if byte_order == LITTLE:
        if start_bit + length > nbits:
            raise ValueError(f"signal (start {start_bit}, len {length}) exceeds {nbits}-bit frame")
        raw = (int.from_bytes(data, "little") >> start_bit) & mask
    elif byte_order == BIG:
        shift = nbits - (_be_stream_pos(start_bit) + length)
        if shift < 0:
            raise ValueError(f"signal (start {start_bit}, len {length}) exceeds {nbits}-bit frame")
        raw = (int.from_bytes(data, "big") >> shift) & mask
    else:
        raise ValueError(f"unknown byte_order {byte_order!r}")
    if signed and raw & (1 << (length - 1)):
        raw -= 1 << length
    return raw


def pack_bits(
    data: bytes | bytearray,
    value: int,
    start_bit: int,
    length: int,
    byte_order: str,
    signed: bool = False,
) -> bytearray:
    """Inverse of :func:`extract_bits`: return a copy of ``data`` with the field set.

    The value must fit the field (two's complement range when ``signed``).
    """
    _check(start_bit, length)
    value = int(value)
    if signed:
        lo, hi = -(1 << (length - 1)), (1 << (length - 1)) - 1
    else:
        lo, hi = 0, (1 << length) - 1
    if not lo <= value <= hi:
        raise ValueError(
            f"value {value} does not fit a {length}-bit {'signed' if signed else 'unsigned'} field"
        )
    nbits = len(data) * 8
    mask = (1 << length) - 1
    raw = value & mask
    if byte_order == LITTLE:
        if start_bit + length > nbits:
            raise ValueError("field exceeds frame")
        n = int.from_bytes(data, "little")
        n = (n & ~(mask << start_bit)) | (raw << start_bit)
        return bytearray(n.to_bytes(len(data), "little"))
    if byte_order == BIG:
        shift = nbits - (_be_stream_pos(start_bit) + length)
        if shift < 0:
            raise ValueError("field exceeds frame")
        n = int.from_bytes(data, "big")
        n = (n & ~(mask << shift)) | (raw << shift)
        return bytearray(n.to_bytes(len(data), "big"))
    raise ValueError(f"unknown byte_order {byte_order!r}")


# --------------------------------------------------------------------------------------
# Signal-level helpers
# --------------------------------------------------------------------------------------
def decode_signal(cfg: SignalConfig, data: bytes) -> float | None:
    """Physical value of one signal from one frame, or None (mux mismatch, out of range,
    frame too short)."""
    if cfg.mux_byte is not None:
        if cfg.mux_byte >= len(data) or data[cfg.mux_byte] != cfg.mux_value:
            return None
    try:
        raw = extract_bits(data, cfg.start_bit, cfg.length, cfg.byte_order, cfg.signed)
    except ValueError:
        return None
    value = raw * cfg.scale + cfg.offset
    if cfg.min is not None and value < cfg.min:
        return None
    if cfg.max is not None and value > cfg.max:
        return None
    return value


def encode_signal(cfg: SignalConfig, value: float, data: bytes | bytearray) -> bytearray:
    """Write a physical ``value`` into a copy of ``data`` (rounds to the nearest raw step).
    Sets the mux byte too when the signal has one.  Mostly for simulators and tests."""
    raw = round((value - cfg.offset) / cfg.scale) if cfg.scale else 0
    out = pack_bits(data, raw, cfg.start_bit, cfg.length, cfg.byte_order, cfg.signed)
    if cfg.mux_byte is not None and cfg.mux_value is not None:
        out[cfg.mux_byte] = cfg.mux_value & 0xFF
    return out


# --------------------------------------------------------------------------------------
# Decoder
# --------------------------------------------------------------------------------------
@dataclass(slots=True)
class _Plan:
    """A SignalConfig pre-digested for the hot path."""

    name: str
    little: bool
    start_bit: int  # Intel: LSB index
    stream_end: int  # Motorola: stream index one past the LSB; Intel: start+length
    length: int
    mask: int
    sign_bit: int  # 0 when unsigned
    modulo: int
    scale: float
    offset: float
    lo: float | None
    hi: float | None
    mux_byte: int | None
    mux_value: int | None


def _compile(cfg: SignalConfig) -> _Plan:
    start = cfg.start_bit if cfg.start_bit is not None else 0
    little = cfg.byte_order == LITTLE
    return _Plan(
        name=cfg.name,
        little=little,
        start_bit=start,
        stream_end=_end_bit(start, cfg.length, cfg.byte_order),
        length=cfg.length,
        mask=(1 << cfg.length) - 1,
        sign_bit=(1 << (cfg.length - 1)) if cfg.signed else 0,
        modulo=1 << cfg.length,
        scale=cfg.scale,
        offset=cfg.offset,
        lo=cfg.min,
        hi=cfg.max,
        mux_byte=cfg.mux_byte,
        mux_value=cfg.mux_value,
    )


class SignalDecoder:
    """Decode configured broadcast signals from raw frames.

    >>> dec = SignalDecoder(vehicle.signals)
    >>> if msg.arbitration_id in dec.ids:
    ...     raw.update(dec.decode(msg.arbitration_id, msg.data))
    """

    def __init__(self, signals: list[SignalConfig]):
        self._by_id: dict[int, list[_Plan]] = {}
        self.signals: list[SignalConfig] = list(signals)
        for cfg in signals:
            self._by_id.setdefault(cfg.id, []).append(_compile(cfg))
        self.ids: set[int] = set(self._by_id)
        self.names: list[str] = [s.name for s in signals]

    def decode(self, arbitration_id: int, data: bytes) -> dict[str, float]:
        plans = self._by_id.get(arbitration_id)
        if not plans:
            return {}
        nbits = len(data) * 8
        big = little = None
        out: dict[str, float] = {}
        for p in plans:
            if p.mux_byte is not None:
                if p.mux_byte >= len(data) or data[p.mux_byte] != p.mux_value:
                    continue
            if p.stream_end > nbits:
                continue  # short frame (DLC < expected): skip rather than misread
            if p.little:
                if little is None:
                    little = int.from_bytes(data, "little")
                raw = (little >> p.start_bit) & p.mask
            else:
                if big is None:
                    big = int.from_bytes(data, "big")
                raw = (big >> (nbits - p.stream_end)) & p.mask
            if p.sign_bit and raw & p.sign_bit:
                raw -= p.modulo
            value = raw * p.scale + p.offset
            if p.lo is not None and value < p.lo:
                continue
            if p.hi is not None and value > p.hi:
                continue
            out[p.name] = value
        return out

    def __len__(self) -> int:
        return len(self.signals)


# --------------------------------------------------------------------------------------
# DBC import (optional, needs cantools)
# --------------------------------------------------------------------------------------
def load_dbc(path: str | Path, name_map: dict[str, str]) -> list[SignalConfig]:
    """Turn the signals of a ``.dbc`` file into :class:`SignalConfig` entries.

    ``name_map`` maps DBC signal names (``"SignalName"`` or ``"MessageName.SignalName"``)
    to canonical ``Sig`` names; signals not in the map are ignored.  Multiplexed signals
    are supported when the multiplexer is a whole byte (our ``mux_byte`` model);
    other multiplexers are skipped with a warning.
    """
    try:
        import cantools  # type: ignore[import-not-found]
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(
            "Loading .dbc files needs the optional 'cantools' package: "
            "pip install cantools  (or pip install 'obddash[dbc]')"
        ) from exc

    db = cantools.database.load_file(str(path))
    out: list[SignalConfig] = []
    for msg in db.messages:
        mux_signals = {s.name: s for s in msg.signals if s.is_multiplexer}
        for sig in msg.signals:
            target = name_map.get(f"{msg.name}.{sig.name}") or name_map.get(sig.name)
            if target is None:
                continue
            mux_byte: int | None = None
            mux_values: list[int | None] = [None]
            if sig.multiplexer_ids:
                mux = mux_signals.get(sig.multiplexer_signal or "")
                mux_byte = _whole_byte_index(mux) if mux is not None else None
                if mux_byte is None:
                    log.warning(
                        "dbc %s.%s: multiplexer is not a whole byte, skipping", msg.name, sig.name
                    )
                    continue
                # one entry per selector value; the decoder only fires the matching one
                mux_values = [int(v) for v in sig.multiplexer_ids]
            lo, hi = sig.minimum, sig.maximum
            if lo is not None and hi is not None and lo == hi == 0:
                lo = hi = None  # DBC "[0|0]" means "no range" (older cantools pass it through)
            for mux_value in mux_values:
                out.append(
                    SignalConfig(
                        name=target,
                        id=msg.frame_id,
                        extended=bool(msg.is_extended_frame),
                        start_bit=sig.start,
                        length=sig.length,
                        byte_order=BIG if sig.byte_order == "big_endian" else LITTLE,
                        signed=bool(sig.is_signed),
                        scale=float(sig.scale),
                        offset=float(sig.offset),
                        min=None if lo is None else float(lo),
                        max=None if hi is None else float(hi),
                        unit=sig.unit or "",
                        mux_byte=mux_byte,
                        mux_value=mux_value,
                    )
                )
    log.info("dbc %s: mapped %d of %d requested signals", path, len(out), len(name_map))
    return out


def _whole_byte_index(sig) -> int | None:
    """Byte index of an 8-bit, byte-aligned cantools signal, else None.

    Byte-aligned means start bit 0 of a byte for Intel (LSB first) but bit 7 for Motorola
    (MSB first); an Intel field at bit 7 or a Motorola one at bit 0 straddles two bytes.
    """
    if sig.length != 8:
        return None
    aligned_bit = 7 if sig.byte_order == "big_endian" else 0
    if sig.start % 8 != aligned_bit:
        return None
    return sig.start // 8
