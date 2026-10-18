"""obddash.bus: CAN bus layer (OBD-II/ISO-TP protocol, DBC-style signals, live reader, scan)."""

from obddash.bus.obd import (
    PID_TABLE,
    IsoTpAssembler,
    PidDef,
    build_flow_control,
    build_request,
    decode_supported,
    parse_mode01,
    parse_mode09_vin,
)
from obddash.bus.reader import CanReader, ReaderStats
from obddash.bus.scan import BusScanner, run_scan
from obddash.bus.signals import SignalDecoder, extract_bits, load_dbc, pack_bits

__all__ = [
    "PID_TABLE",
    "BusScanner",
    "CanReader",
    "IsoTpAssembler",
    "PidDef",
    "ReaderStats",
    "SignalDecoder",
    "build_flow_control",
    "build_request",
    "decode_supported",
    "extract_bits",
    "load_dbc",
    "pack_bits",
    "parse_mode01",
    "parse_mode09_vin",
    "run_scan",
]
