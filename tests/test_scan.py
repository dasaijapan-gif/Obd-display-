"""obddash.bus.scan: BusScanner statistics, change marking, diff mode and run_scan."""

from __future__ import annotations

import io
import threading
import time
import uuid

import can
import pytest

from obddash.bus.scan import BusScanner, run_scan


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def _frame(hexstr: str) -> bytes:
    return bytes.fromhex(hexstr.replace(" ", ""))


def test_counts_last_bytes_and_change_tracking():
    clock = Clock()
    sc = BusScanner(clock=clock)
    sc.feed(0x200, _frame("00 00 00 00 00 00 00 00"))
    clock.t = 0.1
    sc.feed(0x200, _frame("00 00 00 00 00 00 00 00"))
    clock.t = 0.2
    sc.feed(0x200, _frame("04 D2 00 00 00 00 00 00"))  # bytes 0 and 1 change
    clock.t = 0.3
    sc.feed(0x201, _frame("FF 88 00"), extended=False)
    st = sc.ids[0x200]
    assert st.count == 3
    assert st.last == _frame("04 D2 00 00 00 00 00 00")
    assert st.byte_changes == [1, 1, 0, 0, 0, 0, 0, 0]
    assert st.changed_mask(0.3) == 0b11
    assert st.changed_mask(2.3) == 0  # older than 2 s
    assert sc.ids[0x201].count == 1 and sc.ids[0x201].byte_changes == [0] * 8
    assert sc.frames == 4
    assert sc.ids[0x200].label() == "200"


def test_hz_per_id_over_one_second_window():
    clock = Clock()
    sc = BusScanner(clock=clock)
    for i in range(50):
        clock.t = i * 0.02
        sc.feed(0x200, bytes(8))
    for i in range(10):
        clock.t = i * 0.1
        sc.feed(0x300, bytes(8))
    clock.t = 1.0
    rows = sc.snapshot()
    hz = {r.arbitration_id: r.hz for r in rows}
    assert hz[0x200] == pytest.approx(50.0, rel=0.05)
    assert hz[0x300] == pytest.approx(10.0, rel=0.1)
    clock.t = 4.0
    assert all(r.hz == 0.0 for r in sc.snapshot())  # silence -> 0


def test_render_table_marks_changed_bytes_and_respects_width():
    clock = Clock()
    sc = BusScanner(clock=clock)
    sc.feed(0x200, _frame("00 00 00 00 00 00 00 00"))
    clock.t = 0.5
    sc.feed(0x200, _frame("00 2A 00 00 00 00 00 00"))
    sc.feed(0x18DAF110, _frame("01 02"), extended=True)
    text = sc.render_table(width=100, now=0.6)
    lines = text.splitlines()
    assert "2 ids" in lines[0]
    row = next(line for line in lines if line.lstrip().startswith("200"))
    assert "[2A]" in row  # changed within 2 s -> marked
    assert " 00 " in row  # unchanged bytes are not marked
    assert "0/1/0/0/0/0/0/0" in row
    ext_row = next(line for line in lines if "18DAF110" in line)
    assert "  . " in ext_row  # missing bytes shown as dots
    assert all(len(line) <= 100 for line in lines)
    narrow = sc.render_table(width=40, now=0.6)
    assert all(len(line) <= 40 for line in narrow.splitlines())
    # after 2 s the mark goes away
    assert "[2A]" not in sc.render_table(now=3.0)


def test_diff_mode_finds_the_byte_that_moved():
    clock = Clock()
    sc = BusScanner(clock=clock)
    sc.begin_baseline()
    # parked: id 0x200 byte 0 (brake) stays 0, byte 7 is a rolling counter, id 0x300 constant
    for i in range(20):
        clock.t = i * 0.02
        sc.feed(0x200, bytes([0, 0, 0, 0, 0, 0, 0, i & 0x0F]))
        sc.feed(0x300, _frame("11 22 33 44 55 66 77 88"))
    sc.end_baseline()
    assert sc.has_baseline
    assert sc.changes_since_baseline() == []
    # press the brake: byte 0/1 of 0x200 move; counter keeps cycling; a new id appears
    for i in range(20, 40):
        clock.t = i * 0.02
        sc.feed(0x200, bytes([0x04, 0xD2 if i % 2 else 0xD3, 0, 0, 0, 0, 0, i & 0x0F]))
        sc.feed(0x300, _frame("11 22 33 44 55 66 77 88"))
    sc.feed(0x400, _frame("AA BB"))
    changes = sc.changes_since_baseline()
    keys = [(c.arbitration_id, c.byte_index) for c in changes]
    assert (0x200, 7) not in keys  # counter values were all seen in the baseline
    assert all(k[0] != 0x300 for k in keys)  # constant frame is quiet
    assert keys[:2] == [(0x200, 0), (0x200, 1)]  # quiet-then-moved bytes rank first
    b0 = changes[0]
    assert (
        b0.baseline_values == {0}
        and b0.new_values == {0x04}
        and b0.count == 20
        and b0.last_value == 4
    )
    assert changes[1].new_values == {0xD2, 0xD3}
    new_id = [c for c in changes if c.arbitration_id == 0x400]
    assert len(new_id) == 2 and all(c.new_id for c in new_id)
    text = sc.render_diff(width=100)
    assert "200" in text.splitlines()[1] and "-> 04" in text
    assert "new id" in text
    # a new baseline resets the diff
    sc.begin_baseline()
    sc.end_baseline()
    assert sc.changes_since_baseline() == []


def test_render_diff_without_baseline():
    sc = BusScanner(clock=Clock())
    assert "no baseline" in sc.render_diff()


def test_run_scan_on_virtual_bus_prints_table_and_diff():
    channel = f"obddash-scan-{uuid.uuid4().hex}"
    tx = can.Bus(interface="virtual", channel=channel)
    rx = can.Bus(interface="virtual", channel=channel)
    stop = threading.Event()
    phase = {"brake": False}

    def sender():
        i = 0
        while not stop.is_set():
            brake = bytes([0x04, 0xD2]) if phase["brake"] else bytes(2)
            tx.send(
                can.Message(
                    arbitration_id=0x200,
                    data=brake + bytes(5) + bytes([i & 0xFF]),
                    is_extended_id=False,
                )
            )
            tx.send(can.Message(arbitration_id=0x201, data=bytes(8), is_extended_id=False))
            i += 1
            time.sleep(0.01)

    t = threading.Thread(target=sender, daemon=True)
    t.start()
    out = io.StringIO()
    try:
        timer = threading.Timer(0.45, lambda: phase.__setitem__("brake", True))
        timer.start()
        scanner = run_scan(
            rx, duration=0.9, interval=0.2, diff=True, out=out, baseline_s=0.3, width=90
        )
        timer.cancel()
    finally:
        stop.set()
        t.join(timeout=1.0)
        tx.shutdown()
        rx.shutdown()
    text = out.getvalue()
    assert "Recording baseline" in text
    assert "ids," in text and " 200 " in text and " 201 " in text
    assert "moved since baseline" in text
    assert scanner.ids[0x200].count > 10
    changes = scanner.changes_since_baseline()
    assert [(c.arbitration_id, c.byte_index) for c in changes[:2]] == [(0x200, 0), (0x200, 1)]
    assert all(c.byte_index != 7 or c.arbitration_id != 0x200 for c in changes[:2])
    assert scanner._notifier is None  # stopped cleanly


def test_run_scan_without_diff_and_non_tty_output():
    channel = f"obddash-scan-{uuid.uuid4().hex}"
    tx = can.Bus(interface="virtual", channel=channel)
    rx = can.Bus(interface="virtual", channel=channel)
    out = io.StringIO()
    try:
        tx.send(can.Message(arbitration_id=0x123, data=bytes(8), is_extended_id=False))
        scanner = run_scan(rx, duration=0.3, interval=0.1, out=out)
    finally:
        tx.shutdown()
        rx.shutdown()
    assert "\x1b[2J" not in out.getvalue()  # no escape codes when not a tty
    assert "123" in out.getvalue()
    assert scanner.ids[0x123].count == 1
