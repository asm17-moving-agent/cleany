"""Packet and bounded-motion validation without hardware writes."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools/validation'))
from head_pan_tilt_probe import angles, write_ram
from read_head_servos import read_register
import head_pan_tilt_probe as head_module


class Port:
    def __init__(self, reply):
        self.reply = bytearray(reply)
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.reply)

    def reset_input_buffer(self):
        pass

    def write(self, value):
        self.writes.append(value)

    def flush(self):
        pass

    def read(self, size):
        out = bytes(self.reply[:size])
        del self.reply[:size]
        return out


def status(motor, value, error=0):
    body = bytes([motor, 4, error])+value.to_bytes(2, 'little')
    return b'\xff\xff'+body+bytes([~sum(body)&255])


def test_read_packet_is_read_only_and_checked():
    port = Port(status(13, 1995))
    assert read_register(port, 13, 56, 2) == 1995
    assert port.writes[0][4] == 2
    assert sum(port.writes[0][2:])&255 == 255


def test_corrupt_reply_rejected():
    packet = bytearray(status(14, 2132))
    packet[-1] ^= 1
    with pytest.raises(RuntimeError, match='checksum'):
        read_register(Port(packet), 14, 56, 2)


@pytest.mark.parametrize('motor,address,size,value', [(1,42,2,2000), (13,5,1,14), (14,46,2,500), (14,48,2,1000)])
def test_unsafe_writes_never_sent(motor, address, size, value):
    port = Port(b'')
    with pytest.raises(ValueError):
        write_ram(port, motor, address, size, value)
    assert not port.writes


def test_recorded_encoder_direction_and_neutral():
    config = json.loads((ROOT/'tools/validation/config/d435_pan_tilt.json').read_text())
    assert angles([2009, 2067], config) == [0, 0]
    pan, tilt = angles([1995, 2132], config)
    assert pan > 0 and tilt > 0
    assert pan == pytest.approx(14*2*3.141592653589793/4096)


def test_floor_view_failure_disables_tilt(monkeypatch, tmp_path):
    probe = object.__new__(head_module.HeadProbe)
    probe.config = json.loads((ROOT/'tools/validation/config/d435_pan_tilt.json').read_text())
    probe.port = object()
    probe.poll = lambda: [1997, 2134]
    probe.hold_for = lambda _: None
    def fail_move(*_):
        raise RuntimeError('did not settle')
    probe.move = fail_move
    writes = []
    monkeypatch.setattr(head_module, 'write_ram', lambda port, motor, address, size, value: writes.append((motor, address, value)))
    monkeypatch.setattr(head_module, 'read_register', lambda port, motor, address, size: {3:777, 33:0, 40:0, 65:0, 63:30}[address])
    with pytest.raises(RuntimeError, match='settle'):
        probe.floor_view(25, tmp_path)
    assert writes[-1] == (14, 40, 0)
    assert not any(m == 13 for m, _, _ in writes)
    assert writes.index((14, 42, 2134)) < writes.index((14, 40, 1))


def test_floor_view_rejects_out_of_range_before_reading():
    probe = object.__new__(head_module.HeadProbe)
    with pytest.raises(ValueError):
        probe.floor_view(90, Path('/unused'))
