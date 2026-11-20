#!/usr/bin/env python3
"""Read Feetech head registers only; no torque, goal or EEPROM writes."""

import argparse
import json
import time
from pathlib import Path

import serial


def read_register(port, motor: int, address: int, count: int) -> int:
    """Send a protocol-1 READ and validate the complete status reply."""
    payload = bytes([motor, 4, 2, address, count])
    request = b'\xff\xff' + payload + bytes([(~sum(payload)) & 255])
    port.reset_input_buffer()
    port.write(request)
    port.flush()
    deadline = time.monotonic() + 0.3
    buffer = bytearray()
    while time.monotonic() < deadline:
        buffer.extend(port.read(max(1, port.in_waiting)))
        start = buffer.find(b'\xff\xff')
        if start < 0:
            continue
        if start:
            del buffer[:start]
        if len(buffer) < 4:
            continue
        size = buffer[3] + 4
        if len(buffer) < size:
            continue
        packet = bytes(buffer[:size])
        del buffer[:size]
        if packet[2] != motor:
            continue
        if sum(packet[2:]) & 255 != 255:
            raise RuntimeError(f'ID {motor}: bad checksum')
        if packet[4]:
            raise RuntimeError(f'ID {motor}: status error {packet[4]}')
        if len(packet) != count + 6:
            raise RuntimeError(f'ID {motor}: wrong reply length')
        return int.from_bytes(packet[5:-1], 'little')
    raise TimeoutError(f'ID {motor}: no READ response at address {address}')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    registers = {'model': (3, 2), 'mode': (33, 1), 'torque_enable': (40, 1),
                 'position': (56, 2), 'goal': (42, 2), 'speed': (58, 2),
                 'voltage_raw': (62, 1), 'temperature_c': (63, 1), 'status': (65, 1),
                 'min_limit': (9, 2), 'max_limit': (11, 2)}
    report = {'port': args.port, 'baudrate': 1000000, 'read_only': True, 'motors': {}}
    with serial.Serial(args.port, baudrate=1000000, timeout=.02, exclusive=True) as port:
        for motor in (13, 14):
            try:
                report['motors'][motor] = {name: read_register(port, motor, *location)
                                           for name, location in registers.items()}
            except (RuntimeError, TimeoutError) as exc:
                report['motors'][motor] = {'error': str(exc)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
