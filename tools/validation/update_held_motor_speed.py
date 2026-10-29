"""Update only arm speed RAM while an idle, disconnected controller holds torque."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from cleany_robot_interface.arm_torque import Registers, connect


def snapshot(bus: Registers, motor: int) -> dict[str, int]:
    fields = {'model': (3, 2), 'mode': (33, 1), 'torque': (40, 1),
              'acceleration': (41, 1), 'goal': (42, 2), 'time': (44, 2),
              'speed': (46, 2), 'torque_limit': (48, 2), 'position': (56, 2),
              'velocity': (58, 2), 'temperature': (63, 1), 'status': (65, 1),
              'temperature_limit': (13, 1)}
    return {name: bus.read(motor, address, size) for name, (address, size) in fields.items()}


def update(bus: Registers, *, previous: int, requested: int) -> dict:
    """Preflight all 12 held motors, verify speed-only writes, roll back on error.

    The caller must stop command owners and exclusively acquire the serial port.
    Values are raw encoder steps/s. Neither goals, torque nor EEPROM are written.
    """
    if not all(type(value) is int and 1 <= value <= 32767 for value in (previous, requested)):
        raise ValueError('Speed must be a positive 15-bit integer')
    before = {motor: snapshot(bus, motor) for motor in range(1, 13)}
    for motor, state in before.items():
        if state['model'] != (2825 if (motor - 1) % 6 in (1, 2) else 777):
            raise RuntimeError(f'ID {motor}: unexpected model')
        if state['mode'] != 0 or state['torque'] != 1 or state['time'] != 0:
            raise RuntimeError(f'ID {motor}: powered position hold required')
        if state['speed'] != previous or state['status'] != 0:
            raise RuntimeError(f'ID {motor}: speed or status mismatch')
        if state['temperature'] >= min(80, state['temperature_limit']):
            raise RuntimeError(f'ID {motor}: temperature limit')
        if abs(state['goal'] - state['position']) > 32 or (state['velocity'] & 0x7fff) > 2:
            raise RuntimeError(f'ID {motor}: motor is not holding still')
    written = []
    try:
        for motor in before:
            # Include an uncertain acknowledgement in rollback.
            written.append(motor)
            bus.write(motor, 46, 2, requested)
            if bus.read(motor, 46, 2) != requested:
                raise RuntimeError(f'ID {motor}: speed readback mismatch')
        after = {motor: snapshot(bus, motor) for motor in before}
        for motor, state in after.items():
            if state['speed'] != requested or state['status'] != 0:
                raise RuntimeError(f'ID {motor}: final readback mismatch')
            for field in ('goal', 'torque', 'acceleration', 'torque_limit', 'mode', 'time'):
                if state[field] != before[motor][field]:
                    raise RuntimeError(f'ID {motor}: {field} changed')
            if abs(state['position'] - before[motor]['position']) > 3:
                raise RuntimeError(f'ID {motor}: hold moved')
    except Exception as error:
        failures = []
        for motor in written:
            try:
                bus.write(motor, 46, 2, previous)
                if bus.read(motor, 46, 2) != previous:
                    raise RuntimeError('rollback readback mismatch')
            except Exception as rollback_error:
                failures.append(f'{motor}: {rollback_error}')
        raise RuntimeError(f'{error}; rollback failures={failures}') from error
    return {'before': before, 'after': after, 'write_address': 46,
            'previous': previous, 'requested': requested}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', required=True)
    parser.add_argument('--baudrate', type=int, default=1000000)
    parser.add_argument('--previous', type=int, required=True)
    parser.add_argument('--requested', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with connect(args.device, args.baudrate) as bus:
        result = update(bus, previous=args.previous, requested=args.requested)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(f'Updated and verified speed RAM on {len(result["after"])} held motors')


if __name__ == '__main__':
    main()
