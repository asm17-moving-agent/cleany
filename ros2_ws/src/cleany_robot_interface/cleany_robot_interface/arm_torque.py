"""One-shot current-position arm holds, excluding grippers; RAM writes only."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import subprocess
import termios
from typing import Iterator, Literal, Protocol

import yaml


JOINTS = (
    'left_shoulder_yaw_joint', 'left_shoulder_pitch_joint', 'left_elbow_pitch_joint',
    'left_wrist_pitch_joint', 'left_wrist_roll_joint',
)
IDS = tuple(range(1, 6))


def arm_selection(side: Literal['left', 'right']) -> tuple[tuple[int, ...], tuple[str, ...]]:
    if side not in ('left', 'right'):
        raise ValueError('Select left or right arm')
    first = 1 if side == 'left' else 7
    return tuple(range(first, first + 5)), tuple(side + j.removeprefix('left') for j in JOINTS)


class Registers(Protocol):
    def read(self, motor: int, address: int, size: int) -> int: ...
    def write(self, motor: int, address: int, size: int, value: int) -> None: ...


@dataclass(frozen=True)
class Snapshot:
    position: int
    powered: bool


def enable(bus: Registers, profile: dict, *, side: Literal['left', 'right'] = 'left') -> None:
    ids, joints = arm_selection(side)
    snapshots: dict[int, Snapshot] = {}
    settings: dict[int, dict] = {}
    # Complete every preflight before the first write.
    for motor, joint in zip(ids, joints, strict=True):
        values = {**profile['joint_defaults'], **profile.get('joint_overrides', {}).get(joint, {})}
        for key, upper in (('torque_limit_raw', 1000), ('speed_raw', 32767), ('acceleration_raw', 254)):
            if not isinstance(values[key], int) or not 1 <= values[key] <= upper:
                raise ValueError(f'{joint}: invalid {key}')
        if bus.read(motor, 3, 2) != (2825 if (motor - 1) % 6 in (1, 2) else 777):
            raise RuntimeError(f'ID {motor}: unexpected model')
        if bus.read(motor, 33, 1) != 0:
            raise RuntimeError(f'ID {motor}: position mode required')
        position = bus.read(motor, 56, 2)
        low, high = bus.read(motor, 9, 2), bus.read(motor, 11, 2)
        if not 0 <= low <= position <= high <= 4095:
            raise RuntimeError(f'ID {motor}: position outside encoder limits')
        temperature_limit = min(values.get('temperature_limit_c', profile['temperature_limit_c']),
                                bus.read(motor, 13, 1))
        if bus.read(motor, 65, 1) != 0 or bus.read(motor, 63, 1) >= temperature_limit:
            raise RuntimeError(f'ID {motor}: fault or temperature limit reached')
        torque = bus.read(motor, 40, 1)
        if torque not in (0, 1):
            raise RuntimeError(f'ID {motor}: unexpected torque mode {torque}')
        if torque and abs(bus.read(motor, 42, 2) - position) > 3:
            raise RuntimeError(f'ID {motor}: enabled goal is distant from current position')
        if values['torque_limit_raw'] > bus.read(motor, 16, 2):
            raise RuntimeError(f'ID {motor}: requested output exceeds EEPROM maximum')
        snapshots[motor] = Snapshot(position, bool(torque))
        settings[motor] = values
    for motor, state in snapshots.items():
        if state.powered:
            continue  # An already-held joint is adopted without rewriting its goal.
        values = settings[motor]
        for address, size, value in (
            (41, 1, values['acceleration_raw']), (44, 2, 0),
            (46, 2, values['speed_raw']), (48, 2, values['torque_limit_raw']),
            (42, 2, state.position),
        ):
            bus.write(motor, address, size, value)
            if bus.read(motor, address, size) != value:
                raise RuntimeError(f'ID {motor}: register {address} verification failed')
    for motor, state in snapshots.items():
        if abs(bus.read(motor, 56, 2) - state.position) > 3:
            raise RuntimeError(f'ID {motor}: arm moved before torque ON')
    for motor, state in snapshots.items():
        if not state.powered:
            if abs(bus.read(motor, 56, 2) - state.position) > 3:
                raise RuntimeError(f'ID {motor}: arm moved before torque ON')
            bus.write(motor, 40, 1, 1)
        if bus.read(motor, 40, 1) != 1:
            raise RuntimeError(f'ID {motor}: torque ON not confirmed')
        print(f'ID {motor}: position={bus.read(motor, 56, 2)} '
              f'goal={bus.read(motor, 42, 2)} torque=ON', flush=True)


def disable(bus: Registers, *, side: Literal['left', 'right'] = 'left') -> list[str]:
    failures = []
    for motor in arm_selection(side)[0]:
        try:
            bus.write(motor, 40, 1, 0)
            if bus.read(motor, 40, 1) != 0:
                raise RuntimeError('torque OFF not confirmed')
            print(f'ID {motor}: torque=OFF verified', flush=True)
        except (RuntimeError, OSError) as error:
            failures.append(f'ID {motor}: {error}')
    return failures


def report_state(bus: Registers, *, side: Literal['left', 'right'] = 'left') -> None:
    for motor in arm_selection(side)[0]:
        try:
            print(f'ID {motor}: torque={bus.read(motor, 40, 1)} '
                  f'position={bus.read(motor, 56, 2)} goal={bus.read(motor, 42, 2)}')
        except (RuntimeError, OSError) as error:
            print(f'ID {motor}: state UNKNOWN ({error})')


def check_port_unused(device: str) -> None:
    resolved = str(Path(device).resolve(strict=True))
    result = subprocess.run(['fuser', resolved],
                            capture_output=True, text=True, timeout=5)
    diagnostics = result.stderr.replace(resolved + ':', '').strip()
    if result.returncode not in (0, 1) or diagnostics:
        raise RuntimeError(f'Cannot verify serial ownership: {result.stderr.strip()}')
    owners = {int(pid) for pid in result.stdout.split()}
    if owners - {os.getpid()}:
        raise RuntimeError(f'Serial port is in use by PIDs {sorted(owners - {os.getpid()})}')


@contextmanager
def connect(device: str, baudrate: int) -> Iterator[Registers]:
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

    check_port_unused(device)
    port = PortHandler(device)
    try:
        if not port.setBaudRate(baudrate):
            raise RuntimeError(f'Cannot open {device}')
        fcntl.ioctl(port.ser.fileno(), termios.TIOCEXCL)
        # Detect another owner that opened between the first check and TIOCEXCL.
        check_port_unused(device)
        packet = PacketHandler(0)

        class SDKRegisters:
            def read(self, motor: int, address: int, size: int) -> int:
                method = packet.read1ByteTxRx if size == 1 else packet.read2ByteTxRx
                value, communication, error = method(port, motor, address)
                if communication != COMM_SUCCESS or error:
                    raise RuntimeError(f'read {address}: comm={communication} error={error}')
                return value

            def write(self, motor: int, address: int, size: int, value: int) -> None:
                method = packet.write1ByteTxRx if size == 1 else packet.write2ByteTxRx
                communication, error = method(port, motor, address, value)
                if communication != COMM_SUCCESS or error:
                    raise RuntimeError(f'write {address}: comm={communication} error={error}')

        yield SDKRegisters()
    finally:
        if port.is_open:
            # Clear exclusion even if another process has a pre-existing fd.
            try:
                fcntl.ioctl(port.ser.fileno(), termios.TIOCNXCL)
            finally:
                port.closePort()


def main(operation: str, root: Path, *, side: Literal['left', 'right'] = 'left') -> int:
    ids, _ = arm_selection(side)
    parser = argparse.ArgumentParser(description=f'{side.title()} IDs {ids[0]}-{ids[-1]} torque {operation.upper()}')
    parser.add_argument('--bus-config', type=Path, default=root / 'ros2_ws/src/cleany_robot_interface/config/arm_preview.yaml')
    if operation == 'on':
        parser.add_argument('--control-config', type=Path, default=root / f'configs/robot/feetech_operational_{side}_arm.yaml')
    args = parser.parse_args()
    try:
        config = yaml.safe_load(args.bus_config.read_text())
        profile = yaml.safe_load(args.control_config.read_text()) if operation == 'on' else None
        with connect(config['port'], int(config['baudrate'])) as bus:
            if operation == 'on':
                try:
                    enable(bus, profile, side=side)
                except (Exception, KeyboardInterrupt):
                    print('ON incomplete; torque is preserved. Support the arm before OFF.')
                    report_state(bus, side=side)
                    raise
                print(f'{side.upper()} ARM HOLD ON VERIFIED; torque remains ON after this command exits.')
            else:
                failures = disable(bus, side=side)
                if failures:
                    for failure in failures:
                        print(f'ERROR: {failure}')
                    return 1
                print(f'{side.upper()} ARM TORQUE OFF VERIFIED FOR IDS {ids[0]}-{ids[-1]}')
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(f'ERROR: {error}')
        return 1
