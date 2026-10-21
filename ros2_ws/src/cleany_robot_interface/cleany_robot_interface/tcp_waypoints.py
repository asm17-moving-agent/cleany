"""Frozen teaching datasets and separate encoder/board TCP repeatability results.

This module never opens a device or approves a trajectory.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transform(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], (0, 0, 0, 1), atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not math.isclose(np.linalg.det(matrix[:3, :3]), 1, abs_tol=1e-6)):
        raise ValueError('Expected a finite rigid transform')
    return matrix


def board_tcp(camera_T_board, gripper_T_camera, gripper_T_tcp) -> np.ndarray:
    """Independent of joint FK: B_T_TCP = inv(C_T_B) inv(G_T_C) G_T_TCP."""
    return (np.linalg.inv(transform(camera_T_board))
            @ np.linalg.inv(transform(gripper_T_camera)) @ transform(gripper_T_tcp))


def pose_error(reference, observed) -> dict:
    reference, observed = transform(reference), transform(observed)
    delta = (observed[:3, 3] - reference[:3, 3]) * 1000
    cosine = (np.trace(reference[:3, :3].T @ observed[:3, :3]) - 1) / 2
    return {'delta_xyz_mm': delta.tolist(), 'position_mm': float(np.linalg.norm(delta)),
            'orientation_deg': math.degrees(math.acos(float(np.clip(cosine, -1, 1))))}


def sequence(text: str) -> tuple[int, ...]:
    try:
        values = tuple(int(item.strip()) for item in text.split(','))
    except (ValueError, TypeError) as error:
        raise ValueError('Sequence must contain comma-separated positive point numbers') from error
    if not values or any(item < 1 for item in values):
        raise ValueError('Point numbers must be positive')
    return values


@dataclass(frozen=True)
class Dataset:
    root: Path
    manifest: dict

    @classmethod
    def load(cls, root: Path) -> Dataset:
        root = root.expanduser().resolve(strict=True)
        manifest = json.loads((root / 'session.json').read_text())
        if manifest.get('schema') != 1 or manifest.get('side') not in ('left', 'right'):
            raise ValueError('Invalid TCP dataset manifest')
        result = cls(root, manifest)
        result.verify()
        return result

    def verify(self) -> None:
        for name, expected in self.manifest['frozen_sha256'].items():
            path = (self.root / name).resolve(strict=True)
            if not path.is_relative_to(self.root) or digest(path) != expected:
                raise ValueError('Frozen input changed: ' + name)
        for name in ('gripper_T_camera', 'gripper_T_tcp'):
            transform(self.manifest[name])

    @property
    def side(self) -> str:
        return self.manifest['side']

    @property
    def command_joints(self) -> tuple[str, ...]:
        return tuple(self.manifest['command_joints'])

    def register_closed_gripper(self, observation: dict) -> None:
        """Freeze an operator-confirmed physical closure before teaching any points."""
        self.verify()
        if any(directory.exists() and any(directory.iterdir()) for directory in
               (self.root / 'points', self.root / 'runs')):
            raise ValueError('Register the closed gripper before the first point/run; use a new session otherwise')
        if not observation.get('operator_confirmed_closed') or not observation.get('stationary_checks_passed'):
            raise ValueError('Physical closure confirmation and stationary feedback required')
        name = self.side + '_gripper_joint'
        value = observation['positions_rad'][name]
        mapping = next(row for row in json.loads((self.root / 'mapping.json').read_text()) if row['name'] == name)
        raw = observation['joints'][name]['raw_tick']
        mapped = (raw - mapping['zero_tick']) * mapping['direction'] * math.tau / 4096 + mapping['offset_rad']
        limit = ET.parse(self.root / 'profile/robot_control.urdf').find(f"joint[@name='{name}']/limit")
        if (not math.isfinite(value) or not math.isfinite(raw)
                or observation['joints'][name]['id'] != mapping['motor_id']
                or not mapping['range_min'] <= raw <= mapping['range_max']
                or not float(limit.get('lower')) <= value <= float(limit.get('upper'))
                or not math.isclose(value, mapped, abs_tol=1e-9)):
            raise ValueError('Closed reference must match valid calibrated gripper feedback')
        updated = {**self.manifest,
            'cad_closed_gripper_joint_rad': self.manifest.get('cad_closed_gripper_joint_rad', self.manifest['closed_gripper_joint_rad']),
            'closed_gripper_joint_rad': value,
            'gripper_closed_reference': {**observation, 'kind': 'operator_confirmed_physical_closure',
                'model_sha256': self.manifest['model_sha256'], 'motor_calibration_changed': False,
                'tcp_contact_calibrated': False}}
        temporary = self.root / '.session_gripper_reference.json'
        try:
            temporary.write_text(json.dumps(updated, indent=2, allow_nan=False) + '\n')
            temporary.replace(self.root / 'session.json')
        finally:
            temporary.unlink(missing_ok=True)
        self.manifest.update(updated)

    def point(self, number: int) -> dict:
        if number < 1:
            raise ValueError('Point number must be positive')
        directory = self.root / 'points' / str(number)
        return self._read_point(directory, number)

    def _read_point(self, directory: Path, number: int) -> dict:
        record = json.loads((directory / 'point.json').read_text())
        if (record.get('side') != self.side or record.get('number') != number
                or record.get('model_sha256') != self.manifest['model_sha256']
                or record.get('session_sha256') != digest(self.root / 'session.json')
                or record.get('image_sha256') != digest(directory / 'rgb.png')):
            raise ValueError('Point identity/model/image mismatch')
        for key in ('base_T_tcp', 'board_T_tcp'):
            transform(record[key])
        names = self.manifest['state_joints']
        values = record['positions_rad']
        if set(values) != set(names) or not all(math.isfinite(values[n]) for n in names):
            raise ValueError('Incomplete or nonfinite taught joint feedback')
        if not record.get('stationary_checks_passed') or not record.get('torque_hold_verified'):
            raise ValueError('Point lacks stationary powered-hold evidence')
        return record

    def commit(self, number: int, record: dict, image_bytes: bytes) -> Path:
        if number < 1:
            raise ValueError('Point number must be positive')
        parent = self.root / 'points'
        parent.mkdir(exist_ok=True)
        final = parent / str(number)
        if final.exists():
            raise FileExistsError(f'Point {number} already exists; create a new session to re-teach')
        temporary = Path(tempfile.mkdtemp(prefix='.', dir=parent))
        try:
            (temporary / 'rgb.png').write_bytes(image_bytes)
            data = {**record, 'number': number, 'side': self.side,
                    'model_sha256': self.manifest['model_sha256'],
                    'session_sha256': digest(self.root / 'session.json'),
                    'image_sha256': digest(temporary / 'rgb.png')}
            (temporary / 'point.json').write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
            # Validate the complete record/image pair before making it visible.
            self._read_point(temporary, number)
            temporary.rename(final)
        except BaseException:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
        return final


def compare_point(point: dict, measurement: dict) -> dict:
    """A missing visual measurement is an error, never a zero-error fallback."""
    if not measurement.get('stationary_checks_passed'):
        raise ValueError('Arrival lacks stationary evidence')
    return {'encoder_tcp': pose_error(point['base_T_tcp'], measurement['base_T_tcp']),
            'board_video_tcp': pose_error(point['board_T_tcp'], measurement['board_T_tcp']),
            'absolute_tcp_accuracy_verified': False}


def validate_control_contract(dataset: Dataset) -> None:
    """Reject a session that could command any motor outside its five-joint arm."""
    suffixes = ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll')
    selected = tuple(f'{dataset.side}_{suffix}_joint' for suffix in suffixes)
    expected = {f'{side}_{suffix}_joint' for side in ('left', 'right')
                for suffix in (*suffixes, 'gripper')}
    expected.update(('head_pan_joint', 'head_tilt_joint'))
    tree = ET.parse(dataset.root / 'profile/robot_control.urdf')
    control = tree.find('ros2_control')
    if control is None:
        raise ValueError('Frozen model has no native control system')
    elements = control.findall('joint')
    commands = tuple(j.get('name') for j in elements if j.find('command_interface') is not None)
    hardware = control.find('hardware')
    if (tuple(dataset.command_joints) != selected or commands != selected
            or len(elements) != 14 or {j.get('name') for j in elements} != expected
            or set(dataset.manifest['state_joints']) != expected
            or hardware.find('plugin').text != 'cleany_feetech_hardware/FeetechSystem'
            or hardware.find("param[@name='read_only']").text != 'false'):
        raise ValueError('TCP controller must use the native selected five-joint arm only')
    for key in ('arm_port', 'head_port'):
        if hardware.find(f"param[@name='{key}']").text != dataset.manifest[key]:
            raise ValueError('Serial port differs from frozen control model')
