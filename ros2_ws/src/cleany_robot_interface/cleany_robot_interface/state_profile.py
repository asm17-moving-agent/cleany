"""One materialized model for live read-only TF and MoveIt planning."""
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

from cleany_robot_interface.core import mappings
from cleany_robot_interface.joint_bounds import apply_joint_bounds, calibrated_joint_bounds


def apply_head_rgb(model: str, calibration: dict) -> str:
    root = ET.fromstring(model)
    for key in ('xyz', 'rpy'):
        values = calibration[key]
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError('Invalid head RGB ' + key)
    joint = root.find("joint[@name='head_camera_rgb_optical_joint']")
    if (joint is None or joint.find('parent').get('link') != calibration['parent_frame']
            or joint.find('child').get('link') != calibration['child_frame']):
        raise ValueError('Head RGB frame contract mismatch')
    joint.find('origin').attrib.update({
        key: ' '.join(format(v, '.17g') for v in calibration[key]) for key in ('xyz', 'rpy')})
    return ET.tostring(root, encoding='unicode')


@dataclass(frozen=True)
class StateProfile:
    model: Path
    arm_config: Path
    head_config: Path
    head_mapping: Path
    camera_serial: str


def prepare_state_profile(profile_file: Path, repository_root: Path, output: Path) -> StateProfile:
    """Read the existing camera profile without opening serial/video devices."""
    config = json.loads(profile_file.read_text())
    sources = {profile_file.resolve()}

    def resolve(value: str) -> Path:
        path = Path(value).expanduser()
        return path if path.is_absolute() else repository_root / path

    def read(value: str) -> tuple[Path, dict]:
        path = resolve(value).resolve()
        sources.add(path)
        return path, yaml.safe_load(path.read_text())

    arm_path, arm = read(config['arm_config'])
    head_path, head = read(config['head_config'])
    mapping_path, _ = read(config['head_joint_mapping'])
    _, calibration = read(config['head_rgb_extrinsics'])
    calibrated_bounds = {}
    selected = config.get('calibrated_joint_position_bounds', [])
    if not isinstance(selected, list):
        raise ValueError('calibrated_joint_position_bounds must be a list')
    if selected:
        directory = Path(arm['calibration_dir']).expanduser()
        calibrations = {}
        for side, settings in arm['arms'].items():
            _, values = read(str(directory / settings['calibration']))
            calibrations[side] = values
        calibrated_bounds = calibrated_joint_bounds(mappings(arm, calibrations), selected)
    model_path = resolve(config['robot_model']).resolve()
    sources.add(model_path)
    args = dict(config['xacro_args'])
    if args.get('head_fixed') != 'false' or args.get('include_wheel_joints') != 'false':
        raise ValueError('Live read-only state requires a moving head model and fixed wheels')
    args['include_head_camera'] = 'true'
    for key, value in list(args.items()):
        if key.endswith('_file') and value:
            path, _ = read(value)
            args[key] = str(path)
    command = ['xacro', str(model_path), *(f'{key}:={value}' for key, value in args.items())]
    xml = subprocess.check_output(command, text=True, cwd=repository_root)
    xml = apply_head_rgb(xml, calibration)
    if calibrated_bounds:
        xml = apply_joint_bounds(xml, calibrated_bounds)
    tree = ET.fromstring(xml)
    movable = {joint.get('name') for joint in tree.findall('joint')
               if joint.get('type') != 'fixed'}
    expected = {f'{side}_{suffix}_joint' for side in ('left', 'right')
                for suffix in ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
                               'wrist_pitch', 'wrist_roll', 'gripper')}
    expected.update(('head_pan_joint', 'head_tilt_joint'))
    if movable != expected:
        raise ValueError(f'Model/state joint mismatch: {movable ^ expected}')
    for origin in tree.iter('origin'):
        for key in ('xyz', 'rpy'):
            values = [float(v) for v in origin.get(key, '0 0 0').split()]
            if len(values) != 3 or not all(math.isfinite(v) for v in values):
                raise ValueError('Invalid URDF origin')
    output.mkdir(parents=True, exist_ok=False)
    destination = output / 'robot.urdf'
    destination.write_text(xml)
    receipt = {
        'created_at': datetime.now().astimezone().isoformat(),
        'status': 'read_only_planning_with_existing_provisional_calibration',
        'model_sha256': hashlib.sha256(xml.encode()).hexdigest(),
        'profile_dependencies_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in sorted(sources)},
        'xacro_command': command, 'movable_joints': sorted(movable),
        'calibrated_joint_position_bounds': calibrated_bounds,
        'trajectory_execution_enabled': False, 'absolute_calibration_verified': False,
    }
    (output / 'manifest.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return StateProfile(destination, arm_path, head_path, mapping_path, str(head['camera_serial']))


def head_positions(raw: list[int], mapping: dict) -> list[float]:
    zeros, directions = mapping['reference_ticks'], mapping['encoder_directions']
    scale = float(mapping['encoder_scale_rad_per_tick'])
    if (len(raw) != 2 or len(zeros) != 2 or len(directions) != 2
            or not math.isfinite(scale) or scale <= 0
            or any(d not in (-1, 1) for d in directions)
            or any(not math.isfinite(z) or not 0 <= z <= 4095 for z in zeros)
            or any(not 0 <= value <= 4095 for value in raw)):
        raise ValueError('Invalid head position/mapping')
    return [(value - zero) * direction * scale
            for value, zero, direction in zip(raw, zeros, directions)]
