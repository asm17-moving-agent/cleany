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
from cleany_robot_interface.rigid_accessories import attach_rigid_accessories


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
    camera_pose_config: Path | None = None


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
    mapping_path, head_mapping = read(config['head_joint_mapping'])
    calibration = None
    if not config.get('camera_pose_config'):
        calibration = None
    if not config.get('camera_pose_config'):
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
    head_bounds = head_mapping.get('joint_position_bounds', {})
    if (not isinstance(head_bounds, dict)
            or not set(head_bounds) <= {'head_pan_joint', 'head_tilt_joint'}):
        raise ValueError('Head joint position bounds must name head pan or tilt joints')
    calibrated_bounds.update(head_bounds)
    model_path = resolve(config['robot_model']).resolve()
    sources.add(model_path)
    cad_report = None
    if config.get('model_variant') == 'freecad_calibrated':
        from cleany_robot_interface.calibrated_camera_pose import CameraPoseModel
        from cleany_robot_interface.calibrated_cad_model import model_from_binding, calibrated_cad_model
        pose_path, _ = read(config['camera_pose_config'])
        CameraPoseModel(pose_path)
        binding_path, _ = read(config['cad_binding_file'])
        binding = json.loads(binding_path.read_text())
        if binding.get('gripper_endpoint_file'):
            endpoint_path, _ = read(binding['gripper_endpoint_file'])
            if hashlib.sha256(endpoint_path.read_bytes()).hexdigest() != binding['gripper_endpoint_sha256']:
                raise ValueError('CAD gripper endpoint configuration changed; regenerate the binding')
        source, limits = model_from_binding(model_path, binding, pose_path, calibrated_bounds)
        xml, cad_report = calibrated_cad_model(source, binding['joint_coordinates'], limits,
            binding['cad_metadata'], -float(config['grasp_tcp_y_m']))
        command = []
        tree = ET.fromstring(xml)
    else:
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
        if calibration is not None:
            xml = apply_head_rgb(xml, calibration)
        if calibrated_bounds:
            xml = apply_joint_bounds(xml, calibrated_bounds)
        geometry_files = config.get('rigid_accessory_files', [])
        if not isinstance(geometry_files, list) or any(
                not isinstance(path, str) or not path for path in geometry_files):
            raise ValueError('rigid_accessory_files must be a list of configuration paths')
        for geometry_file in geometry_files:
            _, registration = read(geometry_file)
            source_path = resolve(registration['source_model']).resolve()
            sources.add(source_path)
            xml = attach_rigid_accessories(xml, source_path.read_text(), registration)
        tree = ET.fromstring(xml)
        if 'grasp_tcp_y_m' in config:
            y = config['grasp_tcp_y_m']
            if type(y) not in (int, float) or not math.isfinite(y) or y >= 0:
                raise ValueError('grasp_tcp_y_m must be a finite negative number in metres')
            for side in ('left', 'right'):
                tcp = f'{side}_grasp_tcp'
                joint = tree.find(f"joint[@name='{tcp}_joint']")
                if (joint is None or joint.get('type') != 'fixed'
                        or joint.find('parent').get('link') != f'{side}_gripper_frame'
                        or joint.find('child').get('link') != tcp):
                    raise ValueError(f'Grasp TCP frame contract mismatch: {tcp}')
                origin = joint.find('origin')
                if origin is None:
                    raise ValueError(f'Missing grasp TCP origin: {tcp}')
                origin.set('xyz', f'0 {y} 0')
            xml = ET.tostring(tree, encoding='unicode')
    camera_pose_config = None
    if config.get('camera_pose_config'):
        from cleany_robot_interface.calibrated_camera_pose import CameraPoseModel, remove_camera_frames
        camera_pose_config, _ = read(config['camera_pose_config'])
        camera_model = CameraPoseModel(camera_pose_config)
        frames = {frame for camera in camera_model.config['cameras'].values()
                  for frame in camera['optical_frames']}
        xml = remove_camera_frames(xml, frames)
        tree = ET.fromstring(xml)
    from cleany_description.cad_coordinates import CadCoordinates
    cad = CadCoordinates.from_urdf(xml)
    dependent = set(cad.dependent_names) if cad else set()
    movable = {joint.get('name') for joint in tree.findall('joint')
               if joint.get('type') != 'fixed' and joint.find('mimic') is None
               and joint.get('name') not in dependent}
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
        'model_variant': config.get('model_variant', 'legacy'), 'cad_reuse': cad_report,
        'xacro_command': command, 'movable_joints': sorted(movable),
        'calibrated_joint_position_bounds': calibrated_bounds,
        'trajectory_execution_enabled': False, 'absolute_calibration_verified': False,
    }
    (output / 'manifest.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return StateProfile(destination, arm_path, head_path, mapping_path,
                        str(head['camera_serial']), camera_pose_config)


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
