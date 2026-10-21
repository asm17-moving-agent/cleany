"""Materialize the existing model with a Feetech ros2_control system.

No device opens or motor commands occur during profile preparation.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from cleany_robot_interface.core import mappings
from cleany_robot_interface.joint_bounds import apply_joint_bounds, calibrated_joint_bounds
from cleany_robot_interface.state_profile import StateProfile, head_positions, prepare_state_profile


@dataclass(frozen=True)
class ControlProfile:
    state: StateProfile
    model: Path
    controllers: Path
    moveit_controllers: Path
    joint_limits: Path
    controller_names: tuple[str, ...]


def resolve_moveit_trajectory_execution(read_only: bool, requested: str) -> bool:
    """Resolve MoveIt's execution permission independently from motor writes."""
    if not isinstance(read_only, bool):
        raise ValueError('read_only must be a boolean')
    if requested == 'auto':
        return not read_only
    if requested not in ('true', 'false'):
        raise ValueError('allow_trajectory_execution must be auto, true, or false')
    enabled = requested == 'true'
    if read_only and enabled:
        raise ValueError('MoveIt trajectory execution cannot be enabled in read-only mode')
    return enabled


def add_control_model(model: str, arm: dict, calibrations: dict, head: dict,
                      head_mapping: dict, limits: dict, read_only: bool) -> tuple[str, dict, dict]:
    root = ET.fromstring(model)
    if root.find('ros2_control') is not None:
        raise ValueError('Model already contains a hardware backend')
    if (int(arm['baudrate']) != 1_000_000 or int(head['baudrate']) != 1_000_000
            or head['motor_ids'] != {'pan': 13, 'tilt': 14}):
        raise ValueError('Expected 1Mbps arm/head buses and head IDs 13/14')
    head_positions([0, 0], head_mapping)
    if not math.isclose(head_mapping['encoder_scale_rad_per_tick'], math.tau/4096):
        raise ValueError('Expected 4096-tick head scale')
    if not isinstance(read_only, bool):
        raise ValueError('read_only must be a boolean')
    rate = limits['update_rate']
    if not isinstance(rate, int) or not 1 <= rate <= 100:
        raise ValueError('Invalid controller update_rate')
    read_retries = limits.get('serial_read_retries', 0)
    read_budget = limits.get('serial_read_recovery_budget_ms', 60)
    if (type(read_retries) is not int or not 0 <= read_retries <= 1
            or type(read_budget) is not int or not 1 <= read_budget <= 100
            or (read_retries > 0 and read_budget > limits['max_state_age_s']*1000)):
        raise ValueError('Invalid bounded serial READ recovery settings')
    defaults = limits['joint_defaults']
    mapped = mappings(arm, calibrations)
    overrides = limits.get('joint_overrides', {})
    if (not isinstance(overrides, dict) or not set(overrides) <= {j.name for j in mapped}
            or any(not isinstance(v, dict) or not set(v) <= {*defaults, 'temperature_limit_c'}
                   for v in overrides.values())):
        raise ValueError('Invalid per-joint control overrides')
    selected = limits.get('command_joints', [j.name for j in mapped])
    if (not isinstance(selected, list) or not selected or len(selected) != len(set(selected))
            or not set(selected) <= {j.name for j in mapped}):
        raise ValueError('command_joints must be a nonempty unique subset of arm/gripper joints')
    settings = limits['controller']
    for key in ('max_velocity', 'max_acceleration', 'speed_raw',
                'acceleration_raw', 'torque_limit_raw'):
        if not math.isfinite(defaults[key]) or defaults[key] <= 0:
            raise ValueError('Invalid joint limit: ' + key)
    if not math.isfinite(defaults['tracking_error']) or defaults['tracking_error'] < 0:
        raise ValueError('Invalid joint limit: tracking_error')
    if (not math.isfinite(settings['trajectory_tolerance'])
            or settings['trajectory_tolerance'] < 0):
        raise ValueError('Invalid controller tolerance: trajectory_tolerance')
    for key in ('goal_tolerance', 'goal_time'):
        if not math.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError('Invalid controller tolerance: ' + key)
    if (settings['trajectory_tolerance'] > 0 and defaults['tracking_error'] > 0
            and settings['trajectory_tolerance'] >= defaults['tracking_error']):
        raise ValueError('Controller trajectory tolerance must be below hardware fault threshold')
    bounds = dict(limits.get('joint_position_bounds', {}))
    if not set(bounds) <= {joint.name for joint in mapped}:
        raise ValueError('Physical joint position bounds must name calibrated arm joints')
    derived = calibrated_joint_bounds(mapped, limits.get('calibrated_joint_position_bounds', []))
    if set(bounds) & set(derived):
        raise ValueError('Joint position bounds have both explicit and calibrated sources')
    bounds.update(derived)
    root = ET.fromstring(apply_joint_bounds(model, bounds))
    control = ET.SubElement(root, 'ros2_control', name='FeetechSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'cleany_feetech_hardware/FeetechSystem'

    def param(element: ET.Element, key: str, value: object) -> None:
        ET.SubElement(element, 'param', name=key).text = str(value)

    for key, value in {'arm_port': arm['port'], 'head_port': head['serial_port'],
                       'read_only': str(read_only).lower(),
                       'allow_torque_on_recovery': str(bool(
                           limits.get('allow_torque_on_recovery', False))).lower(),
                       'serial_read_retries': limits.get('serial_read_retries', 0),
                       'serial_read_recovery_budget_ms': limits.get('serial_read_recovery_budget_ms', 60),
                       **{k: limits[k] for k in (
                           'serial_timeout_ms', 'max_state_age_s',
                           'temperature_limit_c',
                           'temperature_fault_confirmation_samples',
                       )}}.items():
        param(hardware, key, value)
    for joint in mapped:
        original = root.find(f"joint[@name='{joint.name}']")
        if original is None or original.find('limit') is None:
            raise ValueError('Missing model joint/limit: ' + joint.name)
        bound = original.find('limit')
        element = ET.SubElement(control, 'joint', name=joint.name)
        if joint.name in selected:
            ET.SubElement(element, 'command_interface', name='position')
        ET.SubElement(element, 'state_interface', name='position')
        if joint.motor_id <= 12:
            for name in ('load_raw', 'torque_enabled', 'motor_status', 'temperature',
                         'voltage_raw', 'goal_position_raw', 'command_position',
                         'sent_position', 'sent_goal_raw', 'position_raw', 'current_raw',
                         'load_magnitude_raw', 'moving_flag', 'protocol_error_code',
                         'torque_limit_readback_raw', 'current_limit_raw', 'temperature_raw',
                         'motor_status_raw', 'feedback_stamp_ns', 'communication_recovery_count'):
                ET.SubElement(element, 'state_interface', name=name)
        values = {key: getattr(joint, key) for key in ('motor_id', 'model', 'zero_tick', 'direction',
                                                     'offset_rad', 'homing_offset', 'range_min', 'range_max')}
        joint_values = {**defaults, **overrides.get(joint.name, {})}
        if (not all(math.isfinite(v) for v in joint_values.values())
                or not 1 <= joint_values['torque_limit_raw'] <= 1000
                or not 1 <= joint_values.get('temperature_limit_c', limits['temperature_limit_c'])
                <= limits['temperature_limit_c']):
            raise ValueError('Invalid per-joint register limits')
        values.update(lower=float(bound.get('lower')), upper=float(bound.get('upper')), **joint_values)
        for key, value in values.items():
            param(element, key, value)
    for index, name in enumerate(('head_pan_joint', 'head_tilt_joint')):
        if root.find(f"joint[@name='{name}']") is None:
            raise ValueError('Missing moving head joint')
        element = ET.SubElement(control, 'joint', name=name)
        ET.SubElement(element, 'state_interface', name='position')
        for key, value in {'motor_id': 13+index, 'model': 777,
                           'zero_tick': head_mapping['reference_ticks'][index],
                           'direction': head_mapping['encoder_directions'][index], 'offset_rad': 0}.items():
            param(element, key, value)

    manager = {'update_rate': rate,
               'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}
    controllers = {'controller_manager': {'ros__parameters': manager},
                   'joint_state_broadcaster': {'ros__parameters': {'use_local_topics': True}}}
    moveit = {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
              'moveit_simple_controller_manager': {'controller_names': []}}
    for side in ('left', 'right'):
        for group, suffixes in (('arm', ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
                                        'wrist_pitch', 'wrist_roll')), ('gripper', ('gripper',))):
            name = f'{side}_{group}_controller'
            joints = [f'{side}_{suffix}_joint' for suffix in suffixes]
            joints = [j for j in joints if j in selected]
            if not joints:
                continue
            manager[name] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
            controllers[name] = {'ros__parameters': {
                'joints': joints, 'command_interfaces': ['position'], 'state_interfaces': ['position'],
                'state_publish_rate': float(rate), 'action_monitor_rate': 20.0,
                'open_loop_control': False, 'allow_partial_joints_goal': False,
                'allow_nonzero_velocity_at_trajectory_end': False,
                'constraints': {'goal_time': settings['goal_time'], 'stopped_velocity_tolerance': 0.0,
                                **{j: {'trajectory': settings['trajectory_tolerance'],
                                       'goal': settings['goal_tolerance']} for j in joints}}}}
            moveit['moveit_simple_controller_manager']['controller_names'].append(name)
            moveit['moveit_simple_controller_manager'][name] = {
                'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                'default': True, 'joints': joints}
    return ET.tostring(root, encoding='unicode'), controllers, moveit


def prepare_control_profile(profile_file: Path, repository_root: Path, output: Path,
                            control_file: Path, read_only: bool = True) -> ControlProfile:
    state = prepare_state_profile(profile_file, repository_root, output)
    arm = yaml.safe_load(state.arm_config.read_text())
    directory = Path(arm['calibration_dir']).expanduser()
    cal_files = [directory / arm['arms'][side]['calibration'] for side in ('left', 'right')]
    calibrations = {side: json.loads(path.read_text()) for side, path in zip(('left', 'right'), cal_files)}
    model, controllers, moveit = add_control_model(
        state.model.read_text(), arm, calibrations, yaml.safe_load(state.head_config.read_text()),
        json.loads(state.head_mapping.read_text()), yaml.safe_load(control_file.read_text()), read_only)
    destination = output / 'robot_control.urdf'
    destination.write_text(model)
    controllers_file = output / 'controllers.yaml'
    controllers_file.write_text(yaml.safe_dump(controllers, sort_keys=False))
    moveit_file = output / 'moveit_controllers.yaml'
    moveit_file.write_text(yaml.safe_dump(moveit, sort_keys=False))
    limits = yaml.safe_load(control_file.read_text())['joint_defaults']
    joint_limits = {'joint_limits': {joint.name: {
        'has_velocity_limits': True, 'max_velocity': limits['max_velocity'],
        'has_acceleration_limits': True, 'max_acceleration': limits['max_acceleration']}
        for joint in mappings(arm, calibrations)}}
    limits_file = output / 'joint_limits.yaml'
    limits_file.write_text(yaml.safe_dump(joint_limits, sort_keys=False))
    (output / 'control_manifest.json').write_text(json.dumps({
        'read_only': read_only, 'physical_motion_validated': False,
        'command_joints': [j.get('name') for j in ET.fromstring(model).findall('ros2_control/joint')
                           if j.find('command_interface') is not None],
        'model_sha256': hashlib.sha256(model.encode()).hexdigest(),
        'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in [control_file, *cal_files]},
        'head_commands': False, 'controller_names': moveit['moveit_simple_controller_manager']['controller_names'],
    }, indent=2) + '\n')
    return ControlProfile(state, destination, controllers_file, moveit_file, limits_file,
                          tuple(moveit['moveit_simple_controller_manager']['controller_names']))
