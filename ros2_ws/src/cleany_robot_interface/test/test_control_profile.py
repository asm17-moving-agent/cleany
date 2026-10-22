import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from cleany_robot_interface.control_profile import (
    add_control_model,
    prepare_control_profile,
    resolve_moveit_trajectory_execution,
)
from cleany_robot_interface.core import mappings
from cleany_robot_interface.joint_bounds import calibrated_joint_bounds

ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture
def state_profile_file(tmp_path):
    config = json.loads((ROOT / 'configs/robot/camera_preview.json').read_text())
    arm = yaml.safe_load((ROOT / config['arm_config']).read_text())
    arm['port'] = '/dev/cleany-synthetic-test-only'
    arm['calibration_dir'] = str(tmp_path)
    frozen = ROOT / 'ros2_ws/src/cleany_description/config/calibration/jetson_20261009/motors'
    for side in ('left', 'right'):
        filename = arm['arms'][side]['calibration']
        (tmp_path / filename).write_bytes((frozen / filename).read_bytes())
    arm_path = tmp_path / 'synthetic_arm.yaml'
    arm_path.write_text(yaml.safe_dump(arm))
    config['arm_config'] = str(arm_path)
    profile = tmp_path / 'synthetic_state.json'
    profile.write_text(json.dumps(config))
    return profile


@pytest.fixture
def prepared(tmp_path, state_profile_file):
    return prepare_control_profile(state_profile_file, ROOT, tmp_path/'run',
                                   ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml')


def test_default_has_read_only_head_and_twelve_arm_position_commands(prepared):
    root = ET.parse(prepared.model).getroot()
    hardware = root.find('ros2_control/hardware')
    assert hardware.find("param[@name='read_only']").text == 'true'
    joints = root.findall('ros2_control/joint')
    assert len(joints) == 14
    assert sum(len(j.findall('command_interface')) for j in joints) == 12
    assert all(j.find('command_interface') is None for j in joints if j.get('name').startswith('head_'))
    state_model = ET.parse(prepared.state.model).getroot()
    # Physical bounds are the only permitted change to the state model.
    limits = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    for name, values in limits.get('joint_position_bounds', {}).items():
        actual = root.find(f"joint[@name='{name}']/limit")
        assert [float(actual.get(key)) for key in ('lower', 'upper')] == values
        original = state_model.find(f"joint[@name='{name}']/limit")
        for key in ('lower', 'upper'):
            actual.set(key, original.get(key))
    root.remove(root.find('ros2_control'))
    assert ET.tostring(root) == ET.tostring(state_model)


def test_physical_profile_selects_optimized_arm_collision_meshes(prepared):
    from collections import Counter

    settings = json.loads((ROOT / 'configs/robot/arm_collision_meshes.json').read_text())
    expected = Counter({
        f'package://cleany_description/meshes/collision/{name}': 2
        for name in settings['source_meshes']
    })
    for path in (prepared.state.model, prepared.model):
        root = ET.parse(path).getroot()
        actual = Counter(mesh.get('filename')
                         for mesh in root.findall('./link/collision/geometry/mesh')
                         if '/meshes/collision/' in mesh.get('filename'))
        assert actual == expected
        assert not any('/meshes/collision/' in mesh.get('filename')
                       for mesh in root.findall('./link/visual/geometry/mesh'))
        for side in ('left', 'right'):
            collisions = root.findall(f"./link[@name='{side}_gripper_frame']/collision")
            assert sum('wrist_mount_v8' in c.get('name', '') for c in collisions) == 15
    receipt = json.loads((prepared.state.model.parent / 'manifest.json').read_text())
    assert 'optimized_arm_collision_meshes:=true' in receipt['xacro_command']


def test_motor_writes_and_moveit_execution_permissions_are_independent():
    assert resolve_moveit_trajectory_execution(True, 'auto') is False
    assert resolve_moveit_trajectory_execution(False, 'auto') is True
    assert resolve_moveit_trajectory_execution(False, 'false') is False
    with pytest.raises(ValueError, match='read-only'):
        resolve_moveit_trajectory_execution(True, 'true')
    with pytest.raises(ValueError, match='auto, true, or false'):
        resolve_moveit_trajectory_execution(False, 'invalid')


def test_four_controllers_match_moveit_and_use_actual_position_feedback(prepared):
    config = yaml.safe_load(prepared.controllers.read_text())
    moveit = yaml.safe_load(prepared.moveit_controllers.read_text())['moveit_simple_controller_manager']
    assert set(prepared.controller_names) == {f'{side}_{part}_controller'
                                             for side in ('left', 'right') for part in ('arm', 'gripper')}
    all_joints = []
    for name in prepared.controller_names:
        params = config[name]['ros__parameters']
        assert params['joints'] == moveit[name]['joints']
        assert params['state_interfaces'] == ['position']
        assert params['open_loop_control'] is False
        assert params['allow_partial_joints_goal'] is False
        assert params['constraints']['goal_time'] > 0
        assert all(params['constraints'][joint]['trajectory'] == 0.0
                   for joint in params['joints'])
        all_joints += params['joints']
    assert len(all_joints) == len(set(all_joints)) == 12
    limits = yaml.safe_load(prepared.joint_limits.read_text())['joint_limits']
    assert set(limits) == set(all_joints)
    hardware = ET.parse(prepared.model).getroot()
    for name, item in limits.items():
        joint = hardware.find(f"ros2_control/joint[@name='{name}']")
        assert item['has_acceleration_limits'] is True
        for key in ('max_velocity', 'max_acceleration'):
            assert item[key] == float(joint.find(f"param[@name='{key}']").text)


def test_existing_mapping_and_eeprom_values_preserved(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    root = ET.parse(prepared.model).getroot()
    left = root.find("ros2_control/joint[@name='left_gripper_joint']")
    assert float(left.find("param[@name='zero_tick']").text) == arm['overrides']['left']['gripper']['zero_tick']
    assert left.find("param[@name='motor_id']").text == '6'
    manifest = json.loads((prepared.model.parent/'control_manifest.json').read_text())
    assert manifest['read_only'] and not manifest['physical_motion_validated']


def test_calibrated_pitch_bounds_match_state_model_driver_and_recorded_positions(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    directory = Path(arm['calibration_dir']).expanduser()
    calibrations = {side: json.loads((directory/settings['calibration']).read_text())
                    for side, settings in arm['arms'].items()}
    joints = {j.motor_id: j for j in mappings(arm, calibrations)}
    control = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    selected = control['calibrated_joint_position_bounds']
    bounds = calibrated_joint_bounds(list(joints.values()), selected)
    state = ET.parse(prepared.state.model).getroot()
    model = ET.parse(prepared.model).getroot()
    for name, expected in bounds.items():
        for tree in (state, model):
            limit = tree.find(f"joint[@name='{name}']/limit")
            assert tuple(float(limit.get(key)) for key in ('lower', 'upper')) == expected
        driver = model.find(f"ros2_control/joint[@name='{name}']")
        assert tuple(float(driver.find(f"param[@name='{key}']").text)
                     for key in ('lower', 'upper')) == expected
    # These real readings previously failed the nominal CAD limit check.
    for motor, raw in ((2, 863), (3, 3096), (4, 825), (8, 858), (9, 3100)):
        joint = joints[motor]
        low, high = bounds[joint.name]
        assert low <= joint.radians(raw) <= high
        assert not low <= joint.radians(joint.range_min-1) <= high
        assert not low <= joint.radians(joint.range_max+1) <= high
    receipt = json.loads((prepared.state.model.parent/'manifest.json').read_text())
    for side, settings in arm['arms'].items():
        assert str((directory/settings['calibration']).resolve()) in receipt['profile_dependencies_sha256']


def test_reject_duplicate_hardware_before_any_device_open(prepared):
    with pytest.raises(ValueError, match='already contains'):
        add_control_model(prepared.model.read_text(), {}, {}, {}, {}, {}, True)


@pytest.mark.parametrize('filename,commanded_expected,controllers', [
    ('feetech_gripper_probe_left.yaml', ['left_gripper_joint'], ('left_gripper_controller',)),
    ('feetech_gripper_normal_left.yaml', ['left_gripper_joint'], ('left_gripper_controller',)),
    ('feetech_commissioning_left_wrist.yaml', ['left_wrist_roll_joint'], ('left_arm_controller',)),
    ('feetech_commissioning_left_arm.yaml', [f'left_{s}_joint' for s in
        ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')],
        ('left_arm_controller', 'left_gripper_controller')),
    ('feetech_operational_left_arm.yaml', [f'left_{s}_joint' for s in
        ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')],
        ('left_arm_controller', 'left_gripper_controller')),
    ('feetech_operational_right_arm.yaml', [f'right_{s}_joint' for s in
        ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')],
        ('right_arm_controller', 'right_gripper_controller')),
])
def test_commissioning_profile_exposes_only_selected_motors(tmp_path, filename, commanded_expected, controllers, state_profile_file):
    profile = prepare_control_profile(state_profile_file, ROOT, tmp_path/'single',
        ROOT/'ros2_ws/src/cleany_robot_interface/test/data'/filename, read_only=False)
    tree = ET.parse(profile.model)
    commanded = [j.get('name') for j in tree.findall('ros2_control/joint')
                 if j.find('command_interface') is not None]
    assert commanded == commanded_expected
    assert len(tree.findall('ros2_control/joint/state_interface')) == 254
    for joint in tree.findall('ros2_control/joint'):
        if joint.get('name').startswith('head_'):
            continue
        assert [v.get('name') for v in joint.findall('state_interface')] == [
            'position','load_raw','torque_enabled','motor_status','temperature',
            'voltage_raw','goal_position_raw','command_position','sent_position','sent_goal_raw',
            'position_raw','current_raw','load_magnitude_raw','moving_flag','protocol_error_code',
            'torque_limit_readback_raw','current_limit_raw','temperature_raw','motor_status_raw',
            'feedback_stamp_ns','communication_recovery_count']
    assert profile.controller_names == controllers
    config = yaml.safe_load(profile.controllers.read_text())
    assert 'interfaces' not in config['joint_state_broadcaster']['ros__parameters']
    assert [j for name in controllers for j in config[name]['ros__parameters']['joints']] == commanded


def test_operational_profile_provides_normal_motion_limits(tmp_path, state_profile_file):
    limits = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    defaults = limits['joint_defaults']
    assert limits['serial_read_retries'] == 1
    assert limits['serial_read_recovery_budget_ms'] == 60
    profile = prepare_control_profile(state_profile_file, ROOT, tmp_path/'limits',
        ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml', read_only=False)
    for joint in ET.parse(profile.model).findall('ros2_control/joint'):
        if joint.get('name').startswith('head_'):
            continue
        values = {p.get('name'): p.text for p in joint.findall('param')}
        gripper = 'gripper' in joint.get('name')
        assert int(values['torque_limit_raw']) == (200 if gripper else 800)
        if gripper:
            assert int(values['temperature_limit_c']) == 80
    controller = limits['controller']
    assert defaults == {
        'max_velocity': 1.0,
        'max_acceleration': 2.0,
        'tracking_error': 0.0,
        'speed_raw': 800,
        'acceleration_raw': 254,
        'torque_limit_raw': 800,
    }
    assert controller == {
        'trajectory_tolerance': 0.0,
        'goal_tolerance': 0.10,
        'goal_time': 5.0,
    }
    generated = yaml.safe_load(profile.controllers.read_text())
    for name in profile.controller_names:
        params = generated[name]['ros__parameters']
        assert all(params['constraints'][joint]['goal'] == .10 for joint in params['joints'])
    assert controller['trajectory_tolerance'] == 0.0
    assert defaults['tracking_error'] == 0.0
    assert limits['temperature_limit_c'] == 80
    assert limits['temperature_fault_confirmation_samples'] == 3
    assert limits['allow_torque_on_recovery'] is True
    profile = prepare_control_profile(
        state_profile_file, ROOT,
        tmp_path/'recovery',
        ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml', read_only=False,
    )
    hardware = ET.parse(profile.model).getroot().find('ros2_control/hardware')
    assert hardware.find("param[@name='allow_torque_on_recovery']").text == 'true'
    assert hardware.find(
        "param[@name='temperature_fault_confirmation_samples']"
    ).text == '3'


@pytest.mark.parametrize('filename', [
    'feetech_operational_left_arm.yaml',
    'feetech_operational_right_arm.yaml',
])
def test_single_arm_operational_profiles_match_normal_speed_and_temperature(filename):
    profile = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data'/filename).read_text())
    assert profile['temperature_limit_c'] == 80
    assert profile['joint_defaults']['max_velocity'] == 1.0


@pytest.mark.parametrize('filename', [
    'feetech_control.yaml',
    'feetech_operational_left_arm.yaml',
    'feetech_operational_right_arm.yaml',
])
def test_operational_gripper_temperature_matches_contact_gate(filename):
    profile = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data'/filename).read_text())
    contact = yaml.safe_load((ROOT/'configs/robot/gripper_contact_decision.yaml').read_text())
    assert contact['criteria']['temperature_limit_c'] == 80
    for name, values in profile['joint_overrides'].items():
        if name.endswith('_gripper_joint'):
            assert values.get('temperature_limit_c', profile['temperature_limit_c']) == (
                contact['criteria']['temperature_limit_c'])
