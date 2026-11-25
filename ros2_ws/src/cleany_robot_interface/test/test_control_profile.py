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
from cleany_robot_interface.core import mappings, motor_calibration_directory
from cleany_robot_interface.joint_bounds import calibrated_joint_bounds

ROOT = Path(__file__).resolve().parents[4]


def measured_bounds(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    directory = motor_calibration_directory(prepared.state.arm_config, arm)
    calibrations = {side: json.loads((directory / cfg['calibration']).read_text())
                    for side, cfg in arm['arms'].items()}
    joints = mappings(arm, calibrations)
    return calibrated_joint_bounds(joints, [joint.name for joint in joints])


def write_synthetic_state_profile(directory: Path) -> Path:
    config = json.loads((ROOT / 'configs/robot/camera_preview.json').read_text())
    config.pop('camera_pose_config', None)  # 합성 모터 설정에는 작업자 카메라 모델을 연결하지 않습니다.
    config['arm_config'] = 'ros2_ws/src/cleany_robot_interface/config/arm_preview.yaml'
    config['head_rgb_extrinsics'] = 'configs/robot/head_rgb_board_extrinsics.yaml'
    arm = yaml.safe_load((ROOT / config['arm_config']).read_text())
    arm['port'] = '/dev/cleany-synthetic-test-only'
    arm['calibration_dir'] = str(directory)
    frozen = ROOT / 'ros2_ws/src/cleany_description/config/calibration/jetson_20261009/motors'
    for side in ('left', 'right'):
        filename = arm['arms'][side]['calibration']
        (directory / filename).write_bytes((frozen / filename).read_bytes())
    arm_path = directory / 'synthetic_arm.yaml'
    arm_path.write_text(yaml.safe_dump(arm))
    config['arm_config'] = str(arm_path)
    profile = directory / 'synthetic_state.json'
    profile.write_text(json.dumps(config))
    return profile


@pytest.fixture
def state_profile_file(tmp_path):
    return write_synthetic_state_profile(tmp_path)


@pytest.fixture(scope='module')
def prepared(tmp_path_factory):
    # 기본 프로파일을 읽기만 하는 검사는 한 번 생성한 모델과 설정을 공유합니다.
    directory = tmp_path_factory.mktemp('default_control_profile')
    state_profile = write_synthetic_state_profile(directory)
    return prepare_control_profile(state_profile, ROOT, directory/'run',
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
    physical = measured_bounds(prepared)
    policies = limits.get('joint_position_bounds', {})
    for name, (lower, upper) in physical.items():
        original = state_model.find(f"joint[@name='{name}']/limit")
        original_values = [float(original.get(key)) for key in ('lower', 'upper')]
        policy = policies.get(name, original_values)
        actual = root.find(f"joint[@name='{name}']/limit")
        assert [float(actual.get(key)) for key in ('lower', 'upper')] == [
            max(policy[0], lower), min(policy[1], upper),
        ]
        for key in ('lower', 'upper'):
            actual.set(key, original.get(key))
    root.remove(root.find('ros2_control'))
    assert ET.tostring(root) == ET.tostring(state_model)


def test_physical_profile_selects_optimized_arm_collision_meshes(prepared):
    from collections import Counter

    settings = json.loads((ROOT / 'configs/robot/arm_collision_meshes.json').read_text())
    expected = {f'package://cleany_description/meshes/collision/{name}'
                for name in settings['source_meshes']}
    for path in (prepared.state.model, prepared.model):
        root = ET.parse(path).getroot()
        actual = Counter(mesh.get('filename')
                         for mesh in root.findall('./link/collision/geometry/mesh')
                         if '/meshes/collision/' in mesh.get('filename'))
        assert set(actual) == expected
        assert sum(actual.values()) == 28  # 22 arm and six mast/head instances.
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
        assert config['controller_manager']['ros__parameters'][name]['type'] == (
            'joint_trajectory_controller/JointTrajectoryController')
        params = config[name]['ros__parameters']
        assert params['action_monitor_rate'] == 20.0
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
    defaults = yaml.safe_load((Path(__file__).parent / 'data/feetech_control.yaml').read_text())['joint_defaults']
    for name, item in limits.items():
        joint = hardware.find(f"ros2_control/joint[@name='{name}']")
        assert item['has_acceleration_limits'] is True
        for key in ('max_velocity', 'max_acceleration'):
            assert item[key] == defaults[key]
            assert float(joint.find(f"param[@name='{key}']").text) == item[key]
        assert joint.find("param[@name='tracking_error']") is None


def test_existing_mapping_and_eeprom_values_preserved(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    root = ET.parse(prepared.model).getroot()
    left = root.find("ros2_control/joint[@name='left_gripper_joint']")
    assert float(left.find("param[@name='zero_tick']").text) == arm['overrides']['left']['gripper']['zero_tick']
    assert left.find("param[@name='motor_id']").text == '6'
    manifest = json.loads((prepared.model.parent/'control_manifest.json').read_text())
    assert manifest['read_only'] and not manifest['physical_motion_validated']


def test_planning_limits_include_gripper_overrides(tmp_path, state_profile_file):
    config = yaml.safe_load((Path(__file__).parent / 'data/feetech_control.yaml').read_text())
    for side in ('left', 'right'):
        config.setdefault('joint_overrides', {}).setdefault(f'{side}_gripper_joint', {}).update(
            max_velocity=.06, max_acceleration=.1)
    control = tmp_path / 'gripper_control.yaml'
    control.write_text(yaml.safe_dump(config))
    profile = prepare_control_profile(state_profile_file, ROOT, tmp_path / 'gripper_limits', control)
    planning = yaml.safe_load(profile.joint_limits.read_text())['joint_limits']
    hardware = ET.parse(profile.model).getroot()
    for name, limits in planning.items():
        expected = config['joint_overrides'].get(name, {})
        joint = hardware.find(f"ros2_control/joint[@name='{name}']")
        for key in ('max_velocity', 'max_acceleration'):
            assert limits[key] == expected.get(key, config['joint_defaults'][key])
            assert float(joint.find(f"param[@name='{key}']").text) == limits[key]


def test_calibrated_pitch_bounds_match_state_model_driver_and_recorded_positions(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    directory = motor_calibration_directory(prepared.state.arm_config, arm)
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


def test_head_mapping_bounds_reach_state_and_control_models(tmp_path, state_profile_file):
    config = json.loads(state_profile_file.read_text())
    mapping = json.loads((ROOT / config['head_joint_mapping']).read_text())
    bounds = {'head_pan_joint': [-2., 2.], 'head_tilt_joint': [-.3, .6]}
    mapping['reference_ticks'] = [2010, 1000]
    mapping['joint_position_bounds'] = bounds
    mapping_file = tmp_path / 'head_mapping.json'
    mapping_file.write_text(json.dumps(mapping))
    config['head_joint_mapping'] = str(mapping_file)
    state_profile_file.write_text(json.dumps(config))
    prepared = prepare_control_profile(
        state_profile_file, ROOT, tmp_path / 'head_bounds_run',
        ROOT / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml')
    for path in (prepared.state.model, prepared.model):
        root = ET.parse(path).getroot()
        for name, expected in bounds.items():
            limit = root.find(f"joint[@name='{name}']/limit")
            assert [float(limit.get(key)) for key in ('lower', 'upper')] == expected
    root = ET.parse(prepared.model).getroot()
    tilt = root.find("ros2_control/joint[@name='head_tilt_joint']")
    assert float(tilt.find("param[@name='zero_tick']").text) == 1000


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
    model = ET.parse(profile.model)
    for joint in model.findall('ros2_control/joint'):
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
    assert limits['temperature_limit_c'] == 80
    assert 'temperature_fault_confirmation_samples' not in limits
    assert limits['allow_torque_on_recovery'] is True
    hardware = model.find('ros2_control/hardware')
    assert hardware.find("param[@name='allow_torque_on_recovery']").text == 'true'
    for name, expected in (('fast', .1), ('slow', .5), ('signal', .1)):
        key = f'temperature_macd_{name}_tau_s'
        assert limits[key] == expected
        assert float(hardware.find(f"param[@name='{key}']").text) == expected
    assert hardware.find("param[@name='temperature_fault_confirmation_samples']") is None


@pytest.mark.parametrize('filename', [
    'feetech_operational_left_arm.yaml',
    'feetech_operational_right_arm.yaml',
])
def test_single_arm_operational_profiles_match_normal_speed_and_temperature(filename):
    profile = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data'/filename).read_text())
    assert profile['temperature_limit_c'] == 80
    assert profile['joint_defaults']['max_velocity'] == 1.0


@pytest.mark.parametrize('filename', ['feetech_control.yaml', 'feetech_operational_left_arm.yaml',
                                      'feetech_operational_right_arm.yaml'])
def test_jetson_motor_protection_reaches_driver(tmp_path, state_profile_file, filename):
    control = ROOT/'configs/robot/jetson'/filename
    model = ET.parse(prepare_control_profile(state_profile_file, ROOT, tmp_path/'jetson', control).model)
    assert model.findtext("ros2_control/hardware/param[@name='temperature_limit_c']") == '80'
    for joint in model.findall('ros2_control/joint'):
        params = {p.get('name'): p.text for p in joint.findall('param')}
        motor_id = int(params['motor_id'])
        assert params.get('temperature_limit_c', '80') == '80'
        if motor_id <= 12 and motor_id not in (6, 12):
            assert (params['endpoint_margin_ticks'], params['endpoint_load_raw']) == ('20', '300')
        else:
            assert 'endpoint_margin_ticks' not in params and 'endpoint_load_raw' not in params


@pytest.mark.parametrize('profile,arms', [
    ('feetech_control.yaml', ('left', 'right')),
    ('feetech_operational_left_arm.yaml', ('left',)),
    ('feetech_operational_right_arm.yaml', ('right',)),
])
def test_front_shoulder_bounds_reach_planning_model_and_motor_driver(
        tmp_path, state_profile_file, profile, arms):
    prepared = prepare_control_profile(
        state_profile_file, ROOT, tmp_path/'front',
        Path(__file__).parent/'data'/profile)
    model = ET.parse(prepared.model).getroot()
    for arm in arms:
        name = f'{arm}_shoulder_yaw_joint'
        policy = (-2.16, 0.) if arm == 'left' else (0., 2.0033789089786476)
        physical = measured_bounds(prepared)[name]
        expected = (max(policy[0], physical[0]), min(policy[1], physical[1]))
        limit = model.find(f"joint[@name='{name}']/limit")
        assert tuple(float(limit.get(k)) for k in ('lower', 'upper')) == expected
        joint = model.find(f"ros2_control/joint[@name='{name}']")
        assert tuple(float(joint.find(f"param[@name='{k}']").text)
                     for k in ('lower', 'upper')) == expected


@pytest.mark.parametrize('plugin,rate', [
    ('cleany_trajectory_controller/JointTrajectoryController', 1.),
    ('cleany_trajectory_controller/JointTrajectoryController', 20.),
    ('joint_trajectory_controller/JointTrajectoryController', 100.),
])
def test_controller_transport_profile(tmp_path, state_profile_file, plugin, rate):
    limits = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    limits['controller'].update(type=plugin, action_monitor_rate=rate)
    control = tmp_path/'notification.yaml'
    control.write_text(yaml.safe_dump(limits))
    prepared = prepare_control_profile(state_profile_file, ROOT, tmp_path/'run', control)
    config = yaml.safe_load(prepared.controllers.read_text())
    for name in prepared.controller_names:
        assert config['controller_manager']['ros__parameters'][name]['type'] == plugin
        assert config[name]['ros__parameters']['action_monitor_rate'] == rate


@pytest.mark.parametrize('settings,field', [
    ({'type': 'unknown/controller'}, 'type'),
    *[({'action_monitor_rate': rate}, 'action_monitor_rate')
      for rate in (0, 101, float('nan'), float('inf'), True, '20')],
])
def test_controller_transport_rejects_invalid_settings(tmp_path, state_profile_file, settings, field):
    limits = yaml.safe_load((ROOT/'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    limits['controller'].update(settings)
    control = tmp_path/'invalid-notification.yaml'
    control.write_text(yaml.safe_dump(limits))
    output = tmp_path/'run'
    with pytest.raises(ValueError, match=rf'controller\.{field}'):
        prepare_control_profile(state_profile_file, ROOT, output, control)
    assert not output.exists()


def test_control_limits_cannot_expand_measured_motor_travel(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    directory = motor_calibration_directory(prepared.state.arm_config, arm)
    calibrations = {side: json.loads((directory / cfg['calibration']).read_text())
                    for side, cfg in arm['arms'].items()}
    limits = yaml.safe_load((ROOT / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    limits['joint_position_bounds'] = {f'{side}_shoulder_yaw_joint': [-10., 10.]
                                       for side in ('left', 'right')}
    model, _, _ = add_control_model(prepared.state.model.read_text(), arm, calibrations,
        yaml.safe_load(prepared.state.head_config.read_text()),
        json.loads(prepared.state.head_mapping.read_text()), limits, True)
    root = ET.fromstring(model)
    for joint in mappings(arm, calibrations):
        expected = sorted((joint.radians(joint.range_min), joint.radians(joint.range_max)))
        limit = root.find(f"joint[@name='{joint.name}']/limit")
        assert float(limit.get('lower')) >= expected[0] - 1e-12
        assert float(limit.get('upper')) <= expected[1] + 1e-12


def test_control_rejects_policy_disjoint_from_motor_range(prepared):
    arm = yaml.safe_load(prepared.state.arm_config.read_text())
    directory = motor_calibration_directory(prepared.state.arm_config, arm)
    calibrations = {side: json.loads((directory / cfg['calibration']).read_text())
                    for side, cfg in arm['arms'].items()}
    limits = yaml.safe_load((ROOT / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())
    limits['joint_position_bounds'] = {'left_shoulder_yaw_joint': [10., 11.]}
    with pytest.raises(ValueError, match='do not overlap'):
        add_control_model(prepared.state.model.read_text(), arm, calibrations,
            yaml.safe_load(prepared.state.head_config.read_text()),
            json.loads(prepared.state.head_mapping.read_text()), limits, True)


def test_control_model_supplies_the_host_executor_limit_contract(prepared):
    from cleany_skill_executor.core.hardware_execution import physical_limits

    model = ET.parse(prepared.model).getroot()
    model.find("ros2_control/hardware/param[@name='read_only']").text = 'false'
    xml = ET.tostring(model, encoding='unicode')
    for arm in ('left', 'right'):
        actual = physical_limits(xml, arm)
        assert len(actual) == 6
        for name, bounds in actual.items():
            limit = model.find(f"joint[@name='{name}']/limit")
            assert (bounds.lower, bounds.upper) == tuple(
                float(limit.get(key)) for key in ('lower', 'upper'))
            assert bounds.velocity > 0 and bounds.acceleration > 0


def test_jetson_profile_reads_both_calibrations_without_a_home_cache(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.chdir(tmp_path)
    jetson = ROOT / 'configs/robot/jetson'
    output = tmp_path / 'runtime'
    prepared = prepare_control_profile(
        ROOT / 'configs/robot/cad_grasp.json', ROOT, output, jetson / 'feetech_control.yaml')
    manifest = json.loads((output / 'control_manifest.json').read_text())
    motors = prepared.state.arm_config.parent / 'calibration/jetson_20261115_open160/motors'
    assert manifest['read_only']
    assert set(map(Path, manifest['source_sha256'])) == {
        jetson / 'feetech_control.yaml', motors / 'cleany_so101_left.json', motors / 'cleany_so101_right.json'}
