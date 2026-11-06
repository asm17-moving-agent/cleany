from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
import subprocess

from cleany_moveit_config.joint_limits import limits_from_model


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION_ROOT = PACKAGE_ROOT.parent / 'cleany_description'
CONFIG_ROOT = PACKAGE_ROOT / 'config'

ARM_JOINT_SUFFIXES = (
    'shoulder_yaw_joint',
    'shoulder_pitch_joint',
    'elbow_pitch_joint',
    'wrist_pitch_joint',
    'wrist_roll_joint',
)
SIDES = ('left', 'right')


def test_materialized_bounds_replace_stale_cad_planning_positions_only():
    config = {'robot_description_planning': {'joint_limits': {
        'pitch': {'has_position_limits': True, 'min_position': -.22,
                  'max_position': 3.37, 'max_velocity': 1.0,
                  'has_acceleration_limits': True, 'max_acceleration': 2.0},
    }}}
    model = '<robot><joint name="pitch" type="revolute"><limit lower="-.233" upper="3.45"/></joint></robot>'
    actual = limits_from_model(config, model)['robot_description_planning']['joint_limits']['pitch']
    assert actual['min_position'] == -.233 and actual['max_position'] == 3.45
    assert actual['max_velocity'] == 1.0 and actual['max_acceleration'] == 2.0
    assert config['robot_description_planning']['joint_limits']['pitch']['max_position'] == 3.37


@pytest.mark.parametrize('model', [
    '<robot/>',
    '<robot><joint name="pitch" type="fixed"/></robot>',
    '<robot><joint name="pitch" type="revolute"><limit lower="nan" upper="1"/></joint></robot>',
])
def test_materialized_planning_limits_reject_missing_or_invalid_bounds(model):
    config = {'robot_description_planning': {'joint_limits': {'pitch': {}}}}
    with pytest.raises(ValueError):
        limits_from_model(config, model)


def _launch_nodes_and_context(filename: str, **overrides):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    spec = importlib.util.spec_from_file_location(
        filename, PACKAGE_ROOT / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declared = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert set(overrides) <= set(declared)
    context = LaunchContext()
    context.launch_configurations.update(declared)
    context.launch_configurations.update(overrides)
    actions = []
    for action in description.entities:
        actions.extend(action.execute(context) if isinstance(action, OpaqueFunction) else [action])
    nodes = {perform_substitutions(context, normalize_to_list_of_substitutions(
        action.node_executable)): action
             for action in actions if isinstance(action, Node)}
    return nodes, context


def _node_parameters(node, context) -> dict:
    from launch_ros.utilities import evaluate_parameters

    evaluated = evaluate_parameters(context, node._Node__parameters)
    return {key: value for item in evaluated if isinstance(item, dict)
            for key, value in item.items()}


@pytest.mark.parametrize('filename,include_head', [
    ('physical_move_group.launch.py', False),
    ('physical_move_group.launch.py', True),
])
@pytest.mark.parametrize('use_sim_time', [False, True])
def test_optional_rviz_uses_the_moveit_model_and_selected_clock(
    filename: str, include_head: bool, use_sim_time: bool,
) -> None:
    options = {'use_rviz': 'true', 'use_sim_time': str(use_sim_time).lower()}
    if filename != 'move_group.launch.py':
        options['include_head_camera'] = str(include_head).lower()
    nodes, context = _launch_nodes_and_context(filename, **options)
    rviz = nodes['rviz2']
    assert rviz.condition.evaluate(context)
    combined = _node_parameters(rviz, context)
    server = _node_parameters(nodes['move_group'], context)
    assert server['capabilities'] == 'cleany/RequireCollisionDetector'
    assert server['collision_detector'] == 'cleany/SolidFCL'
    assert all(server[key] == value for key, value in combined.items())
    assert 'robot_description' in combined
    assert 'robot_description_semantic' in combined
    for side in SIDES:
        assert combined[f'robot_description_kinematics.{side}_arm.kinematics_solver'] == (
            'kdl_kinematics_plugin/KDLKinematicsPlugin')
    assert combined['use_sim_time'] is use_sim_time
    root = ET.fromstring(combined['robot_description'])
    assert (root.find("joint[@name='head_pan_joint']") is not None) is include_head
    assert all(root.find(f"joint[@name='{name}_wheel_joint']").get('type') == 'fixed'
               for name in ('rear_left', 'rear_right', 'front_left', 'front_right'))
    context.launch_configurations['use_rviz'] = 'false'
    assert not rviz.condition.evaluate(context)


@pytest.mark.parametrize('explicit_limits', [False, True])
def test_physical_rviz_and_planning_preserve_calibrated_wrist_bounds(
    tmp_path: Path, explicit_limits: bool,
) -> None:
    calibration = yaml.safe_load((DESCRIPTION_ROOT / 'config' / 'calibration' /
        'jetson_20261009' / 'arm_joint_calibration.yaml').read_text())
    lower, upper = calibration['arms']['left']['joints']['left_wrist_roll_joint'][
        'observed_position_limits_rad']
    root = ET.fromstring(subprocess.check_output([
        'xacro', str(DESCRIPTION_ROOT / 'urdf' / 'cleany.urdf.xacro'),
        'include_wheel_joints:=false',
    ], text=True))
    root.find("joint[@name='left_wrist_roll_joint']/limit").attrib.update(
        lower=str(lower), upper=str(upper))
    model = tmp_path / 'robot.urdf'
    model.write_text(ET.tostring(root, encoding='unicode'))
    options = {'robot_description_file': str(model), 'include_head_camera': 'true'}
    if explicit_limits:
        # Real control profiles supply dynamics; positions remain in their URDF.
        limits = tmp_path / 'joint_limits.yaml'
        limits.write_text(yaml.safe_dump({'joint_limits': {'left_wrist_roll_joint': {
            'has_velocity_limits': True, 'max_velocity': 0.3,
            'has_acceleration_limits': True, 'max_acceleration': 0.5}}}))
        options['joint_limits_file'] = str(limits)
    nodes, context = _launch_nodes_and_context('physical_move_group.launch.py', **options)
    for node in (nodes['move_group'], nodes['rviz2']):
        parameters = _node_parameters(node, context)
        actual = ET.fromstring(parameters['robot_description']).find(
            "joint[@name='left_wrist_roll_joint']/limit")
        assert float(actual.get('lower')) == lower
        assert float(actual.get('upper')) == upper
        prefix = 'robot_description_planning.joint_limits.left_wrist_roll_joint.'
        if explicit_limits:
            assert prefix + 'min_position' not in parameters
            assert parameters[prefix + 'max_velocity'] == 0.3
            assert parameters[prefix + 'max_acceleration'] == 0.5
        else:
            assert parameters[prefix + 'min_position'] == lower
            assert parameters[prefix + 'max_position'] == upper
