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


def _arm_joints(side: str) -> tuple[str, ...]:
    return tuple(f'{side}_{suffix}' for suffix in ARM_JOINT_SUFFIXES)


def _all_modeled_joints() -> set[str]:
    return {
        *(_arm_joints('left')),
        'left_gripper_joint',
        *(_arm_joints('right')),
        'right_gripper_joint',
    }


def _load_yaml(filename: str) -> dict:
    with (CONFIG_ROOT / filename).open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    assert isinstance(value, dict)
    return value


def _expand_side(value: str, side: str) -> str:
    return value.replace('${side}', side)


def _canonical_joint_elements() -> list[ET.Element]:
    root = ET.parse(DESCRIPTION_ROOT / 'urdf' / 'dual_arm.xacro').getroot()
    return [
        element
        for element in root.findall('.//joint')
        if element.get('name', '').startswith('${side}_')
    ]


def _canonical_chain(side: str, tip_link: str) -> tuple[str, ...]:
    outgoing: dict[str, tuple[str, str]] = {}
    for joint in _canonical_joint_elements():
        parent = joint.find('parent')
        child = joint.find('child')
        assert parent is not None and child is not None
        parent_link = _expand_side(parent.attrib['link'], side)
        child_link = _expand_side(child.attrib['link'], side)
        joint_name = _expand_side(joint.attrib['name'], side)
        outgoing[parent_link] = (joint_name, child_link)

    current = 'base_link'
    joints: list[str] = []
    while current != tip_link:
        assert current in outgoing, (
            f'chain stops before {tip_link} at {current}'
        )
        joint_name, current = outgoing[current]
        joints.append(joint_name)
        assert len(joints) <= len(outgoing), 'cycle in canonical arm chain'
    return tuple(joints)


def test_srdf_has_arm_and_grasp_tcp_chains() -> None:
    root = ET.parse(CONFIG_ROOT / 'cleany.srdf').getroot()
    groups = {
        element.attrib['name']: element for element in root.findall('group')
    }
    assert set(groups) == {
        'left_arm', 'right_arm', 'left_grasp_arm', 'right_grasp_arm',
        'left_pregrasp_aim_arm', 'right_pregrasp_aim_arm',
        'left_pregrasp_open', 'right_pregrasp_open',
        'left_return_close', 'right_return_close',
    }

    for side in SIDES:
        for suffix in ('pregrasp_open', 'return_close'):
            coordinated = groups[f'{side}_{suffix}']
            assert [child.attrib for child in coordinated.findall('group')] == [
                {'name': f'{side}_grasp_arm'}]
            assert [child.attrib for child in coordinated.findall('joint')] == [
                {'name': f'{side}_gripper_joint'}]
            assert len(coordinated) == 2
        group = groups[f'{side}_arm']
        children = list(group)
        assert len(children) == 1
        assert children[0].tag == 'chain'
        assert children[0].attrib == {
            'base_link': 'base_link',
            'tip_link': f'{side}_gripper_frame',
        }
        chain = _canonical_chain(side, f'{side}_gripper_frame')
        assert chain == _arm_joints(side)
        assert f'{side}_gripper_joint' not in chain
        grasp_group = groups[f'{side}_grasp_arm']
        grasp_chain = list(grasp_group)[0]
        assert grasp_chain.attrib == {
            'base_link': 'base_link',
            'tip_link': f'{side}_grasp_tcp',
        }
        assert _canonical_chain(side, f'{side}_grasp_tcp') == (
            *_arm_joints(side), f'{side}_grasp_tcp_joint'
        )
        aim_group = groups[f'{side}_pregrasp_aim_arm']
        aim_chain = list(aim_group)[0]
        assert aim_chain.attrib == {
            'base_link': 'base_link',
            'tip_link': f'{side}_pregrasp_aim_tip',
        }
        assert _canonical_chain(side, f'{side}_pregrasp_aim_tip') == (
            *_arm_joints(side),
            f'{side}_grasp_tcp_joint',
            f'{side}_pregrasp_aim_tip_joint',
        )
        aim_joint = next(
            joint
            for joint in _canonical_joint_elements()
            if joint.attrib['name'] == '${side}_pregrasp_aim_tip_joint'
        )
        assert aim_joint.find('origin').attrib == {
            'xyz': '0 -0.140 0',
            'rpy': '0 0 0',
        }


def test_named_home_states_are_all_zero() -> None:
    root = ET.parse(CONFIG_ROOT / 'cleany.srdf').getroot()
    states = {
        (element.attrib['group'], element.attrib['name']): element
        for element in root.findall('group_state')
    }
    assert set(states) == {
        ('left_arm', 'left_home'),
        ('right_arm', 'right_home'),
    }

    for side in SIDES:
        state = states[(f'{side}_arm', f'{side}_home')]
        values = {
            joint.attrib['name']: float(joint.attrib['value'])
            for joint in state.findall('joint')
        }
        assert tuple(values) == _arm_joints(side)
        assert all(value == 0.0 for value in values.values())


def test_self_collision_matrix_only_disables_adjacent_links() -> None:
    root = ET.parse(CONFIG_ROOT / 'cleany.srdf').getroot()
    disabled = {
        frozenset((entry.attrib['link1'], entry.attrib['link2']))
        for entry in root.findall('disable_collisions')
    }
    assert all(
        entry.attrib['reason'] == 'Adjacent'
        for entry in root.findall('disable_collisions')
    )

    expected: set[frozenset[str]] = {frozenset(('base_link', 'top_base_link'))}
    for side in SIDES:
        links = (
            'base_link',
            f'{side}_shoulder_yaw_link',
            f'{side}_upper_arm_link',
            f'{side}_lower_arm_link',
            f'{side}_wrist_pitch_link',
            f'{side}_gripper_frame',
            f'{side}_moving_jaw_link',
        )
        expected.update(frozenset(pair) for pair in zip(links, links[1:]))
    assert disabled == expected
    assert not any(
        any(link.startswith('left_') for link in pair)
        and any(link.startswith('right_') for link in pair)
        for pair in disabled
    )


def test_both_groups_use_position_only_kdl() -> None:
    kinematics = _load_yaml('kinematics.yaml')
    assert set(kinematics) == {
        'left_arm', 'right_arm', 'left_grasp_arm', 'right_grasp_arm',
        'left_pregrasp_aim_arm', 'right_pregrasp_aim_arm',
    }
    for group in kinematics:
        assert kinematics[group] == {
            'kinematics_solver': (
                'kdl_kinematics_plugin/KDLKinematicsPlugin'
            ),
            'position_only_ik': True,
        }


def test_moveit_joint_limits_match_canonical_urdf() -> None:
    configured = _load_yaml('joint_limits.yaml')['joint_limits']
    assert set(configured) == _all_modeled_joints()

    expanded = subprocess.run(
        ['xacro', str(DESCRIPTION_ROOT / 'urdf' / 'cleany.urdf.xacro')],
        check=True, capture_output=True, text=True,
    )
    root = ET.fromstring(expanded.stdout)
    for joint in root.findall('./joint'):
        if joint.attrib['name'] not in configured:
            continue
        if joint.attrib['type'] == 'fixed':
            continue
        limit = joint.find('limit')
        if limit is None:
            assert joint.attrib['type'] == 'fixed'
            continue
        actual = configured[joint.attrib['name']]
        assert actual['has_position_limits'] is True
        assert actual['min_position'] == pytest.approx(float(limit.attrib['lower']))
        assert actual['max_position'] == pytest.approx(float(limit.attrib['upper']))
        assert actual['has_velocity_limits'] is True
        assert actual['max_velocity'] == pytest.approx(float(limit.attrib['velocity']))
        assert actual['has_acceleration_limits'] is False


def test_ompl_is_configured_for_each_arm() -> None:
    ompl = _load_yaml('ompl_planning.yaml')
    assert ompl['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    assert ompl['planner_configs']['RRTConnectkConfigDefault']['type'] == (
        'geometric::RRTConnect'
    )
    for group in (
        'left_arm', 'right_arm', 'left_grasp_arm', 'right_grasp_arm',
        'left_pregrasp_aim_arm', 'right_pregrasp_aim_arm',
    ):
        expected = ['RRTConnectkConfigDefault']
        if group in ('left_grasp_arm', 'right_grasp_arm'):
            expected.append('RRTConnectReturnkConfigDefault')
        assert ompl[group]['planner_configs'] == expected


def test_return_profile_keeps_collision_resolution_and_avoids_duplicate_simplification():
    ompl = _load_yaml('ompl_planning.yaml')
    base = ompl['planner_configs']['RRTConnectkConfigDefault']
    fast = ompl['planner_configs']['RRTConnectReturnkConfigDefault']
    assert fast['type'] == base['type'] and fast['range'] == base['range']
    assert fast['simplify_solutions'] == 0 and 'simplify_solutions' not in base
    assert 'interpolate' not in fast
    for side in SIDES:
        assert ompl[f'{side}_grasp_arm']['longest_valid_segment_fraction'] == .001
    assert 'default_planner_request_adapters/AddTimeOptimalParameterization' in ompl['request_adapters']


def test_moveit_controllers_claim_disjoint_side_joints() -> None:
    config = _load_yaml('moveit_controllers.yaml')
    assert config['moveit_controller_manager'] == (
        'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    )
    manager = config['moveit_simple_controller_manager']
    assert manager['controller_names'] == [
        'left_arm_controller',
        'right_arm_controller',
    ]

    claimed: list[set[str]] = []
    for side in SIDES:
        controller = manager[f'{side}_arm_controller']
        assert controller['type'] == 'FollowJointTrajectory'
        assert controller['action_ns'] == 'follow_joint_trajectory'
        assert controller['default'] is True
        assert tuple(controller['joints']) == _arm_joints(side)
        assert f'{side}_gripper_joint' not in controller['joints']
        claimed.append(set(controller['joints']))
    assert claimed[0].isdisjoint(claimed[1])


@pytest.mark.parametrize('include_head', [False, True])
def test_mock_ros2_control_matches_moveit_controller_contract(include_head: bool) -> None:
    controllers = _load_yaml('mock_ros2_controllers.yaml')
    manager = controllers['controller_manager']['ros__parameters']
    assert manager['joint_state_broadcaster']['type'] == (
        'joint_state_broadcaster/JointStateBroadcaster'
    )

    for side in SIDES:
        name = f'{side}_arm_controller'
        assert manager[name]['type'] == (
            'joint_trajectory_controller/JointTrajectoryController'
        )
        parameters = controllers[name]['ros__parameters']
        assert tuple(parameters['joints']) == _arm_joints(side)
        assert parameters['command_interfaces'] == ['position']
        assert parameters['state_interfaces'] == ['position', 'velocity']
        assert parameters['allow_partial_joints_goal'] is False
        assert parameters['allow_nonzero_velocity_at_trajectory_end'] is False

    mock_root = ET.fromstring(subprocess.check_output([
        'xacro', str(CONFIG_ROOT / 'cleany_mock.urdf.xacro'),
        f'include_head_camera:={str(include_head).lower()}',
    ], text=True))
    plugin = mock_root.find('.//hardware/plugin')
    assert plugin is not None
    assert plugin.text == 'mock_components/GenericSystem'
    configured_mock_joints = {
        element.attrib['name']
        for element in mock_root.findall('ros2_control/joint')
    }
    expected = _all_modeled_joints()
    if include_head:
        expected.update(('head_pan_joint', 'head_tilt_joint'))
    assert configured_mock_joints == expected


def test_optional_rviz_uses_the_moveit_model_and_selected_clock() -> None:
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location(
        'move_group_launch', PACKAGE_ROOT / 'launch' / 'move_group.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declared = {
        action.name: perform_substitutions(LaunchContext(), action.default_value)
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert declared['use_rviz'] == declared['include_head_camera'] == 'false'
    context = LaunchContext()
    context.launch_configurations.update(declared)
    context.launch_configurations['use_rviz'] = 'true'
    _, rviz = module._launch_nodes(context)
    assert rviz.condition.evaluate(context)
    parameters = rviz._Node__parameters
    from launch_ros.utilities import evaluate_parameters
    evaluated = evaluate_parameters(context, parameters)
    combined = {key: value for item in evaluated for key, value in item.items()}
    assert 'robot_description' in combined
    assert 'robot_description_semantic' in combined
    for side in SIDES:
        assert combined[f'robot_description_kinematics.{side}_arm.kinematics_solver'] == (
            'kdl_kinematics_plugin/KDLKinematicsPlugin')
    assert combined['use_sim_time'] is False
    root = ET.fromstring(combined['robot_description'])
    assert root.find("joint[@name='head_pan_joint']") is None
    assert all(root.find(f"joint[@name='{name}_wheel_joint']").get('type') == 'fixed'
               for name in ('rear_left', 'rear_right', 'front_left', 'front_right'))
    context.launch_configurations.update(use_rviz='false', use_sim_time='true')
    _, rviz = module._launch_nodes(context)
    assert not rviz.condition.evaluate(context)
    evaluated = evaluate_parameters(context, rviz._Node__parameters)
    assert next(item['use_sim_time'] for item in evaluated if 'use_sim_time' in item) is True
