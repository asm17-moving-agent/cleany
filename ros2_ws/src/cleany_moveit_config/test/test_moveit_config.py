from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET
import runpy

import pytest
import yaml


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


def test_grasp_corridors_use_fine_collision_resolution_and_bounded_rounding():
    config = _load_yaml('ompl_planning.yaml')
    assert config['path_tolerance'] == 0.0001
    for side in SIDES:
        assert config[f'{side}_grasp_arm']['longest_valid_segment_fraction'] == .001


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
        combined = groups[f'{side}_pregrasp_open']
        assert combined.find('group').attrib == {'name': f'{side}_grasp_arm'}
        assert combined.find('joint').attrib == {'name': f'{side}_gripper_joint'}
        returning = groups[f'{side}_return_close']
        assert returning.find('group').attrib == {'name': f'{side}_grasp_arm'}
        assert returning.find('joint').attrib == {'name': f'{side}_gripper_joint'}
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

    expected: set[frozenset[str]] = set()
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

    for joint in _canonical_joint_elements():
        if joint.attrib['type'] == 'fixed':
            continue
        template_name = joint.attrib['name']
        limit = joint.find('limit')
        if limit is None:
            assert joint.attrib['type'] == 'fixed'
            continue
        for side in SIDES:
            joint_name = _expand_side(template_name, side)
            actual = configured[joint_name]
            assert actual['has_position_limits'] is True
            assert actual['min_position'] == pytest.approx(
                float(limit.attrib['lower'])
            )
            assert actual['max_position'] == pytest.approx(
                float(limit.attrib['upper'])
            )
            assert actual['has_velocity_limits'] is True
            assert actual['max_velocity'] == pytest.approx(
                float(limit.attrib['velocity'])
            )
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
        assert ompl[group]['planner_configs'] == ['RRTConnectkConfigDefault']


def test_pilz_lin_and_cartesian_limits_are_configured() -> None:
    pilz = _load_yaml('pilz_industrial_motion_planner_planning.yaml')
    limits = _load_yaml('pilz_cartesian_limits.yaml')['cartesian_limits']
    assert pilz['planning_plugin'] == (
        'pilz_industrial_motion_planner/CommandPlanner'
    )
    assert limits == {
        'max_trans_vel': 0.10,
        'max_trans_acc': 0.20,
        'max_trans_dec': -0.20,
        'max_rot_vel': 0.50,
    }


@pytest.mark.parametrize('launch_name', ['move_group.launch.py', 'mock_planning.launch.py'])
def test_ompl_remains_default_with_pilz_available(launch_name: str) -> None:
    source = (PACKAGE_ROOT / 'launch' / launch_name).read_text(encoding='utf-8')
    assert "default_planning_pipeline='ompl'" in source
    assert "['ompl', 'pilz_industrial_motion_planner']" in source


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


def test_sorting_controller_profile_adds_disjoint_grippers_only():
    base = _load_yaml('moveit_controllers.yaml')['moveit_simple_controller_manager']
    combined = _load_yaml('sorting_moveit_controllers.yaml')['moveit_simple_controller_manager']
    claimed = set()
    for name in combined['controller_names']:
        joints = set(combined[name]['joints'])
        assert not joints & claimed
        claimed |= joints
    for arm in SIDES:
        assert combined[f'{arm}_arm_controller'] == base[f'{arm}_arm_controller']
        assert combined[f'{arm}_gripper_controller']['joints'] == [f'{arm}_gripper_joint']
        assert _load_yaml('ompl_planning.yaml')[f'{arm}_pregrasp_open']['longest_valid_segment_fraction'] == .001
        assert _load_yaml('ompl_planning.yaml')[f'{arm}_return_close']['longest_valid_segment_fraction'] == .001


def test_mock_ros2_control_matches_moveit_controller_contract() -> None:
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

    mock_root = ET.parse(CONFIG_ROOT / 'cleany_mock.urdf.xacro').getroot()
    plugin = mock_root.find('.//hardware/plugin')
    assert plugin is not None
    assert plugin.text == 'mock_components/GenericSystem'
    configured_mock_joints = {
        element.attrib['joint_name']
        for element in mock_root.iter()
        if element.tag.endswith('cleany_mock_joint')
        and 'joint_name' in element.attrib
    }
    assert configured_mock_joints == _all_modeled_joints()


def test_optional_rviz_uses_the_moveit_model_and_selected_clock() -> None:
    source = (PACKAGE_ROOT / 'launch' / 'move_group.launch.py').read_text(
        encoding='utf-8'
    )

    assert "DeclareLaunchArgument('use_rviz', default_value='false')" in source
    assert "mappings={'include_head_camera': 'false'}" in source
    assert "condition=IfCondition(use_rviz)" in source
    assert 'moveit_config.robot_description,' in source
    assert 'moveit_config.robot_description_semantic,' in source
    assert 'moveit_config.robot_description_kinematics,' in source
    assert "'use_sim_time': ParameterValue(" in source


def test_move_group_exit_uses_a_launch_action_not_a_raw_event() -> None:
    from launch import LaunchDescriptionEntity

    namespace = runpy.run_path(str(PACKAGE_ROOT / 'launch/move_group.launch.py'))
    shutdown = namespace['Shutdown'](reason='test required process exit')
    assert isinstance(shutdown, LaunchDescriptionEntity)


def test_depth_octomap_and_rviz_share_sensor_cloud_contract() -> None:
    config = _load_yaml('depth_octomap.yaml')['/**']['ros__parameters']
    assert config['octomap_frame'] == 'base_link'
    assert config['octomap_resolution'] == 0.01
    sensor = config['depth_cloud']
    assert sensor['sensor_plugin'] == (
        'occupancy_map_monitor/PointCloudOctomapUpdater'
    )
    assert sensor['point_cloud_topic'] == '/perception/scene_cloud'
    assert sensor['filtered_cloud_topic'] == '/perception/scene_cloud_filtered'
    # Self-filter margin must cover a voxel half-diagonal plus model error.
    assert sensor['padding_offset'] >= (
        3 ** 0.5 * config['octomap_resolution'] / 2 + 0.005
    )
    displays = _load_yaml('moveit.rviz')['Visualization Manager']['Displays']
    cloud = next(d for d in displays
                 if d['Class'] == 'rviz_default_plugins/PointCloud2')
    assert cloud['Topic']['Value'] == sensor['point_cloud_topic']
    manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    assert 'moveit_ros_perception' in {
        dependency.text for dependency in manifest.findall('exec_depend')
    }
