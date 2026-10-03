from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest
import yaml

from cleany_navigation.runtime_config import (adapt_behavior_tree, adapt_nav2_parameters, configure_safety,
    load_parameters, merge_parameters, set_sim_time, validate_robot_parameters)
from cleany_navigation.urdf_geometry import forward_kinematics

PACKAGE = Path(__file__).resolve().parents[1]


def test_bt_compatibility_preserves_control_flow_and_humble_ports():
    xml = (PACKAGE/'config/behavior_trees/wait_before_detour.xml').read_text()
    humble = ET.fromstring(adapt_behavior_tree(xml, 'humble'))
    jazzy = ET.fromstring(adapt_behavior_tree(xml, 'jazzy'))
    assert [node.tag for node in humble.iter()] == [node.tag for node in jazzy.iter()]
    assert 'error_code_id' not in humble.find('.//ComputePathToPose').attrib
    assert 'error_code_id' in jazzy.find('.//ComputePathToPose').attrib
    assert humble.find('.//YieldWait').get('wait_duration') == '15.0'


def parameters():
    result = load_parameters(PACKAGE/'config/nav2_amcl.yaml')
    merge_parameters(result, load_parameters(PACKAGE/'config/nav2_safety.yaml'))
    return result


@pytest.mark.parametrize('pan,memory,voxel', [(True, True, True), (False, True, True),
                                            (False, False, False), (True, True, False)])
def test_serial_velocity_route_has_exactly_one_final_stage(pan, memory, voxel):
    result = configure_safety(parameters(), pan_enabled=pan, memory_enabled=memory, voxel_enabled=voxel)
    monitor = result['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == ('pan_ready/cmd_vel' if pan else 'nav2/cmd_vel')
    assert monitor['cmd_vel_out_topic'] == ('safety_2d/cmd_vel' if voxel else 'cmd_vel')
    if voxel:
        assert result['voxel_guard']['ros__parameters']['command_topic'] == monitor['cmd_vel_out_topic']
        assert result['voxel_guard']['ros__parameters']['output_topic'] == 'cmd_vel'
        assert result['obstacle_memory']['ros__parameters']['publish_3d'] is True
    if memory:
        for name in ('global_costmap', 'local_costmap'):
            costmap = result[name][name]['ros__parameters']
            assert costmap['memory_layer']['map_topic'] == '/obstacle_memory/grid'
            assert costmap['use_maximum'] is True
            assert costmap['plugins'].index('memory_layer') < costmap['plugins'].index('inflation_layer')


def test_voxel_without_observation_memory_is_rejected():
    with pytest.raises(ValueError, match='requires obstacle_memory'):
        configure_safety(parameters(), memory_enabled=False)


def test_humble_preserves_three_point_stop_boundary_and_bt_registration():
    source = parameters()
    before = deepcopy(source)
    result = adapt_nav2_parameters(source, 'humble')
    zone = result['collision_monitor']['ros__parameters']['StopZone']
    assert len(zone['points']) == 8
    assert zone['max_points'] == 2
    assert [n > zone['max_points'] for n in (2, 3, 4)] == [False, True, True]
    assert result['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_navfn_planner/NavfnPlanner'
    assert result['controller_server']['ros__parameters']['progress_checker_plugin'] == 'progress_checker'
    assert {'nav2_follow_path_action_bt_node', 'nav2_truncate_path_local_action_bt_node',
            'cleany_yield_wait_bt_node'} <= set(result['bt_navigator']['ros__parameters']['plugin_lib_names'])
    assert source == before
    assert adapt_nav2_parameters(source, 'jazzy') == source


def test_clock_override_reaches_costmaps_and_child_nodes():
    source = parameters()
    set_sim_time(source, True)
    set_sim_time(source, False)
    assert source['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is False
    assert source['collision_monitor']['ros__parameters']['use_sim_time'] is False
    with pytest.raises(ValueError, match='Unsupported'):
        adapt_nav2_parameters(source, 'rolling')


@pytest.mark.parametrize('distro', ['humble', 'jazzy'])
def test_yaml_vertex_lists_become_ros_compatible_polygon_parameters(distro):
    source = parameters()
    points = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]]
    source['local_costmap']['local_costmap']['ros__parameters']['footprint'] = points
    source['collision_monitor']['ros__parameters']['StopZone']['points'] = points
    result = adapt_nav2_parameters(source, distro)
    assert isinstance(result['local_costmap']['local_costmap']['ros__parameters']['footprint'], str)
    assert isinstance(result['collision_monitor']['ros__parameters']['StopZone']['points'],
                      list if distro == 'humble' else str)
    assert source['collision_monitor']['ros__parameters']['StopZone']['points'] == points


def robot_parameters(tmp_path):
    body = tmp_path/'body.yaml'
    body.write_text(yaml.safe_dump({'park_joints': {'head_pan_joint': 0.}}))
    result = parameters()
    result['voxel_guard'] = {'ros__parameters': dict(profile='body.yaml', max_speed=.1,
        max_yaw_rate=.2, reaction_s=.2, linear_deceleration=.4, angular_deceleration=.6)}
    result['obstacle_memory'] = {'ros__parameters': {'floor_z': -.38}}
    result['depth_clearance']['ros__parameters'].update(camera_frame='head_camera_depth_frame',
        optical_frame='head_camera_depth_optical_frame')
    return result, body


def test_real_profile_requires_geometry_and_resolves_relative_body_path(tmp_path):
    robot, body = robot_parameters(tmp_path)
    validate_robot_parameters(robot, tmp_path, use_sim_time=False)
    assert robot['voxel_guard']['ros__parameters']['profile'] == str(body)
    del robot['obstacle_memory']['ros__parameters']['floor_z']
    with pytest.raises(ValueError, match='floor_z'):
        validate_robot_parameters(robot, tmp_path, use_sim_time=False)


@pytest.mark.parametrize('key', ['max_speed', 'linear_deceleration', 'angular_deceleration'])
def test_invalid_braking_parameters_fail_before_launch(tmp_path, key):
    robot, _ = robot_parameters(tmp_path)
    robot['voxel_guard']['ros__parameters'][key] = float('nan')
    with pytest.raises(ValueError, match=key):
        validate_robot_parameters(robot, tmp_path, use_sim_time=False)


def test_simulation_depth_and_joint_overrides_cannot_enter_real_profile(tmp_path):
    robot, body = robot_parameters(tmp_path)
    robot['depth_clearance']['ros__parameters']['positive_infinity_is_free'] = True
    with pytest.raises(ValueError, match='depth clearing'):
        validate_robot_parameters(robot, tmp_path, use_sim_time=False)
    validate_robot_parameters(robot, tmp_path, use_sim_time=True)
    robot['depth_clearance']['ros__parameters']['positive_infinity_is_free'] = False
    body.write_text(yaml.safe_dump({'park_joints': {'head_pan_joint': 0.},
                                  'simulation_park_limits': {'head_pan_joint': [-4, 4]}}))
    with pytest.raises(ValueError, match='joint limit'):
        validate_robot_parameters(robot, tmp_path, use_sim_time=False)


def test_shared_fk_preserves_input_and_enforces_real_joint_limits():
    robot = ET.fromstring('''<robot><joint name="arm" type="revolute">
        <parent link="base_link"/><child link="arm_link"/><origin xyz="1 0 0"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint></robot>''')
    before = ET.tostring(robot)
    pose = forward_kinematics(robot, {'arm': .5})['arm_link']
    np.testing.assert_allclose(pose[:3, 3], [1, 0, 0])
    assert ET.tostring(robot) == before
    with pytest.raises(ValueError, match='outside limits'):
        forward_kinematics(robot, {'arm': -2})


def test_longitudinal_lattice_removes_curves_and_requires_each_heading():
    from cleany_navigation.runtime_config import longitudinal_lattice
    source = {'lattice_metadata': {'num_of_headings': 2, 'number_of_trajectories': 6},
              'primitives': []}
    for heading in range(2):
        for end, length, arc, straight in [(heading, .15, 0., .15),
                                          (1-heading, 0., 0., 0.),
                                          (1-heading, .4, .3, .1)]:
            source['primitives'].append(dict(trajectory_id=len(source['primitives']),
                start_angle_index=heading, end_angle_index=end, trajectory_length=length,
                arc_length=arc, straight_length=straight))
    result = longitudinal_lattice(source)
    assert result['lattice_metadata']['number_of_trajectories'] == 4
    assert [p['trajectory_id'] for p in result['primitives']] == list(range(4))
    assert all(p['arc_length'] == 0. for p in result['primitives'])
    assert len(source['primitives']) == 6
    source['primitives'] = source['primitives'][:3]
    with pytest.raises(ValueError, match='heading 1'):
        longitudinal_lattice(source)
