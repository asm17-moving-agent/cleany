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
    branch = humble.find(".//ReactiveFallback[@name='PathClearOrReplan']")
    assert branch[-1].tag == 'ComputePathToPose'
    assert float(humble.find('.//YieldWait').get('wait_duration')) <= .2


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
        assert result['voxel_guard']['ros__parameters']['voxel_topic'] == 'obstacle_memory/local_voxels'
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


def test_reactive_route_keeps_monitor_and_one_final_owner_without_voxel_dependency():
    source = parameters()
    before = deepcopy(source['collision_monitor']['ros__parameters'])
    result = configure_safety(source, safety_mode='reactive', voxel_enabled=False, memory_enabled=False)
    assert result['pan_motion_gate']['ros__parameters']['observation_only'] is True
    monitor = result['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == 'nav2/cmd_vel'
    assert monitor['cmd_vel_out_topic'] == result['reactive_guard']['ros__parameters']['command_topic']
    assert result['reactive_guard']['ros__parameters']['output_topic'] == 'cmd_vel'
    for key in ('StopZone', 'SlowZone', 'source_timeout', 'observation_sources'):
        assert monitor[key] == before[key]
    assert result['obstacle_memory']['ros__parameters'] == source.get('obstacle_memory', {}).get('ros__parameters', {})
    with pytest.raises(ValueError, match='replaces the serial voxel'):
        configure_safety(source, safety_mode='reactive', voxel_enabled=True)


def test_humble_preserves_three_point_stop_boundary_and_bt_registration():
    source = parameters()
    before = deepcopy(source)
    result = adapt_nav2_parameters(source, 'humble')
    zone = result['collision_monitor']['ros__parameters']['StopZone']
    assert len(zone['points']) == 8
    # Jazzy triggers at points >= min_points, Humble at points > max_points.
    assert 'min_points' not in zone
    assert zone['max_points'] == before['collision_monitor']['ros__parameters']['StopZone']['min_points'] - 1 == 2
    assert result['planner_server']['ros__parameters']['GridBased']['plugin'] == 'nav2_smac_planner/SmacPlannerLattice'
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
    body.write_text(yaml.safe_dump({'park_joints': {'head_pan_joint': 0.},
                                  'simulation_excluded_link_roots': ['left_shoulder_yaw_link']}))
    with pytest.raises(ValueError, match='link exclusions'):
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


def test_simulation_geometry_removes_descendants_and_rejects_physical_use():
    from cleany_navigation.urdf_geometry import apply_simulation_geometry
    from xml.etree import ElementTree as ET
    source = '<robot><link name="base_link"/><link name="arm"/><link name="wrist"/><link name="head"/><joint name="shoulder"><parent link="base_link"/><child link="arm"/></joint><joint name="wrist_joint"><parent link="arm"/><child link="wrist"/></joint><joint name="head_joint"><parent link="base_link"/><child link="head"/></joint></robot>'
    profile = dict(simulation_excluded_link_roots=['arm'], park_joints=dict(shoulder=0., wrist_joint=0., head_joint=0.), simulation_park_limits=dict(wrist_joint=[-2, 2]))
    with pytest.raises(ValueError, match='physical robots'):
        apply_simulation_geometry(ET.fromstring(source), profile, allowed=False)
    urdf = ET.fromstring(source)
    filtered = apply_simulation_geometry(urdf, profile, allowed=True)
    assert {l.get('name') for l in urdf.findall('link')} == {'base_link', 'head'}
    assert filtered['park_joints'] == {'head_joint': 0.}
    assert filtered['simulation_park_limits'] == {}
    assert 'shoulder' in profile['park_joints']


def test_localization_calibration_preserves_motion_and_safety_and_is_isolated():
    from cleany_navigation.runtime_config import merge_localization_parameters
    target = parameters()
    prior = deepcopy(target)
    overlay = {'amcl': {'ros__parameters': {'sigma_hit': .04, 'alpha3': .002}}}
    merge_localization_parameters(target, overlay)
    for node, value in prior.items():
        if node != 'amcl':
            assert target[node] == value
    assert target['amcl']['ros__parameters']['initial_pose'] == prior['amcl']['ros__parameters']['initial_pose']
    overlay['amcl']['ros__parameters']['sigma_hit'] = .4
    assert target['amcl']['ros__parameters']['sigma_hit'] == .04


@pytest.mark.parametrize('overlay', [{}, {'controller_server': {'ros__parameters': {}}},
                                    {'amcl': {'ros__parameters': []}},
                                    {'amcl': {'ros__parameters': {}, 'other': {}}}])
def test_localization_profile_cannot_silently_replace_control_or_malformed_parameters(overlay):
    from cleany_navigation.runtime_config import merge_localization_parameters
    with pytest.raises(ValueError, match='localization overlay'):
        merge_localization_parameters(parameters(), overlay)


def test_shared_map_replaces_depth_voxel_build_but_preserves_lidar_and_static_map():
    source = parameters()
    merge_parameters(source, load_parameters(PACKAGE/'config/obstacle_memory.yaml'))
    before = deepcopy(source)
    result = configure_safety(source)
    assert result['voxel_guard']['ros__parameters']['voxel_topic'] == 'obstacle_memory/local_voxels'
    for name in ('local_costmap', 'global_costmap'):
        cm = result[name][name]['ros__parameters']
        assert 'depth_layer' not in cm['plugins']
        assert 'obstacle_layer' in cm['plugins']
        assert cm['memory_layer']['map_topic'] == '/obstacle_memory/navigation_grid'
        assert cm['obstacle_layer'] == before[name][name]['ros__parameters']['obstacle_layer']
    assert source == before
    assert result['obstacle_memory']['ros__parameters']['projection_max_z'] == before['local_costmap']['local_costmap']['ros__parameters']['depth_layer']['max_obstacle_height']
    disabled = configure_safety(source, memory_enabled=False, voxel_enabled=False)
    assert 'depth_layer' in disabled['local_costmap']['local_costmap']['ros__parameters']['plugins']


def test_default_omni_lattice_mppi_survives_humble_layering():
    """The selected motion layer must not restore DWB or an unsupported plugin name."""
    doc = parameters()
    merge_parameters(doc, load_parameters(PACKAGE/'config/nav2_omni_motion.yaml'))
    adapted = adapt_nav2_parameters(doc, 'humble')
    assert adapted['planner_server']['ros__parameters']['GridBased']['plugin'] == (
        'nav2_smac_planner/SmacPlannerLattice')
    assert 'w_traversal_cost' not in adapted['planner_server']['ros__parameters']['GridBased']
    follow = adapted['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'cleany_navigation::MPPIController'
    assert follow['motion_model'] == 'Omni'
    assert follow['vy_max'] > 0
    assert follow['PathAlignCritic']['use_path_orientations'] is False
    assert follow['PathAlignCritic']['offset_from_furthest'] == 2
    assert follow['CostCritic']['consider_footprint'] is True
    assert follow['PathAngleCritic']['enabled'] is False


@pytest.mark.parametrize('omni_layer', [False, True])
def test_default_lattice_file_matches_costmap_and_supports_omni(omni_layer):
    """Both launch modes resolve the actual regenerated primitive without stock fallback."""
    import hashlib
    import json
    from cleany_navigation.runtime_config import resolve_default_lattice
    doc = parameters()
    if omni_layer:
        merge_parameters(doc, load_parameters(PACKAGE/'config/nav2_omni_motion.yaml'))
    primitive = PACKAGE/'config/lattice/omni-r10cm.json'
    resolve_default_lattice(doc, primitive)
    adapted = adapt_nav2_parameters(doc, 'humble')
    planner = adapted['planner_server']['ros__parameters']['GridBased']
    assert planner['lattice_filepath'] == str(primitive.resolve())
    assert planner['plugin'] == 'nav2_smac_planner/SmacPlannerLattice'
    assert 'w_traversal_cost' not in planner
    metadata = json.loads(primitive.read_text())['lattice_metadata']
    assert metadata['turning_radius'] == 0.1
    assert metadata['motion_model'] == 'omni'
    assert metadata['num_of_headings'] == 16
    assert metadata['number_of_trajectories'] == 144
    assert hashlib.sha256(primitive.read_bytes()).hexdigest() == (
        '01165b98a42f01c0676f9ee6e691a9ed27b72dc7413c18a3a92b830ca3f2756e')
    for name in ('local_costmap', 'global_costmap'):
        assert doc[name][name]['ros__parameters']['resolution'] == metadata['grid_resolution']
    assert adapted['controller_server']['ros__parameters']['FollowPath']['motion_model'] == 'Omni'


def test_lattice_resolution_preserves_custom_file_and_other_planners(tmp_path):
    from cleany_navigation.runtime_config import resolve_default_lattice
    doc = parameters()
    planner = doc['planner_server']['ros__parameters']['GridBased']
    planner['lattice_filepath'] = '/custom/robot-primitive.json'
    before = deepcopy(doc)
    resolve_default_lattice(doc, tmp_path/'absent.json')
    assert doc == before
    planner['plugin'] = 'nav2_theta_star_planner/ThetaStarPlanner'
    planner.pop('lattice_filepath')
    before = deepcopy(doc)
    resolve_default_lattice(doc, tmp_path/'absent.json')
    assert doc == before
    planner['plugin'] = 'nav2_smac_planner/SmacPlannerLattice'
    with pytest.raises(ValueError, match='Packaged Lattice primitive not found'):
        resolve_default_lattice(doc, tmp_path/'absent.json')
