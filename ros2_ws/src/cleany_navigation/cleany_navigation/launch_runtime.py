"""Backend-independent launch assembly for localization, Nav2 and safety."""
from __future__ import annotations

import os
import json
from pathlib import Path
from tempfile import mkdtemp
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import yaml
import xacro

from cleany_navigation.runtime_config import (adapt_behavior_tree, adapt_nav2_parameters, configure_safety,
    configure_tof_route, load_parameters, merge_parameters, merge_localization_parameters, set_sim_time, validate_robot_parameters, longitudinal_lattice)
from cleany_navigation.bt_trace import describe_tree
from cleany_navigation.urdf_geometry import forward_kinematics, apply_simulation_geometry


def nav2_nodes(parameters: str, map_yaml: str, use_sim_time: bool, autostart: bool) -> list:
    nodes = [Node(package=package, executable=executable, name=name, output='screen',
        parameters=[parameters, {'use_sim_time': use_sim_time},
                    *([{'yaml_filename': map_yaml}] if name == 'map_server' else [])])
        for package, executable, name in [
            ('nav2_map_server', 'map_server', 'map_server'), ('nav2_amcl', 'amcl', 'amcl'),
            ('nav2_controller', 'controller_server', 'controller_server'),
            ('nav2_planner', 'planner_server', 'planner_server'),
            ('nav2_behaviors', 'behavior_server', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator')]]
    for suffix, names in [('localization', ['map_server', 'amcl']),
        ('navigation', ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator'])]:
        nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_'+suffix, output='screen', parameters=[
                {'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': names}]))
    return nodes


def _launch(context: LaunchContext, *, safety: bool) -> list:
    share = Path(get_package_share_directory('cleany_navigation'))
    directory = Path(mkdtemp(prefix='cleany-navigation-'))
    def argument(name: str) -> str:
        return LaunchConfiguration(name).perform(context)
    def flag(name: str) -> bool:
        value = argument(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'
    map_yaml = Path(argument('map')).expanduser().resolve()
    if not map_yaml.is_file():
        raise ValueError(f'Map YAML not found: {map_yaml}')
    sim_time, autostart = flag('use_sim_time'), flag('autostart')
    parameters = load_parameters(Path(argument('params_file')))
    localization_file = argument('localization_params_file')
    if localization_file:
        merge_localization_parameters(parameters, load_parameters(Path(localization_file)))
    robot_path = argument('robot_params_file')
    if safety:
        for filename in ('safety_params_file', 'pan_params_file', 'voxel_params_file', 'memory_params_file'):
            merge_parameters(parameters, load_parameters(Path(argument(filename))))
        motion_file = 'axis_params_file' if flag('axis_motion') else 'omni_params_file'
        merge_parameters(parameters, load_parameters(Path(argument(motion_file))))
        if not robot_path:
            raise ValueError('robot_params_file is required for navigation safety')
    if robot_path:
        path = Path(robot_path).expanduser().resolve()
        robot = load_parameters(path)
        if safety:
            validate_robot_parameters(robot, path.parent, use_sim_time=sim_time)
            guard = robot['voxel_guard']['ros__parameters']
            body = load_parameters(Path(guard['profile']))
            description = Path(get_package_share_directory('cleany_description'))/'urdf/cleany.urdf.xacro'
            urdf = ET.fromstring(xacro.process_file(str(description)).toxml())
            body = apply_simulation_geometry(urdf, body, allowed=sim_time and guard.get('allow_simulation_limits', False))
            forward_kinematics(urdf, body['park_joints'],
                body.get('simulation_park_limits') if guard.get('allow_simulation_limits', False) else None)
        merge_parameters(parameters, robot)
    if safety:
        pan, memory, voxel = flag('pan_enabled'), flag('memory_enabled'), flag('voxel_enabled')
        safety_mode = argument('safety_mode')
        if safety_mode == 'reactive' and not sim_time:
            raise ValueError('Reactive mode currently requires simulation; physical braking and sensor coverage are unvalidated')
        parameters = configure_safety(parameters, pan_enabled=pan, memory_enabled=memory,
            voxel_enabled=voxel, safety_mode=safety_mode, output_directory=argument('output_directory'))
        projection = parameters['depth_clearance']['ros__parameters']
        guard = parameters['voxel_guard']['ros__parameters']
        coverage_enabled = flag('tof_enabled') or parameters.get('depth_coverage', {}).get('ros__parameters', {}).get('enabled', False)
        if coverage_enabled:
            coverage = parameters.setdefault('depth_coverage', {}).setdefault('ros__parameters', {})
            for key in ('profile', 'allow_simulation_limits', 'margin', 'reaction_s',
                        'linear_deceleration', 'angular_deceleration', 'step_s', 'max_speed', 'max_yaw_rate'):
                if key in guard:
                    coverage[key] = guard[key]
            for key in ('input_topic', 'camera_info_topic', 'optical_frame', 'base_frame'):
                if key in projection:
                    coverage[key] = projection[key]
            coverage['source_timeout_s'] = parameters['pan_motion_gate']['ros__parameters']['source_timeout_s']
        if (voxel or safety_mode == 'reactive') and not projection.get('self_filter_profile'):
            projection['self_filter_profile'] = guard['profile']
        # All consumers agree on the backend's canonical odometry and depth.
        for name in ('controller_server', 'bt_navigator', 'pan_motion_gate', 'voxel_guard'):
            parameters[name]['ros__parameters']['odom_topic'] = 'odom'
        parameters['pan_motion_gate']['ros__parameters'].update(
            depth_topic=projection.get('output_topic', 'camera/head/depth/points'),
            depth_frame=projection['camera_frame'])
        parameters['obstacle_memory']['ros__parameters']['depth_topic'] = projection.get('output_topic', 'camera/head/depth/points')
        monitor = parameters['collision_monitor']['ros__parameters']
        monitor['depth']['topic'] = projection.get('output_topic', 'camera/head/depth/points')
        if safety_mode == 'reactive':
            parameters['reactive_guard']['ros__parameters'].update(odom_topic='odom',
                scan_topic=monitor['scan']['topic'], depth_topic=monitor['depth']['topic'],
                depth_frame=projection['camera_frame'], min_height=monitor['depth']['min_height'],
                max_height=monitor['depth']['max_height'])
        depth_layer = parameters['local_costmap']['local_costmap']['ros__parameters']['depth_layer']['depth']
        topic = projection.get('output_topic', 'camera/head/depth/points')
        depth_layer.update(topic='/'+projection.get('obstacle_topic', 'camera/head/depth/obstacle_points').lstrip('/'),
                           sensor_frame=projection['camera_frame'])
        clearing = parameters['local_costmap']['local_costmap']['ros__parameters']['depth_layer'].get('depth_clear')
        if clearing is not None:
            clearing.update(topic='/'+topic.lstrip('/'), sensor_frame=projection['camera_frame'])
        if not sim_time and (projection.get('positive_infinity_is_free', False)
            or projection.get('allow_simulation_limits', False) or guard.get('allow_simulation_limits', False)):
            raise ValueError('Simulation-only overrides cannot be used with real sensor time')
        planner = parameters['planner_server']['ros__parameters']['GridBased']
        if planner.get('plugin') == 'cleany_navigation::StraightFirstPlanner' and not planner.get('lattice_filepath'):
            lateral = not flag('axis_motion') or parameters['controller_server']['ros__parameters']['FollowPath'].get('axis_allow_lateral', False)
            model = 'omni' if lateral else 'diff'
            lattice = Path(get_package_share_directory('nav2_smac_planner'))/f'sample_primitives/5cm_resolution/0.5m_turning_radius/{model}/output.json'
            if not lattice.is_file():
                raise ValueError(f'Installed {model} lattice not found: {lattice}; set lattice_filepath')
            if not lateral:
                primitives = longitudinal_lattice(json.loads(lattice.read_text()))
                lattice = directory/'longitudinal_lattice.json'
                lattice.write_text(json.dumps(primitives))
            planner['lattice_filepath'] = str(lattice)
    # Lifecycle children and costmaps read the same clock settings from YAML.
    for name in ('bt_navigator_navigate_to_pose_rclcpp_node', 'bt_navigator_navigate_through_poses_rclcpp_node'):
        parameters.setdefault(name, {'ros__parameters': {}})
    tof_enabled = safety and flag('tof_enabled')
    if tof_enabled:
        parameters = configure_tof_route(parameters)
        coverage_enabled = True
        parameters['depth_coverage']['ros__parameters'].update(
            tof_coverage_enabled=True, tof_params_file=argument('tof_params_file'), update_period_s=.05)
    set_sim_time(parameters, sim_time)
    parameters = adapt_nav2_parameters(parameters, os.environ.get('ROS_DISTRO', ''))
    if safety:
        tree = directory/'wait_before_detour.xml'
        runtime_xml, manifest = describe_tree(adapt_behavior_tree(
            (share/'config/behavior_trees/wait_before_detour.xml').read_text(),
            os.environ.get('ROS_DISTRO', '')), str(tree))
        tree.write_text(runtime_xml)
        manifest_file = directory/'bt_manifest.json'
        manifest_file.write_text(json.dumps(manifest, ensure_ascii=False))
        parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(tree)
    merged = directory/'parameters.yaml'
    merged.write_text(yaml.safe_dump(parameters, sort_keys=False))
    nodes = nav2_nodes(str(merged), str(map_yaml), sim_time, autostart)
    if not safety:
        return nodes
    result = [GroupAction([SetRemap(src='cmd_vel', dst='nav2/cmd_vel'), *nodes]),
        Node(package='cleany_navigation', executable='nav2_bt_trace', name='nav2_bt_trace',
             output='screen', parameters=[{'use_sim_time': sim_time,
                 'manifest_file': str(manifest_file), 'xml_file': str(tree)}])]
    for executable, enabled in [('depth_clearance', True), ('pan_motion_gate', pan),
                                ('obstacle_memory', memory), ('voxel_guard', voxel),
                                ('reactive_guard', safety_mode == 'reactive'),
                                ('depth_coverage', coverage_enabled)]:
        if enabled:
            result.append(Node(package='cleany_navigation', executable=executable, name=executable,
                output='screen', parameters=[str(merged), {'use_sim_time': sim_time}],
                remappings=[('nav2/cmd_vel', 'motion/cmd_vel')] if tof_enabled else []))
    if tof_enabled:
        tof_file = argument('tof_params_file')
        for executable in ('tof_driver', 'proximity_guard', 'desk_approach'):
            result.append(Node(package='cleany_tof', executable=executable, name=executable,
                output='screen', parameters=[tof_file, {'use_sim_time': sim_time}]))
    result.extend([
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             output='screen', parameters=[str(merged), {'use_sim_time': sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_safety',
             output='screen', parameters=[{'use_sim_time': sim_time, 'autostart': autostart,
                                          'node_names': ['collision_monitor']}]),
    ])
    return result


def navigation_launch_description(*, safety: bool = True) -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_navigation'))
    arguments = [DeclareLaunchArgument('map', description='Prebuilt map YAML for the active backend.'),
        DeclareLaunchArgument('robot_params_file', default_value='', description='Robot geometry, frames and braking parameters.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/nav2_amcl.yaml')),
        DeclareLaunchArgument('localization_params_file', default_value='',
                             description='Optional AMCL-only calibration overlay; does not change safety/motion.')]
    if safety:
        arguments.append(DeclareLaunchArgument('tof_enabled', default_value='false'))
        arguments.append(DeclareLaunchArgument('tof_params_file', default_value=str(
            Path(get_package_share_directory('cleany_tof'))/'config/tof.yaml')))
        for name, filename in [('safety_params_file', 'nav2_safety.yaml'), ('axis_params_file', 'nav2_axis_motion.yaml'),
            ('pan_params_file', 'pan_motion_gate.yaml'), ('voxel_params_file', 'voxel_guard.yaml'),
            ('memory_params_file', 'obstacle_memory.yaml'), ('omni_params_file', 'nav2_omni_motion.yaml')]:
            arguments.append(DeclareLaunchArgument(name, default_value=str(share/'config'/filename)))
        for name in ('axis_motion', 'pan_enabled', 'memory_enabled', 'voxel_enabled'):
            arguments.append(DeclareLaunchArgument(name, default_value='false' if name == 'axis_motion' else 'true'))
        arguments.append(DeclareLaunchArgument('output_directory', default_value=''))
        arguments.append(DeclareLaunchArgument('safety_mode', default_value='serial', choices=['serial', 'reactive']))
    return LaunchDescription([*arguments, OpaqueFunction(function=lambda context: _launch(context, safety=safety))])
