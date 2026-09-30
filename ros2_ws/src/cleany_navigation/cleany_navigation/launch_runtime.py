"""Backend-independent launch assembly for localization, Nav2 and safety."""
from __future__ import annotations

import os
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
    load_parameters, merge_parameters, set_sim_time, validate_robot_parameters)
from cleany_navigation.urdf_geometry import forward_kinematics


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
    robot_path = argument('robot_params_file')
    if safety:
        for filename in ('safety_params_file', 'pan_params_file', 'voxel_params_file', 'memory_params_file'):
            merge_parameters(parameters, load_parameters(Path(argument(filename))))
        if flag('axis_motion'):
            merge_parameters(parameters, load_parameters(Path(argument('axis_params_file'))))
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
            forward_kinematics(urdf, body['park_joints'],
                body.get('simulation_park_limits') if guard.get('allow_simulation_limits', False) else None)
        merge_parameters(parameters, robot)
    if safety:
        pan, memory, voxel = flag('pan_enabled'), flag('memory_enabled'), flag('voxel_enabled')
        parameters = configure_safety(parameters, pan_enabled=pan, memory_enabled=memory,
            voxel_enabled=voxel, output_directory=argument('output_directory'))
        projection = parameters['depth_clearance']['ros__parameters']
        guard = parameters['voxel_guard']['ros__parameters']
        if voxel and not projection.get('self_filter_profile'):
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
        depth_layer = parameters['local_costmap']['local_costmap']['ros__parameters']['depth_layer']['depth']
        topic = projection.get('output_topic', 'camera/head/depth/points')
        depth_layer.update(topic='/'+topic.lstrip('/'), sensor_frame=projection['camera_frame'])
        if not sim_time and (projection.get('positive_infinity_is_free', False)
            or projection.get('allow_simulation_limits', False) or guard.get('allow_simulation_limits', False)):
            raise ValueError('Simulation-only overrides cannot be used with real sensor time')
        parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(share/'config/behavior_trees/wait_before_detour.xml')
        planner = parameters['planner_server']['ros__parameters']['GridBased']
        if flag('axis_motion') and not planner.get('lattice_filepath'):
            lattice = Path(get_package_share_directory('nav2_smac_planner'))/'sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json'
            if not lattice.is_file():
                raise ValueError(f'Installed Omni lattice not found: {lattice}; set lattice_filepath')
            planner['lattice_filepath'] = str(lattice)
    # Lifecycle children and costmaps read the same clock settings from YAML.
    for name in ('bt_navigator_navigate_to_pose_rclcpp_node', 'bt_navigator_navigate_through_poses_rclcpp_node'):
        parameters.setdefault(name, {'ros__parameters': {}})
    set_sim_time(parameters, sim_time)
    parameters = adapt_nav2_parameters(parameters, os.environ.get('ROS_DISTRO', ''))
    directory = Path(mkdtemp(prefix='cleany-navigation-'))
    if safety:
        tree = directory/'wait_before_detour.xml'
        tree.write_text(adapt_behavior_tree((share/'config/behavior_trees/wait_before_detour.xml').read_text(),
                                           os.environ.get('ROS_DISTRO', '')))
        parameters['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(tree)
    merged = directory/'parameters.yaml'
    merged.write_text(yaml.safe_dump(parameters, sort_keys=False))
    nodes = nav2_nodes(str(merged), str(map_yaml), sim_time, autostart)
    if not safety:
        return nodes
    result = [GroupAction([SetRemap(src='cmd_vel', dst='nav2/cmd_vel'), *nodes])]
    for executable, enabled in [('depth_clearance', True), ('pan_motion_gate', pan),
                                ('obstacle_memory', memory), ('voxel_guard', voxel)]:
        if enabled:
            result.append(Node(package='cleany_navigation', executable=executable, name=executable,
                output='screen', parameters=[str(merged), {'use_sim_time': sim_time}]))
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
        DeclareLaunchArgument('params_file', default_value=str(share/'config/nav2_amcl.yaml'))]
    if safety:
        for name, filename in [('safety_params_file', 'nav2_safety.yaml'), ('axis_params_file', 'nav2_axis_motion.yaml'),
            ('pan_params_file', 'pan_motion_gate.yaml'), ('voxel_params_file', 'voxel_guard.yaml'),
            ('memory_params_file', 'obstacle_memory.yaml')]:
            arguments.append(DeclareLaunchArgument(name, default_value=str(share/'config'/filename)))
        for name in ('axis_motion', 'pan_enabled', 'memory_enabled', 'voxel_enabled'):
            arguments.append(DeclareLaunchArgument(name, default_value='true'))
        arguments.append(DeclareLaunchArgument('output_directory', default_value=''))
    return LaunchDescription([*arguments, OpaqueFunction(function=lambda context: _launch(context, safety=safety))])
