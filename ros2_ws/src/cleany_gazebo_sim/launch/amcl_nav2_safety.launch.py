"""Jazzy evaluation: LiDAR/depth costmap and monitored Nav2 velocity output."""

from pathlib import Path
from tempfile import mkdtemp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
import yaml


def _merge(target: dict, overlay: dict) -> None:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = value


def _launch(context: LaunchContext) -> list:
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    base = Path(LaunchConfiguration('params_file').perform(context))
    safety = Path(LaunchConfiguration('safety_params_file').perform(context))
    parameters = yaml.safe_load(base.read_text())
    overlay = yaml.safe_load(safety.read_text())
    camera = overlay.pop('depth_camera_tf')
    _merge(parameters, overlay)
    if LaunchConfiguration('axis_motion').perform(context).lower() == 'true':
        axis_path = Path(LaunchConfiguration('axis_params_file').perform(context))
        axis = yaml.safe_load(axis_path.read_text())
        planner = axis['planner_server']['ros__parameters']['GridBased']
        if not planner.get('lattice_filepath'):
            planner['lattice_filepath'] = str(Path(get_package_share_directory('nav2_smac_planner')) /
                'sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json')
        _merge(parameters, axis)
    if 'pan_pivot' in camera:
        parameters['collision_monitor']['ros__parameters']['cmd_vel_in_topic'] = '/pan_ready/cmd_vel'
        parameters['depth_clearance']['ros__parameters']['self_filter_profile'] = str(share/'config/cad_frame.yaml')
    directory = Path(mkdtemp(prefix='cleany-nav2-safety-'))
    merged = directory / 'nav2.yaml'
    merged.write_text(yaml.safe_dump(parameters, sort_keys=False))
    sim_time = LaunchConfiguration('use_sim_time')
    return [
        Node(
            package='cleany_gazebo_sim', executable='depth_clearance',
            name='depth_clearance', output='screen',
            parameters=[str(merged), {'use_sim_time': sim_time}],
        ),
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/nav2/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch/amcl_nav2.launch.py')),
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': str(merged),
                    'use_sim_time': sim_time,
                    'autostart': LaunchConfiguration('autostart'),
                }.items(),
            ),
        ]),
        *([Node(package='ros_gz_bridge', executable='parameter_bridge',
                name='head_pan_bridge', output='screen',
                arguments=['/model/cleany_mecanum/head_pan/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'],
                remappings=[('/model/cleany_mecanum/head_pan/cmd_pos', '/head_pan/command')]),
            Node(package='cleany_gazebo_sim', executable='pan_motion_gate',
                name='pan_motion_gate', output='screen',
                parameters=[str(share/'config/pan_motion_gate.yaml'), camera, {'use_sim_time': sim_time}])]
          if 'pan_pivot' in camera else [
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='depth_camera_tf', output='screen',
            parameters=[{'use_sim_time': sim_time}],
            arguments=[
                '--frame-id', 'base_link', '--child-frame-id', 'head_camera_depth_frame',
                *[item for name, value in zip(
                    ('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                    camera['translation'] + camera['rotation_rpy'],
                ) for item in ('--' + name, str(value))],
            ],
        )
        ]),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor',
            name='collision_monitor', output='screen',
            parameters=[str(merged), {'use_sim_time': sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_safety', output='screen',
            parameters=[{'use_sim_time': sim_time},
                        {'autostart': LaunchConfiguration('autostart')},
                        {'node_names': ['collision_monitor']}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('axis_motion', default_value='true'),
        DeclareLaunchArgument('axis_params_file', default_value=str(share / 'config/nav2_axis_motion.yaml')),
        DeclareLaunchArgument('map', description='Map YAML matching the simulated scene.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/nav2_amcl.yaml')),
        DeclareLaunchArgument('safety_params_file', default_value=str(share / 'config/nav2_safety.yaml')),
        OpaqueFunction(function=_launch),
    ])
