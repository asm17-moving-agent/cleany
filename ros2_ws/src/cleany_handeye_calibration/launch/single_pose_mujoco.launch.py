from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package_name: str, name: str) -> str:
    return str(
        Path(get_package_share_directory(package_name)) / 'launch' / name
    )


def generate_launch_description() -> LaunchDescription:
    request_file = LaunchConfiguration('request_file')
    scene_path = LaunchConfiguration('scene_path')
    headless = LaunchConfiguration('headless')
    sim_speed_factor = LaunchConfiguration('sim_speed_factor')

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        launch_arguments={
            'scene_path': scene_path,
            'headless': headless,
            'sim_speed_factor': sim_speed_factor,
        }.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_moveit_config', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'allow_trajectory_execution': 'true',
        }.items(),
    )
    collision_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file(
                'cleany_moveit_config',
                'handeye_collision_scene.launch.py',
            )
        )
    )
    orchestrator = Node(
        package='cleany_handeye_calibration',
        executable='single_pose_calibration',
        parameters=[{'request_file': request_file}],
        output='screen',
    )
    default_scene = str(
        Path(get_package_share_directory('cleany_mujoco_sim'))
        / 'scenes'
        / 'handeye.xml.in'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'request_file',
                description=(
                    'Absolute path to a fully materialized single-pose '
                    'runtime JSON file.'
                ),
            ),
            DeclareLaunchArgument(
                'scene_path',
                default_value=default_scene,
                description='Fixed-base calibration MuJoCo scene template.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description=(
                    'Show the MuJoCo viewer for an operator-observed '
                    'calibration run. Automated tests override this to true.'
                ),
            ),
            DeclareLaunchArgument(
                'sim_speed_factor', default_value='1.0'
            ),
            backend,
            move_group,
            collision_scene,
            orchestrator,
        ]
    )
