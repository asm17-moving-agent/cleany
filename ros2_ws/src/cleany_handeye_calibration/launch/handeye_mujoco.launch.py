from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_file(package_name: str, launch_file: str) -> str:
    return str(
        Path(get_package_share_directory(package_name))
        / 'launch'
        / launch_file
    )


def generate_launch_description() -> LaunchDescription:
    scene_path = LaunchConfiguration('scene_path')
    headless = LaunchConfiguration('headless')
    sim_speed_factor = LaunchConfiguration('sim_speed_factor')

    # The backend owns the one robot_state_publisher for this composition.
    # This launch intentionally adds only motion infrastructure; camera,
    # calibration scene, and orchestration are introduced by later commits.
    mujoco_backend = IncludeLaunchDescription(
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

    default_scene = str(
        Path(get_package_share_directory('cleany_mujoco_sim'))
        / 'scenes'
        / 'default.xml.in'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'scene_path',
                default_value=default_scene,
                description=(
                    'MuJoCo control scene XML or XML template used by the '
                    'motion-only hand-eye stack.'
                ),
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run MuJoCo without its native viewer.',
            ),
            DeclareLaunchArgument(
                'sim_speed_factor',
                default_value='1.0',
                description='MuJoCo simulation speed relative to wall time.',
            ),
            mujoco_backend,
            move_group,
        ]
    )
