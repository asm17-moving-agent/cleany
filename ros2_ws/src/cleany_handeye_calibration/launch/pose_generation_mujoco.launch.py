from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package_name: str, name: str) -> str:
    return str(
        Path(get_package_share_directory(package_name)) / 'launch' / name
    )


def generate_launch_description() -> LaunchDescription:
    handeye_share = Path(
        get_package_share_directory('cleany_handeye_calibration')
    )
    sim_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    scene_path = LaunchConfiguration('scene_path')
    headless = LaunchConfiguration('headless')
    sim_speed_factor = LaunchConfiguration('sim_speed_factor')
    generator = Node(
        package='cleany_handeye_calibration',
        executable='generate_pose_manifest',
        parameters=[
            {
                'profile': LaunchConfiguration('profile'),
                'scene_path': scene_path,
                'output_directory': LaunchConfiguration(
                    'output_directory'
                ),
                'artifact_root': LaunchConfiguration('artifact_root'),
                'repository_root': LaunchConfiguration('repository_root'),
                'run_id': LaunchConfiguration('run_id'),
            }
        ],
        output='screen',
        additional_env={'MUJOCO_GL': 'egl'},
        on_exit=Shutdown(reason='pose generation finished'),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'profile',
                default_value=str(
                    handeye_share / 'config' / 'pose_generation.mujoco.yaml'
                ),
            ),
            DeclareLaunchArgument(
                'scene_path',
                default_value=str(sim_share / 'scenes' / 'handeye.xml.in'),
            ),
            DeclareLaunchArgument('output_directory'),
            DeclareLaunchArgument('artifact_root'),
            DeclareLaunchArgument('repository_root'),
            DeclareLaunchArgument(
                'run_id',
                default_value='mujoco_seed_20260810',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='true',
                description=(
                    'Pose generation is a preparation pass; the subsequent '
                    'calibration run opens the viewer.'
                ),
            ),
            DeclareLaunchArgument('sim_speed_factor', default_value='1.0'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file(
                        'cleany_mujoco_sim', 'handeye_backend.launch.py'
                    )
                ),
                launch_arguments={
                    'scene_path': scene_path,
                    'headless': headless,
                    'sim_speed_factor': sim_speed_factor,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file(
                        'cleany_moveit_config', 'move_group.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'allow_trajectory_execution': 'true',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _launch_file(
                        'cleany_moveit_config',
                        'handeye_collision_scene.launch.py',
                    )
                )
            ),
            generator,
        ]
    )
