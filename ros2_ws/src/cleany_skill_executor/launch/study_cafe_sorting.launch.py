"""Simulation-only learned RGB-D pick/sort/place workflow."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    skill = Path(get_package_share_directory('cleany_skill_executor'))
    sim = Path(get_package_share_directory('cleany_mujoco_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_image_view', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            skill / 'launch' / 'study_cafe_nearest_grasp_demo.launch.py'
        )), launch_arguments={
            'sorting_mode': 'true',
            'start_simulator': 'true',
            'use_sim_time': 'true',
            'plan_only': 'false',
            'sensor_scene': 'true',
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_image_view': LaunchConfiguration('use_image_view'),
            'sorting_bins_config': str(sim / 'config' / 'robot_top_bins.yaml'),
        }.items()),
    ])
