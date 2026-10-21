"""Simulation-only learned RGB-D pick/sort/place workflow."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from cleany_skill_executor.motion_runtime import motion_launch_arguments


def generate_launch_description():
    skill = Path(get_package_share_directory('cleany_skill_executor'))
    sim = Path(get_package_share_directory('cleany_mujoco_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('repository_root', default_value=str(Path(__file__).resolve().parents[4])),
        *motion_launch_arguments(),
        DeclareLaunchArgument('motion_backend', default_value='curobo_ik_moveit', choices=['moveit', 'curobo_ik_moveit']),
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_image_view', default_value='false', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            skill / 'launch' / 'study_cafe_nearest_grasp_demo.launch.py'
        )), launch_arguments={
            'sorting_mode': 'true',
            'repository_root': LaunchConfiguration('repository_root'),
            'motion_runtime': LaunchConfiguration('motion_runtime'),
            'motion_backend': LaunchConfiguration('motion_backend'),
            'curobo_python': LaunchConfiguration('curobo_python'),
            'start_simulator': 'true',
            'use_sim_time': 'true',
            'plan_only': 'false',
            'sensor_scene': 'true',
            'perception_detector_type': 'yoloe_gemini',
            'perception_segmenter_type': 'yoloe_seg',
            'sorting_use_wrist_camera': 'true',
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_image_view': LaunchConfiguration('use_image_view'),
            'sorting_bins_config': str(sim / 'config' / 'robot_top_bins.yaml'),
        }.items()),
    ])
