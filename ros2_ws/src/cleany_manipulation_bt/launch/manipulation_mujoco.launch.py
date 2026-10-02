"""One-object BT Action execution using the existing sensor-driven sorting stack."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    skill = Path(get_package_share_directory('cleany_skill_executor'))
    bt = Path(get_package_share_directory('cleany_manipulation_bt'))
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_image_view', default_value='false'),
        DeclareLaunchArgument('bt_database_path', default_value=str(
            Path(os.environ.get('XDG_STATE_HOME') or Path.home() / '.local/state')
            / 'cleany/manipulation_mujoco/executions.sqlite3')),
        DeclareLaunchArgument('bt_monitor_port', default_value='1667'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.08'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.08'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            skill / 'launch/study_cafe_nearest_grasp_demo.launch.py')), launch_arguments={
                'sorting_bins_config': str(bt / 'config/manipulation_bins.yaml'),
                'coordinator_parameters': str(bt / 'config/manipulation_mujoco.yaml'),
                'manipulation_bt': 'true', 'sorting_mode': 'true', 'start_simulator': 'true',
                'use_sim_time': 'true', 'plan_only': 'false', 'sensor_scene': 'true',
                'perception_detector_type': 'yoloe_gemini', 'perception_segmenter_type': 'yoloe_seg',
                'sorting_use_wrist_camera': 'true', 'sorting_verify_placement': 'true',
                'headless': LaunchConfiguration('headless'), 'use_rviz': LaunchConfiguration('use_rviz'),
                'use_image_view': LaunchConfiguration('use_image_view'),
                'bt_database_path': LaunchConfiguration('bt_database_path'),
                'bt_monitor_port': LaunchConfiguration('bt_monitor_port'),
                'velocity_scaling': LaunchConfiguration('velocity_scaling'),
                'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
            }.items()),
    ])
