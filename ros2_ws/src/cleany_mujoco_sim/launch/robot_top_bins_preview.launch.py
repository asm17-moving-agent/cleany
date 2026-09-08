"""Stationary layout preview; intentionally starts no autonomous executor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = FindPackageShare('cleany_mujoco_sim')
    arguments = {
        'scene_path': PathJoinSubstitution([share, 'scenes', 'study_cafe_grasp_execution.xml.in']),
        'sorting_bins_config': PathJoinSubstitution([share, 'config', 'robot_top_bins.yaml']),
        'controller_config': PathJoinSubstitution([share, 'config', 'grasp_demo_ros2_controllers.yaml']),
        'headless': LaunchConfiguration('headless'),
        'sim_speed_factor': '1.0',
        'camera_name': 'head_realsense_rgb',
        'camera_frame_name': 'head_camera_rgb_optical_frame',
        'enable_camera_contract_adapter': 'false',
        'enable_gripper_controllers': 'true',
        'head_tilt_initial': '1.0',
    }
    joints = ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')
    for side, positions in (
        ('left', (-1.53, 3.35, 3.12, -1.63, 1.58, -0.35)),
        ('right', (1.58, 3.35, 3.12, -1.63, -1.58, -0.35)),
    ):
        arguments.update({f'{side}_{joint}_initial': str(value) for joint, value in zip(joints, positions)})
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'handeye_backend.launch.py'])),
            launch_arguments=arguments.items(),
        ),
    ])
