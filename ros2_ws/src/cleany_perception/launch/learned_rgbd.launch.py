"""YOLOE-seg + Gemini on any aligned RGB-D / TF provider."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_perception'))
    profile_path = share / 'config' / 'yoloe_seg_gemini.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('enable_wrist_observation', default_value='false',
                             choices=['true', 'false']),
        DeclareLaunchArgument('model_profile', default_value=str(profile_path)),
        DeclareLaunchArgument(
            'model_directory', default_value=EnvironmentVariable(
                'CLEANY_MODEL_DIR',
                default_value=str(Path.home() / 'models'),
            ),
        ),
        DeclareLaunchArgument(
            'color_image_topic', default_value='/camera/color/image_raw'
        ),
        DeclareLaunchArgument(
            'color_info_topic', default_value='/camera/color/camera_info'
        ),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
        ),
        DeclareLaunchArgument(
            'depth_info_topic',
            default_value=LaunchConfiguration('color_info_topic'),
        ),
        Node(
            package='cleany_perception', executable='inspection_node',
            parameters=[
                str(share / 'config' / 'inspect_scene.yaml'),
                LaunchConfiguration('model_profile'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'model_directory': LaunchConfiguration('model_directory'),
                    'yoloe_device': LaunchConfiguration('device'),
                    'enable_wrist_observation': ParameterValue(
                        LaunchConfiguration('enable_wrist_observation'), value_type=bool),
                    **{name: LaunchConfiguration(name) for name in (
                        'color_image_topic', 'color_info_topic',
                        'depth_image_topic', 'depth_info_topic',
                    )},
                },
            ],
            output='screen',
        ),
    ])
