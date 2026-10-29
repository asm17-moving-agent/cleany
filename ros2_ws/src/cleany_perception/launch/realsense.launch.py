"""Physical RGB-D source for the existing Cleany snapshot interface."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='RealSense serial; empty selects an available device.',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cleany_perception'),
                'config', 'realsense.yaml',
            ]),
        ),
        Node(
            package='realsense2_camera', executable='realsense2_camera_node',
            name='camera', namespace='', output='screen',
            parameters=[LaunchConfiguration('config_file'), {
                'serial_no': ParameterValue(
                    LaunchConfiguration('serial_no'), value_type=str,
                ),
            }],
            remappings=[
                ('/tf', LaunchConfiguration('tf_topic')),
                ('/tf_static', LaunchConfiguration('tf_static_topic')),
                ('~/depth/image_rect_raw', '/camera/raw_depth/image_raw'),
                ('~/depth/camera_info', '/camera/raw_depth/camera_info'),
                ('~/aligned_depth_to_color/image_raw',
                 '/camera/depth/image_raw'),
                ('~/aligned_depth_to_color/camera_info',
                 '/camera/depth/camera_info'),
            ],
        ),
    ])
