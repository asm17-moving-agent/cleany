"""Physical RGB-D source for the existing Cleany snapshot interface."""

from launch import LaunchDescription
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def setup(context):
    selected = LaunchConfiguration('rgb_intrinsics_file').perform(context)
    if not selected:
        selection = Path(get_package_share_directory('cleany_perception')) / 'config/head_rgb_intrinsics.yaml'
        selected = yaml.safe_load(selection.read_text())['intrinsics_file']
    remaps = [('/tf', LaunchConfiguration('tf_topic')),
              ('/tf_static', LaunchConfiguration('tf_static_topic')),
              ('~/depth/image_rect_raw', '/camera/raw_depth/image_raw'),
              ('~/depth/camera_info', '/camera/raw_depth/camera_info')]
    parameters = [LaunchConfiguration('config_file'), {'serial_no': ParameterValue(
        LaunchConfiguration('serial_no'), value_type=str)}]
    if selected:
        path = Path(selected).expanduser().resolve(strict=True)
        parameters.append({'align_depth.enable': False})
        remaps.extend([('~/color/image_raw', '/camera/sdk/color/image_raw'),
                       ('~/color/camera_info', '/camera/sdk/color/camera_info'),
                       ('~/color/image_raw/compressed', '/camera/sdk/color/image_raw/compressed')])
    else:
        remaps.extend([('~/aligned_depth_to_color/image_raw', '/camera/depth/image_raw'),
                       ('~/aligned_depth_to_color/camera_info', '/camera/depth/camera_info')])
    nodes = [Node(package='realsense2_camera', executable='realsense2_camera_node',
                  name='camera', namespace='', output='screen', parameters=parameters,
                  remappings=remaps)]
    if selected:
        nodes.append(Node(package='cleany_perception', executable='calibrated_rgbd',
                          output='screen', parameters=[{'intrinsics_file': str(path)}],
                          remappings=[('/tf', LaunchConfiguration('tf_topic')),
                                      ('/tf_static', LaunchConfiguration('tf_static_topic'))]))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('tf_topic', default_value='/tf'),
        DeclareLaunchArgument('tf_static_topic', default_value='/tf_static'),
        DeclareLaunchArgument('rgb_intrinsics_file', default_value=''),
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
        OpaqueFunction(function=setup),
    ])
