from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = (
        Path(get_package_share_directory('cleany_base_odometry'))
        / 'config' / 'encoder_http.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        Node(
            package='cleany_base_odometry', executable='encoder_http_node',
            name='encoder_http', parameters=[LaunchConfiguration('config')],
            output='screen',
        ),
    ])
