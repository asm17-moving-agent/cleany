from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_base_odometry'))
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.4.1'),
        DeclareLaunchArgument('start_receiver', default_value='true'),
        DeclareLaunchArgument('receiver_config', default_value=str(share / 'config/encoder_http.yaml')),
        DeclareLaunchArgument('odom_config', default_value=str(share / 'config/hardware_wheel_odometry.yaml')),
        Node(
            package='cleany_base_odometry', executable='encoder_http_node',
            name='encoder_http', output='screen',
            parameters=[LaunchConfiguration('receiver_config'), {'host': LaunchConfiguration('host')}],
            condition=IfCondition(LaunchConfiguration('start_receiver')),
        ),
        Node(
            package='cleany_base_odometry', executable='hardware_odom_node',
            name='hardware_wheel_odometry', output='screen',
            parameters=[LaunchConfiguration('odom_config')],
        ),
    ])
