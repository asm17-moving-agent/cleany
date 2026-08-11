from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    default_params = package_share / 'config' / 'slam_toolbox.yaml'

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock.',
    )
    params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=str(default_params),
        description='slam_toolbox parameter file.',
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription([use_sim_time_arg, params_file_arg, slam])
