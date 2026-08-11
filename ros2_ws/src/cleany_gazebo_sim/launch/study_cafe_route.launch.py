from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    route_config = package_share / 'config' / 'study_cafe_route.yaml'
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    follower = Node(
        package='cleany_gazebo_sim',
        executable='ground_truth_route_follower',
        name='ground_truth_route_follower',
        parameters=[
            route_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    return LaunchDescription([use_sim_time_arg, follower])
