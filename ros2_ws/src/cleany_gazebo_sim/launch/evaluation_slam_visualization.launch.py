from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    navigation_share = Path(
        get_package_share_directory('cleany_navigation')
    )
    marker = Node(
        package='cleany_gazebo_sim',
        executable='occupancy_grid_marker',
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            str(
                navigation_share / 'rviz' / 'slam_visualization.rviz'
            ),
        ],
        output='screen',
    )
    return LaunchDescription([marker, rviz])
