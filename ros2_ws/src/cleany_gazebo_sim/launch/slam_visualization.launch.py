from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
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
            str(package_share / 'config' / 'slam_visualization.rviz'),
        ],
        output='screen',
    )
    return LaunchDescription([marker, rviz])
