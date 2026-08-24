from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_skill_executor'))
    return LaunchDescription(
        [
            Node(
                package='cleany_skill_executor',
                executable='nearest_pregrasp_coordinator',
                parameters=[str(share / 'config' / 'nearest_pregrasp.yaml')],
                output='screen',
            )
        ]
    )
