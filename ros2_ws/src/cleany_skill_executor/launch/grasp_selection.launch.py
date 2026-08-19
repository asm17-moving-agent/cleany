from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory('cleany_skill_executor')) / 'config' / 'grasp_selection.yaml'
    return LaunchDescription([
        Node(
            package='cleany_skill_executor',
            executable='grasp_selection_server',
            parameters=[str(config)],
            output='screen',
        )
    ])
