from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory('cleany_moveit_config'))
        / 'config'
        / 'handeye_collision_objects.yaml'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'scene_config',
                default_value=default_config,
                description='Fixed hand-eye MoveIt collision-object YAML.',
            ),
            DeclareLaunchArgument(
                'service_wait_timeout_sec',
                default_value='60.0',
            ),
            Node(
                package='cleany_moveit_config',
                executable='apply_handeye_collision_scene',
                parameters=[
                    {
                        'scene_config': LaunchConfiguration('scene_config'),
                        'service_wait_timeout_sec': LaunchConfiguration(
                            'service_wait_timeout_sec'
                        ),
                    }
                ],
                output='screen',
            ),
        ]
    )
