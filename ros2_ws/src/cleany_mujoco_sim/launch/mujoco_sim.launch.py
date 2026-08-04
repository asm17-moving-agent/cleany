from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scene_path_arg = DeclareLaunchArgument(
        'scene_path',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('cleany_mujoco_sim'),
                'scenes',
                'default.xml.in',
            ]
        ),
        description='Path to a MuJoCo scene XML or XML template.',
    )
    publish_rate_arg = DeclareLaunchArgument('publish_rate_hz', default_value='60.0')
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    scan_rate_arg = DeclareLaunchArgument('scan_rate_hz', default_value='5.5')
    scan_samples_arg = DeclareLaunchArgument('scan_samples', default_value='0')

    node = Node(
        package='cleany_mujoco_sim',
        executable='mujoco_sim_node',
        name='mujoco_sim',
        parameters=[
            {
                'scene_path': LaunchConfiguration('scene_path'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'headless': LaunchConfiguration('headless'),
                'scan_rate_hz': LaunchConfiguration('scan_rate_hz'),
                'scan_samples': LaunchConfiguration('scan_samples'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        scene_path_arg,
        publish_rate_arg,
        headless_arg,
        scan_rate_arg,
        scan_samples_arg,
        node,
    ])
