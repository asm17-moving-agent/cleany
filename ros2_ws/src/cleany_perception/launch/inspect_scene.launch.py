from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_path = PathJoinSubstitution(
        [FindPackageShare('cleany_perception'), 'config', 'inspect_scene.yaml']
    )
    node = Node(
        package='cleany_perception',
        executable='inspection_node',
        name='perception_inspector',
        output='screen',
        parameters=[
            config_path,
            {
                'gemini_model': LaunchConfiguration('gemini_model'),
                'sam2_model_config': LaunchConfiguration('sam2_model_config'),
                'sam2_checkpoint': LaunchConfiguration('sam2_checkpoint'),
                'sam2_device': LaunchConfiguration('sam2_device'),
                'target_frame': LaunchConfiguration('target_frame'),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gemini_model',
                default_value='gemini-robotics-er-2-preview',
            ),
            DeclareLaunchArgument('sam2_model_config', default_value=''),
            DeclareLaunchArgument('sam2_checkpoint', default_value=''),
            DeclareLaunchArgument('sam2_device', default_value='cuda'),
            DeclareLaunchArgument('target_frame', default_value='base_link'),
            node,
        ]
    )
