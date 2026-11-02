from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
                'detector_type': LaunchConfiguration('detector_type'),
                'segmenter_type': LaunchConfiguration('segmenter_type'),
                'gemini_model': LaunchConfiguration('gemini_model'),
                'yoloe_model_path': LaunchConfiguration(
                    'yoloe_model_path'
                ),
                'yoloe_classes': ParameterValue(
                    LaunchConfiguration('yoloe_classes'),
                    value_type=list[str],
                ),
                'yoloe_device': LaunchConfiguration('yoloe_device'),
                'yoloe_text_encoder_directory': LaunchConfiguration(
                    'yoloe_text_encoder_directory'
                ),
                'minimum_detection_confidence': ParameterValue(
                    LaunchConfiguration('minimum_detection_confidence'),
                    value_type=float,
                ),
                'target_frame': LaunchConfiguration('target_frame'),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('detector_type', default_value='yoloe_gemini'),
            DeclareLaunchArgument('segmenter_type', default_value='yoloe_seg'),
            DeclareLaunchArgument(
                'gemini_model',
                default_value='gemini-robotics-er-2-preview',
            ),
            DeclareLaunchArgument('yoloe_model_path', default_value=''),
            DeclareLaunchArgument(
                'yoloe_classes',
                default_value="['cup', 'wallet', 'crumpled tissue', 'lego brick']",
            ),
            DeclareLaunchArgument('yoloe_device', default_value='cuda'),
            DeclareLaunchArgument(
                'yoloe_text_encoder_directory', default_value=''
            ),
            DeclareLaunchArgument(
                'minimum_detection_confidence', default_value='0.25'
            ),
            DeclareLaunchArgument('target_frame', default_value='base_link'),
            node,
        ]
    )
