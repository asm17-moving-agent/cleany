from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _inspection_node(context):
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
            LaunchConfiguration('yoloe_config_file'),
            {
                **{name: ParameterValue(LaunchConfiguration(name), value_type=bool)
                   for name in ('use_sim_time', 'enable_reference_observation', 'enable_wrist_observation', 'preload_models', 'vlm_ik_warmup_enabled')},
                'reference_ttl_seconds': ParameterValue(LaunchConfiguration('reference_ttl_seconds'), value_type=float),
                **{name: LaunchConfiguration(name) for name in (
                    'yoloe_model_path', 'yoloe_text_encoder_directory',
                    'detector_type', 'segmenter_type', 'gemini_model', 'yoloe_device',
                ) if LaunchConfiguration(name).perform(context).strip()},
                **({'yoloe_classes': ParameterValue(
                    LaunchConfiguration('yoloe_classes'), value_type=list[str])}
                   if LaunchConfiguration('yoloe_classes').perform(context).strip() else {}),
                **({'minimum_detection_confidence': ParameterValue(
                    LaunchConfiguration('minimum_detection_confidence'),
                    value_type=float,
                )} if LaunchConfiguration('minimum_detection_confidence').perform(context).strip() else {}),
                'target_frame': LaunchConfiguration('target_frame'),
                'rgbd_qos_reliability': LaunchConfiguration('rgbd_qos_reliability'),
            },
        ],
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            *(DeclareLaunchArgument(name, default_value=value) for name, value in {
                'use_sim_time': 'false', 'enable_reference_observation': 'false',
                'enable_wrist_observation': 'false', 'preload_models': 'true',
                'vlm_ik_warmup_enabled': 'false',
                'reference_ttl_seconds': '600.0'}.items()),
            DeclareLaunchArgument('detector_type', default_value=''),
            DeclareLaunchArgument('segmenter_type', default_value=''),
            DeclareLaunchArgument(
                'gemini_model',
                default_value='',
            ),
            DeclareLaunchArgument('yoloe_config_file', default_value=PathJoinSubstitution([
                FindPackageShare('cleany_perception'), 'config', 'yoloe_real34.yaml'])),
            DeclareLaunchArgument('yoloe_model_path', default_value=''),
            DeclareLaunchArgument(
                'yoloe_classes',
                default_value='',
            ),
            DeclareLaunchArgument('yoloe_device', default_value=''),
            DeclareLaunchArgument(
                'yoloe_text_encoder_directory', default_value=''
            ),
            DeclareLaunchArgument(
                'minimum_detection_confidence', default_value=''
            ),
            DeclareLaunchArgument('target_frame', default_value='base_link'),
            DeclareLaunchArgument('rgbd_qos_reliability', default_value='best_effort',
                                  choices=['best_effort', 'reliable']),
            OpaqueFunction(function=_inspection_node),
        ]
    )
