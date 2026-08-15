from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare('cleany_hardware_bringup')
    camera_config = PathJoinSubstitution(
        [bringup_share, 'config', 'jetson_d435.yaml']
    )
    perception_config = PathJoinSubstitution(
        [bringup_share, 'config', 'perception_d435.yaml']
    )
    perception_defaults = PathJoinSubstitution(
        [FindPackageShare('cleany_perception'), 'config', 'inspect_scene.yaml']
    )

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        emulate_tty=True,
        parameters=[
            camera_config,
            {
                'pointcloud__neon_.enable': ParameterValue(
                    LaunchConfiguration('enable_pointcloud'),
                    value_type=bool,
                ),
            },
        ],
    )
    perception = Node(
        package='cleany_perception',
        executable='inspection_node',
        name='perception_inspector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_perception')),
        parameters=[
            perception_defaults,
            perception_config,
            {
                'target_frame': LaunchConfiguration('target_frame'),
                'gemini_model': LaunchConfiguration('gemini_model'),
                'sam2_model_config': LaunchConfiguration(
                    'sam2_model_config'
                ),
                'sam2_checkpoint': LaunchConfiguration('sam2_checkpoint'),
                'sam2_device': LaunchConfiguration('sam2_device'),
                'save_debug_images': ParameterValue(
                    LaunchConfiguration('save_debug_images'),
                    value_type=bool,
                ),
                'runtime_metrics_enabled': ParameterValue(
                    LaunchConfiguration('runtime_metrics_enabled'),
                    value_type=bool,
                ),
                'diagnostics_output_root': LaunchConfiguration(
                    'diagnostics_output_root'
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_pointcloud',
                default_value='true',
                description='Publish the D435 colored PointCloud2 topic.',
            ),
            DeclareLaunchArgument(
                'start_perception',
                default_value='false',
                description='Start the Gemini/SAM2 perception node.',
            ),
            DeclareLaunchArgument(
                'target_frame',
                default_value='camera_color_optical_frame',
                description='Handheld perception output frame.',
            ),
            DeclareLaunchArgument(
                'gemini_model',
                default_value='gemini-robotics-er-2-preview',
                description='Gemini detector model ID.',
            ),
            DeclareLaunchArgument(
                'sam2_model_config',
                default_value='',
                description='SAM2 model config path.',
            ),
            DeclareLaunchArgument(
                'sam2_checkpoint',
                default_value='',
                description='SAM2 checkpoint path.',
            ),
            DeclareLaunchArgument(
                'sam2_device',
                default_value='cuda',
                description='SAM2 execution device.',
            ),
            DeclareLaunchArgument(
                'save_debug_images',
                default_value='false',
                description='Save bbox and mask images by snapshot ID.',
            ),
            DeclareLaunchArgument(
                'runtime_metrics_enabled',
                default_value='false',
                description='Log and save action timing and memory metrics.',
            ),
            DeclareLaunchArgument(
                'diagnostics_output_root',
                default_value='/tmp/cleany-perception',
                description='Root directory for snapshot diagnostics.',
            ),
            camera,
            perception,
        ]
    )
