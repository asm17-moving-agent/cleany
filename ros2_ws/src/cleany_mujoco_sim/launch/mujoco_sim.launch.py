from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scene_path_arg = DeclareLaunchArgument(
        'scene_path',
        default_value=PathJoinSubstitution(
            [FindPackageShare('cleany_mujoco_sim'), 'hardware', 'scene.xml']
        ),
        description='Path to a MuJoCo scene XML.',
    )
    publish_rate_arg = DeclareLaunchArgument('publish_rate_hz', default_value='60.0')
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    scan_rate_arg = DeclareLaunchArgument('scan_rate_hz', default_value='5.5')
    scan_samples_arg = DeclareLaunchArgument('scan_samples', default_value='0')
    camera_enabled_arg = DeclareLaunchArgument('camera_enabled', default_value='true')
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='head_realsense_rgb')
    camera_width_arg = DeclareLaunchArgument('camera_width', default_value='640')
    camera_height_arg = DeclareLaunchArgument('camera_height', default_value='480')
    camera_rate_arg = DeclareLaunchArgument('camera_rate_hz', default_value='15.0')
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='image_raw')
    camera_info_topic_arg = DeclareLaunchArgument('camera_info_topic', default_value='camera_info')
    depth_enabled_arg = DeclareLaunchArgument('depth_enabled', default_value='true')
    depth_topic_arg = DeclareLaunchArgument('depth_topic', default_value='depth')

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
                'camera_enabled': LaunchConfiguration('camera_enabled'),
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'camera_rate_hz': LaunchConfiguration('camera_rate_hz'),
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_enabled': LaunchConfiguration('depth_enabled'),
                'depth_topic': LaunchConfiguration('depth_topic'),
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
        camera_enabled_arg,
        camera_name_arg,
        camera_width_arg,
        camera_height_arg,
        camera_rate_arg,
        image_topic_arg,
        camera_info_topic_arg,
        depth_enabled_arg,
        depth_topic_arg,
        node,
    ])
