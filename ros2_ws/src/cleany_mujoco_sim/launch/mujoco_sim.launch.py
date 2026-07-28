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
    max_linear_x_arg = DeclareLaunchArgument(
        'max_linear_x',
        default_value='0.3',
        description='Maximum absolute forward or reverse velocity in m/s.',
    )
    max_linear_y_arg = DeclareLaunchArgument(
        'max_linear_y',
        default_value='0.3',
        description='Maximum absolute lateral velocity in m/s.',
    )
    max_angular_z_arg = DeclareLaunchArgument(
        'max_angular_z',
        default_value='0.8',
        description='Maximum absolute yaw velocity in rad/s.',
    )
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout_sec',
        default_value='0.5',
        description='Stop target timeout after the last valid cmd_vel in seconds.',
    )
    timeout_check_rate_arg = DeclareLaunchArgument(
        'timeout_check_rate_hz',
        default_value='20.0',
        description='Rate for checking cmd_vel timeout in Hz.',
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0635',
        description='Effective mecanum wheel radius in meters.',
    )
    wheelbase_length_arg = DeclareLaunchArgument(
        'wheelbase_length',
        default_value='0.30',
        description='Distance between front and rear wheel centers in meters.',
    )
    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='0.51',
        description='Distance between left and right wheel centers in meters.',
    )
    max_wheel_speed_arg = DeclareLaunchArgument(
        'max_wheel_speed',
        default_value='10.815',
        description='Maximum absolute target wheel speed in rad/s.',
    )

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
                'max_linear_x': LaunchConfiguration('max_linear_x'),
                'max_linear_y': LaunchConfiguration('max_linear_y'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
                'cmd_vel_timeout_sec': LaunchConfiguration('cmd_vel_timeout_sec'),
                'timeout_check_rate_hz': LaunchConfiguration(
                    'timeout_check_rate_hz'
                ),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheelbase_length': LaunchConfiguration('wheelbase_length'),
                'track_width': LaunchConfiguration('track_width'),
                'max_wheel_speed': LaunchConfiguration('max_wheel_speed'),
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
        max_linear_x_arg,
        max_linear_y_arg,
        max_angular_z_arg,
        cmd_vel_timeout_arg,
        timeout_check_rate_arg,
        wheel_radius_arg,
        wheelbase_length_arg,
        track_width_arg,
        max_wheel_speed_arg,
        node,
    ])
