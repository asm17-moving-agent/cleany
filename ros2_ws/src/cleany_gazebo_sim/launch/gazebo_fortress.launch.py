from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_gazebo_sim.sensor_profile_launch import (
    declare_sensor_profile_argument,
    sensor_profile_bridges,
)
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    description_share = Path(
        get_package_share_directory('cleany_description')
    )
    world_template = package_share / 'worlds' / 'cleany_mecanum_fortress.sdf'
    default_world = materialize_mecanum_wheel_world(world_template)
    base_config = package_share / 'config' / 'base.yaml'

    world_arg = DeclareLaunchArgument(
        'world', default_value=str(default_world)
    )
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value='',
        description='Optional bridge config overriding the sensor profile.',
    )
    sensor_config_arg = DeclareLaunchArgument(
        'sensor_config', default_value=str(base_config)
    )
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    sensor_profile_arg = declare_sensor_profile_argument()

    server = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-s',
            '--render-engine-server',
            'ogre2',
            LaunchConfiguration('world'),
        ],
        condition=IfCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    gui = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '--render-engine-server',
            'ogre2',
            '--render-engine-gui',
            'ogre',
            LaunchConfiguration('world'),
        ],
        condition=UnlessCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    bridges = sensor_profile_bridges(
        package_share,
        harmonic=False,
        bridge_config=LaunchConfiguration('bridge_config'),
    )
    command_guard = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_command_guard',
        name='gazebo_command_guard',
        parameters=[
            base_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    odom_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_odom_tf_publisher',
        name='gazebo_odom_tf_publisher',
        parameters=[
            base_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    sensor_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_sensor_tf_publisher',
        name='gazebo_sensor_tf_publisher',
        parameters=[
            LaunchConfiguration('sensor_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            world_arg,
            bridge_config_arg,
            sensor_config_arg,
            headless_arg,
            use_sim_time_arg,
            sensor_profile_arg,
            # Reuse the authoritative description meshes instead of
            # committing duplicate, large STL assets to this package.
            AppendEnvironmentVariable(
                'IGN_GAZEBO_RESOURCE_PATH', str(description_share)
            ),
            server,
            gui,
            bridges,
            command_guard,
            odom_tf,
            sensor_tf,
        ]
    )
