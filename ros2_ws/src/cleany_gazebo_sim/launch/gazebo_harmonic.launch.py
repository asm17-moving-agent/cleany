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
from cleany_gazebo_sim.world_generator import materialize_mecanum_wheel_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    description_share = Path(get_package_share_directory('cleany_description'))
    world_template = package_share / 'worlds' / 'cleany_mecanum_harmonic.sdf'
    default_world = materialize_mecanum_wheel_world(world_template)
    base_config = package_share / 'config' / 'base.yaml'

    world_arg = DeclareLaunchArgument(
        'world', default_value=str(default_world)
    )
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    sensor_profile_arg = declare_sensor_profile_argument()

    server = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            '-s',
            '--headless-rendering',
            '--render-engine-server',
            'ogre2',
            LaunchConfiguration('world'),
        ],
        condition=IfCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    gui = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
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
    bridges = sensor_profile_bridges(package_share, harmonic=True)
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
            base_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            use_sim_time_arg,
            sensor_profile_arg,
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', str(description_share)
            ),
            server,
            gui,
            bridges,
            command_guard,
            odom_tf,
            sensor_tf,
        ]
    )
