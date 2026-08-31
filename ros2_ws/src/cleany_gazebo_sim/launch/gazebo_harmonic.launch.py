from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from cleany_gazebo_sim.launch_helpers.sensor_profile import (
    declare_sensor_profile_argument,
    sensor_profile_bridges,
)
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    odometry_share = Path(
        get_package_share_directory('cleany_base_odometry')
    )
    description_share = Path(get_package_share_directory('cleany_description'))
    world_template = package_share / 'worlds' / 'cleany_mecanum_harmonic.sdf'
    default_world = materialize_mecanum_wheel_world(world_template)
    base_config = package_share / 'config' / 'base.yaml'
    encoder_config = package_share / 'config' / 'simulated_encoder.yaml'
    wheel_odometry_config = (
        odometry_share / 'config' / 'wheel_odometry.yaml'
    )

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
    encoder_config_arg = DeclareLaunchArgument(
        'encoder_config',
        default_value=str(encoder_config),
        description='Simulated wheel encoder parameter file.',
    )
    wheel_odometry_config_arg = DeclareLaunchArgument(
        'wheel_odometry_config',
        default_value=str(wheel_odometry_config),
        description='Wheel odometry parameter file.',
    )
    odometry_source_arg = DeclareLaunchArgument(
        'odometry_source',
        default_value='wheel',
        choices=['wheel', 'gazebo'],
        description='Source republished as /odom and odom -> base_link TF.',
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
    bridges = sensor_profile_bridges(
        package_share,
        harmonic=True,
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
            {
                'input_topic': PythonExpression(
                    [
                        "'wheel/odom' if '",
                        LaunchConfiguration('odometry_source'),
                        "' == 'wheel' else 'gazebo_odom'",
                    ]
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
    )
    wheel_odometry = Node(
        package='cleany_base_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        parameters=[
            LaunchConfiguration('wheel_odometry_config'),
            {
                'input_topic': 'wheel_encoder/joint_states',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
    )
    simulated_encoder = Node(
        package='cleany_gazebo_sim',
        executable='simulated_encoder_node',
        name='simulated_encoder',
        parameters=[
            LaunchConfiguration('encoder_config'),
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
            encoder_config_arg,
            wheel_odometry_config_arg,
            odometry_source_arg,
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
            simulated_encoder,
            wheel_odometry,
            sensor_tf,
        ]
    )
