from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node

from cleany_gazebo_sim.launch_helpers.sensor_profile import (
    declare_sensor_profile_argument,
    sensor_profile_bridges,
)
from cleany_gazebo_sim.lidar_noise import load_lidar_noise_profile
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    odometry_share = Path(
        get_package_share_directory('cleany_base_odometry')
    )
    description_share = Path(
        get_package_share_directory('cleany_description')
    )
    world_template = package_share / 'worlds' / 'cleany_mecanum_fortress.sdf'
    base_config = package_share / 'config' / 'base.yaml'
    noise_profiles = package_share / 'config' / 'lidar_noise_profiles.yaml'

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Optional SDF world overriding the generated LiDAR profile world.',
    )
    lidar_noise_profile_arg = DeclareLaunchArgument(
        'lidar_noise_profile',
        default_value='measured',
        description='LiDAR Gaussian noise profile: measured or stress.',
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
        default_value=str(
            package_share / 'config' / 'simulated_encoder.yaml'
        ),
        description='Simulated wheel encoder parameter file.',
    )
    wheel_odometry_config_arg = DeclareLaunchArgument(
        'wheel_odometry_config',
        default_value=str(
            odometry_share / 'config' / 'wheel_odometry.yaml'
        ),
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
    gui_render_engine_arg = DeclareLaunchArgument(
        'gui_render_engine',
        default_value=EnvironmentVariable(
            'GAZEBO_GUI_RENDER_ENGINE', default_value='ogre'
        ),
        choices=['ogre', 'ogre2'],
        description='Rendering engine used by the Gazebo GUI.',
    )
    server_render_engine_arg = DeclareLaunchArgument(
        'server_render_engine',
        default_value='ogre2',
        choices=['ogre', 'ogre2'],
        description='Rendering engine used by the Gazebo server.',
    )
    sensor_profile_arg = declare_sensor_profile_argument()

    return LaunchDescription(
        [
            world_arg,
            lidar_noise_profile_arg,
            bridge_config_arg,
            sensor_config_arg,
            encoder_config_arg,
            wheel_odometry_config_arg,
            odometry_source_arg,
            headless_arg,
            use_sim_time_arg,
            gui_render_engine_arg,
            server_render_engine_arg,
            sensor_profile_arg,
            OpaqueFunction(
                function=_launch_setup,
                kwargs={
                    'package_share': package_share,
                    'description_share': description_share,
                    'world_template': world_template,
                    'noise_profiles': noise_profiles,
                    'base_config': base_config,
                },
            ),
        ]
    )


def _launch_setup(
    context,
    *,
    package_share: Path,
    description_share: Path,
    world_template: Path,
    noise_profiles: Path,
    base_config: Path,
):
    world_override = LaunchConfiguration('world').perform(context)
    if world_override:
        world = world_override
    else:
        profile_name = LaunchConfiguration('lidar_noise_profile').perform(context)
        profile = load_lidar_noise_profile(noise_profiles, profile_name)
        world = str(materialize_mecanum_wheel_world(world_template, profile))

    server = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            '-s',
            '--render-engine-server',
            LaunchConfiguration('server_render_engine'),
            world,
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
            LaunchConfiguration('server_render_engine'),
            '--render-engine-gui',
            LaunchConfiguration('gui_render_engine'),
            world,
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

    return [
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
        simulated_encoder,
        wheel_odometry,
        sensor_tf,
    ]
