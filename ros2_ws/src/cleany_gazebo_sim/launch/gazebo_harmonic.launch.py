from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from cleany_gazebo_sim.launch_helpers.robot_nodes import robot_runtime_nodes
from cleany_gazebo_sim.launch_helpers.sensor_profile import (
    declare_sensor_profile_argument,
    sensor_profile_bridges,
)
from cleany_gazebo_sim.world.sdf import materialize_friction_frames
from cleany_gazebo_sim.world.generator import materialize_mecanum_wheel_world


def _normalize_world(context):
    source = Path(LaunchConfiguration('world').perform(context))
    step = LaunchConfiguration('physics_max_step_size').perform(context).strip()
    factor = LaunchConfiguration('physics_real_time_factor').perform(context).strip()
    return [SetLaunchConfiguration('world', str(materialize_friction_frames(
        source, harmonic=True, max_step_size=float(step) if step else None,
        real_time_factor=float(factor) if factor else None,
    )))]


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
    odometry_error_config = (
        package_share / 'config' / 'odometry_error_ideal.yaml'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value=str(default_world)
    )
    physics_step_arg = DeclareLaunchArgument(
        'physics_max_step_size', default_value='',
        description='Optional physics step in seconds (0, 0.01]; empty preserves the input SDF.',
    )
    physics_factor_arg = DeclareLaunchArgument(
        'physics_real_time_factor', default_value='',
        description='Requested simulation/wall speed; empty preserves SDF. Measure /clock to verify achieved speed.',
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
    odometry_error_config_arg = DeclareLaunchArgument(
        'odometry_error_config',
        default_value=str(odometry_error_config),
        description='Simulation-only wheel odometry error parameter file.',
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
    gui_render_engine_arg = DeclareLaunchArgument(
        'gui_render_engine', default_value='ogre2', choices=['ogre', 'ogre2'],
        description='Rendering engine used by the Gazebo GUI.',
    )

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
            LaunchConfiguration('gui_render_engine'),
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
    return LaunchDescription(
        [
            world_arg,
            physics_step_arg,
            physics_factor_arg,
            bridge_config_arg,
            sensor_config_arg,
            encoder_config_arg,
            wheel_odometry_config_arg,
            odometry_error_config_arg,
            odometry_source_arg,
            headless_arg,
            use_sim_time_arg,
            sensor_profile_arg,
            gui_render_engine_arg,
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', str(description_share)
            ),
            OpaqueFunction(function=_normalize_world),
            server,
            gui,
            bridges,
            *robot_runtime_nodes(base_config),
        ]
    )
