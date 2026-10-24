from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from cleany_gazebo_sim.launch_helpers.robot_nodes import robot_runtime_nodes
from cleany_gazebo_sim.launch_helpers.sensor_profile import (
    declare_sensor_profile_argument,
    sensor_profile_bridges,
)
from cleany_gazebo_sim.lidar_noise import load_lidar_noise_profile
from cleany_gazebo_sim.world.sdf import materialize_friction_frames
from cleany_gazebo_sim.world.static_scene import materialize_static_scene
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
    base_config = Path('/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/dwb/20261014T0035_5bfca0/campaign/4_distant/config/backend_base.yaml')
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
    odometry_error_config_arg = DeclareLaunchArgument(
        'odometry_error_config',
        default_value=str(
            package_share / 'config' / 'odometry_error_ideal.yaml'
        ),
        description='Simulation-only wheel odometry error parameter file.',
    )
    odometry_source_arg = DeclareLaunchArgument(
        'odometry_source',
        default_value='wheel',
        choices=['wheel', 'gazebo'],
        description='Source republished as /odom and odom -> base_link TF.',
    )
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    headless_rendering_arg = DeclareLaunchArgument(
        'headless_rendering', default_value='false', choices=['true', 'false'],
        description='Use EGL rendering for the headless server without X11.',
    )
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
    server_gallium_thread_arg = DeclareLaunchArgument(
        'server_gallium_thread',
        default_value=EnvironmentVariable('GALLIUM_THREAD', default_value='0'),
        choices=['0', '1'],
        description='Mesa threaded context for sensor rendering; 0 avoids Freedreno readback stalls.',
    )
    sensor_profile_arg = declare_sensor_profile_argument()

    return LaunchDescription(
        [
            world_arg,
            DeclareLaunchArgument(
                'compact_static_scene', default_value='false', choices=['true', 'false'],
                description='Group simple static geometry in a temporary SDF; model identities change.',
            ),
            physics_step_arg,
            physics_factor_arg,
            lidar_noise_profile_arg,
            bridge_config_arg,
            sensor_config_arg,
            encoder_config_arg,
            wheel_odometry_config_arg,
            odometry_error_config_arg,
            odometry_source_arg,
            headless_arg,
            headless_rendering_arg,
            use_sim_time_arg,
            gui_render_engine_arg,
            server_render_engine_arg,
            server_gallium_thread_arg,
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

    if LaunchConfiguration('compact_static_scene').perform(context) == 'true':
        world = str(materialize_static_scene(Path(world)))

    step = LaunchConfiguration('physics_max_step_size').perform(context).strip()
    factor = LaunchConfiguration('physics_real_time_factor').perform(context).strip()
    world = str(materialize_friction_frames(
        Path(world), max_step_size=float(step) if step else None,
        real_time_factor=float(factor) if factor else None,
    ))
    server_command = [
        'ign', 'gazebo', '-r', '-s', '--render-engine-server',
        LaunchConfiguration('server_render_engine'),
    ]
    if LaunchConfiguration('headless_rendering').perform(context) == 'true':
        server_command.append('--headless-rendering')
    server_command.append(world)
    server = ExecuteProcess(
        cmd=server_command,
        additional_env={'GALLIUM_THREAD': LaunchConfiguration('server_gallium_thread')},
        output='screen',
    )
    gui = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-g',
            '--render-engine-gui',
            LaunchConfiguration('gui_render_engine'),
        ],
        condition=UnlessCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    bridges = sensor_profile_bridges(
        package_share,
        harmonic=False,
        bridge_config=LaunchConfiguration('bridge_config'),
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
        *robot_runtime_nodes(base_config),
    ]
