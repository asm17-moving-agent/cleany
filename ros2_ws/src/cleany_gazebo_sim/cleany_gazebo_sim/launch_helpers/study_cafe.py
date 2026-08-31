"""Launch construction for Fortress and Harmonic study-cafe worlds."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from cleany_gazebo_sim.gazebo_slam_experiment import (
    load_mount_profiles,
    write_sensor_tf_config,
)
from cleany_gazebo_sim.lidar_noise import load_lidar_noise_profile
from cleany_gazebo_sim.launch_helpers.sensor_profile import (
    declare_sensor_profile_argument,
)
from cleany_gazebo_sim.world.generator import materialize_study_cafe_world


def _optional_spawn_pose(value: str) -> tuple[float, ...] | None:
    if not value.strip():
        return None
    try:
        pose = tuple(float(item) for item in value.split(','))
    except ValueError as error:
        raise ValueError(
            'robot_spawn_pose must be six comma-separated numbers'
        ) from error
    if len(pose) != 6:
        raise ValueError(
            'robot_spawn_pose must be six comma-separated numbers'
        )
    return pose


_BACKEND_LAUNCH = {
    'fortress': 'gazebo_fortress.launch.py',
    'harmonic': 'gazebo_harmonic.launch.py',
}


def _launch_simulation(
    context: LaunchContext, *, package_share: Path, simulator: str
) -> list[IncludeLaunchDescription]:
    profiles_path = Path(
        LaunchConfiguration('lidar_profiles_config').perform(context)
    )
    profile_name = LaunchConfiguration('lidar_profile').perform(context)
    noise_profile_name = LaunchConfiguration('lidar_noise_profile').perform(
        context
    )
    noise_profiles_path = Path(
        LaunchConfiguration('lidar_noise_profiles_config').perform(context)
    )
    try:
        profile = load_mount_profiles(profiles_path)[profile_name]
    except KeyError as error:
        raise ValueError(
            f'unknown lidar_profile {profile_name!r}; check {profiles_path}'
        ) from error
    if profile.transform.rotation_xyzw != (0.0, 0.0, 0.0, 1.0):
        raise ValueError('study-cafe LiDAR profiles must be level mounts')

    world = materialize_study_cafe_world(
        package_share / 'worlds' / f'cleany_mecanum_{simulator}.sdf',
        max_step_size=float(
            LaunchConfiguration('physics_max_step_size').perform(context)
        ),
        real_time_factor=float(
            LaunchConfiguration('physics_real_time_factor').perform(context)
        ),
        layout_path=Path(
            LaunchConfiguration('layout_config').perform(context)
        ),
        lidar_translation=profile.transform.translation,
        lidar_noise=load_lidar_noise_profile(
            noise_profiles_path,
            noise_profile_name,
        ),
        sensor_render_engine=LaunchConfiguration(
            'sensor_render_engine'
        ).perform(context),
        robot_spawn_pose=_optional_spawn_pose(
            LaunchConfiguration('robot_spawn_pose').perform(context)
        ),
    )
    sensor_config = world.parent / 'sensor_tf.yaml'
    write_sensor_tf_config(profile, sensor_config)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / _BACKEND_LAUNCH[simulator])
        ),
        launch_arguments={
            'world': str(world),
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bridge_config': LaunchConfiguration('bridge_config'),
            'sensor_config': str(sensor_config),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
            'encoder_config': LaunchConfiguration('encoder_config'),
            'wheel_odometry_config': LaunchConfiguration(
                'wheel_odometry_config'
            ),
            'odometry_source': LaunchConfiguration('odometry_source'),
            **(
                {
                    'gui_render_engine': LaunchConfiguration('gui_render_engine'),
                    'server_render_engine': LaunchConfiguration(
                        'server_render_engine'
                    ),
                }
                if simulator == 'fortress'
                else {}
            ),
        }.items(),
    )
    return [simulation]


def study_cafe_launch_description(
    simulator: str = 'fortress',
) -> LaunchDescription:
    """Build a profile-specific study-cafe scenario launch description."""
    if simulator not in _BACKEND_LAUNCH:
        raise ValueError(f'unsupported study-cafe simulator: {simulator!r}')
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    odometry_share = Path(
        get_package_share_directory('cleany_base_odometry')
    )
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
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
    sensor_render_engine_arg = DeclareLaunchArgument(
        'sensor_render_engine',
        default_value='ogre2',
        choices=['ogre', 'ogre2'],
        description='Rendering engine used by Gazebo rendering sensors.',
    )
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value='',
        description='Optional bridge config overriding the sensor profile.',
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
    lidar_profiles_config_arg = DeclareLaunchArgument(
        'lidar_profiles_config',
        default_value=str(
            package_share / 'config' / 'lidar_mount_profiles.yaml'
        ),
    )
    lidar_profile_arg = DeclareLaunchArgument(
        'lidar_profile', default_value='floor_26cm'
    )
    lidar_noise_profiles_config_arg = DeclareLaunchArgument(
        'lidar_noise_profiles_config',
        default_value=str(
            package_share / 'config' / 'lidar_noise_profiles.yaml'
        ),
    )
    lidar_noise_profile_arg = DeclareLaunchArgument(
        'lidar_noise_profile', default_value='measured'
    )
    layout_config_arg = DeclareLaunchArgument(
        'layout_config',
        default_value=str(
            package_share
            / 'config'
            / 'study_cafe'
            / 'study_cafe_layout.yaml'
        ),
        description='Study-cafe room and repeated furniture layout.',
    )
    robot_spawn_pose_arg = DeclareLaunchArgument(
        'robot_spawn_pose',
        default_value='',
        description=(
            'Optional x,y,z,roll,pitch,yaw pose overriding the layout spawn.'
        ),
    )
    physics_step_arg = DeclareLaunchArgument(
        'physics_max_step_size', default_value='0.002'
    )
    real_time_factor_arg = DeclareLaunchArgument(
        'physics_real_time_factor', default_value='1.0'
    )
    sensor_profile_arg = declare_sensor_profile_argument()
    simulation = OpaqueFunction(
        function=_launch_simulation,
        kwargs={'package_share': package_share, 'simulator': simulator},
    )

    return LaunchDescription(
        [
            headless_arg,
            use_sim_time_arg,
            gui_render_engine_arg,
            server_render_engine_arg,
            sensor_render_engine_arg,
            bridge_config_arg,
            encoder_config_arg,
            wheel_odometry_config_arg,
            odometry_source_arg,
            lidar_profiles_config_arg,
            lidar_profile_arg,
            lidar_noise_profiles_config_arg,
            lidar_noise_profile_arg,
            layout_config_arg,
            robot_spawn_pose_arg,
            physics_step_arg,
            real_time_factor_arg,
            sensor_profile_arg,
            SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
            SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
            SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0'),
            simulation,
        ]
    )
