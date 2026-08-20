from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from cleany_gazebo_sim.world.generator import materialize_study_cafe_world
from cleany_gazebo_sim.gazebo_slam_experiment import (
    load_mount_profiles,
    write_sensor_tf_config,
)
from cleany_gazebo_sim.sensor_profile_launch import (
    declare_sensor_profile_argument,
)


def _launch_simulation(
    context: LaunchContext, *, package_share: Path
) -> list[IncludeLaunchDescription]:
    profiles_path = Path(
        LaunchConfiguration('lidar_profiles_config').perform(context)
    )
    profile_name = LaunchConfiguration('lidar_profile').perform(context)
    try:
        profile = load_mount_profiles(profiles_path)[profile_name]
    except KeyError as error:
        raise ValueError(
            f'unknown lidar_profile {profile_name!r}; check {profiles_path}'
        ) from error
    if profile.transform.rotation_xyzw != (0.0, 0.0, 0.0, 1.0):
        raise ValueError('study-cafe LiDAR profiles must be level mounts')
    world = materialize_study_cafe_world(
        package_share / 'worlds' / 'cleany_mecanum_harmonic.sdf',
        simulator='harmonic',
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
    )
    sensor_config = Path('/tmp') / (
        f'cleany_study_cafe_sensor_tf_{profile.name}.yaml'
    )
    write_sensor_tf_config(profile, sensor_config)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'gazebo_harmonic.launch.py')
        ),
        launch_arguments={
            'world': str(world),
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bridge_config': LaunchConfiguration('bridge_config'),
            'sensor_config': str(sensor_config),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
        }.items(),
    )
    return [simulation]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value='',
        description='Optional bridge config overriding the sensor profile.',
    )
    lidar_profiles_config_arg = DeclareLaunchArgument(
        'lidar_profiles_config',
        default_value=str(package_share / 'config' / 'lidar_mount_profiles.yaml'),
    )
    lidar_profile_arg = DeclareLaunchArgument(
        'lidar_profile', default_value='floor_26cm'
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
    physics_step_arg = DeclareLaunchArgument(
        'physics_max_step_size', default_value='0.001'
    )
    real_time_factor_arg = DeclareLaunchArgument(
        'physics_real_time_factor', default_value='1.0'
    )
    sensor_profile_arg = declare_sensor_profile_argument()
    simulation = OpaqueFunction(
        function=_launch_simulation,
        kwargs={'package_share': package_share},
    )

    return LaunchDescription(
        [
            headless_arg,
            use_sim_time_arg,
            bridge_config_arg,
            lidar_profiles_config_arg,
            lidar_profile_arg,
            layout_config_arg,
            physics_step_arg,
            real_time_factor_arg,
            sensor_profile_arg,
            SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
            SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
            SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0'),
            simulation,
        ]
    )
