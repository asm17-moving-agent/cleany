"""Launch construction for the Gazebo Fortress study-cafe world."""

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


def _launch_simulation(
    context: LaunchContext, *, package_share: Path
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
        package_share / 'worlds' / 'cleany_mecanum_fortress.sdf',
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
    )
    sensor_config = world.parent / 'sensor_tf.yaml'
    write_sensor_tf_config(profile, sensor_config)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'gazebo_fortress.launch.py')
        ),
        launch_arguments={
            'world': str(world),
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui_render_engine': LaunchConfiguration('gui_render_engine'),
            'bridge_config': LaunchConfiguration('bridge_config'),
            'sensor_config': str(sensor_config),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
        }.items(),
    )
    return [simulation]


def study_cafe_launch_description() -> LaunchDescription:
    """Build the Gazebo Fortress study-cafe scenario launch description."""
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
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
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value='',
        description='Optional bridge config overriding the sensor profile.',
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
    physics_step_arg = DeclareLaunchArgument(
        'physics_max_step_size', default_value='0.002'
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
            gui_render_engine_arg,
            bridge_config_arg,
            lidar_profiles_config_arg,
            lidar_profile_arg,
            lidar_noise_profiles_config_arg,
            lidar_noise_profile_arg,
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
