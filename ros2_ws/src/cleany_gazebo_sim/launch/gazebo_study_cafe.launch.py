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

from cleany_gazebo_sim.world_generator import materialize_study_cafe_world
from cleany_gazebo_sim.sensor_profile_launch import (
    declare_sensor_profile_argument,
)


def _launch_simulation(
    context: LaunchContext, *, package_share: Path
) -> list[IncludeLaunchDescription]:
    world = materialize_study_cafe_world(
        package_share / 'worlds' / 'cleany_mecanum_harmonic.sdf',
        simulator='harmonic',
        max_step_size=float(
            LaunchConfiguration('physics_max_step_size').perform(context)
        ),
        real_time_factor=float(
            LaunchConfiguration('physics_real_time_factor').perform(context)
        ),
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'gazebo_harmonic.launch.py')
        ),
        launch_arguments={
            'world': str(world),
            'headless': LaunchConfiguration('headless'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'bridge_config': LaunchConfiguration('bridge_config'),
            'sensor_config': LaunchConfiguration('sensor_config'),
            'sensor_profile': LaunchConfiguration('sensor_profile'),
        }.items(),
    )
    return [simulation]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    base_config = package_share / 'config' / 'base.yaml'
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value='',
        description='Optional bridge config overriding the sensor profile.',
    )
    sensor_config_arg = DeclareLaunchArgument(
        'sensor_config', default_value=str(base_config)
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
            sensor_config_arg,
            physics_step_arg,
            real_time_factor_arg,
            sensor_profile_arg,
            SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
            SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
            SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0'),
            simulation,
        ]
    )
