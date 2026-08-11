from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from cleany_gazebo_sim.world_generator import materialize_study_cafe_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    world = materialize_study_cafe_world(
        package_share / 'worlds' / 'cleany_mecanum_harmonic.sdf',
        simulator='harmonic',
    )
    base_config = package_share / 'config' / 'base.yaml'
    bridge_config = (
        package_share / 'config' / 'navigation_bridge_harmonic.yaml'
    )

    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config', default_value=str(bridge_config)
    )
    sensor_config_arg = DeclareLaunchArgument(
        'sensor_config', default_value=str(base_config)
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
        }.items(),
    )

    return LaunchDescription(
        [
            headless_arg,
            use_sim_time_arg,
            bridge_config_arg,
            sensor_config_arg,
            SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
            SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
            SetEnvironmentVariable('QT_SCALE_FACTOR', '1.0'),
            simulation,
        ]
    )
