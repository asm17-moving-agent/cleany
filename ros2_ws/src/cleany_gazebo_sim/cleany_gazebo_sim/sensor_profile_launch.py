from __future__ import annotations

from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SENSOR_PROFILES = (
    'lidar_nav',
    'head_rgbd',
    'left_wrist',
    'right_wrist',
    'all_cameras',
)

_PROFILE_BRIDGES = {
    'lidar_nav': ('lidar',),
    'head_rgbd': ('head_rgbd',),
    'left_wrist': ('left_wrist',),
    'right_wrist': ('right_wrist',),
    'all_cameras': ('head_rgbd', 'left_wrist', 'right_wrist'),
}


def declare_sensor_profile_argument() -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        'sensor_profile',
        default_value='lidar_nav',
        choices=list(SENSOR_PROFILES),
        description='Rendering sensor and bridge workload to enable.',
    )


def sensor_profile_bridge_groups(profile: str) -> tuple[str, ...]:
    try:
        return ('core', *_PROFILE_BRIDGES[profile])
    except KeyError as error:
        choices = ', '.join(SENSOR_PROFILES)
        raise ValueError(
            f'unknown sensor profile {profile!r}; choose one of: {choices}'
        ) from error


def _sensor_bridge_nodes(
    context: LaunchContext,
    *,
    package_share: Path,
    harmonic: bool,
) -> list[Node]:
    profile = LaunchConfiguration('sensor_profile').perform(context)
    config_suffix = '_harmonic' if harmonic else ''
    node_prefix = 'gazebo_harmonic' if harmonic else 'gazebo'

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{node_prefix}_{group}_bridge',
            parameters=[
                {
                    'config_file': str(
                        package_share
                        / 'config'
                        / f'{group}_bridge{config_suffix}.yaml'
                    )
                }
            ],
            output='screen',
        )
        for group in sensor_profile_bridge_groups(profile)
    ]


def sensor_profile_bridges(
    package_share: Path,
    *,
    harmonic: bool,
) -> OpaqueFunction:
    return OpaqueFunction(
        function=_sensor_bridge_nodes,
        kwargs={
            'package_share': package_share,
            'harmonic': harmonic,
        },
    )
