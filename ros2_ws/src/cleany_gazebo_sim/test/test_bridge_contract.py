from pathlib import Path

import pytest
import yaml

from cleany_gazebo_sim.sensor_profile_launch import (
    SENSOR_PROFILES,
    sensor_profile_bridge_groups,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PACKAGE_ROOT / 'config' / 'bridge'
EXPECTED_GROUP_TOPICS = {
    'core': {
        '/gazebo_cmd_vel',
        '/gazebo_odom',
        '/ground_truth/odom',
        '/joint_states',
        '/clock',
        '/imu/data',
    },
    'lidar': {'/scan'},
    'head_rgbd': {
        '/camera/head/color/image_raw',
        '/camera/head/depth/image_raw',
    },
    'left_wrist': {'/camera/left_wrist/color/image_raw'},
    'right_wrist': {'/camera/right_wrist/color/image_raw'},
}
NAVIGATION_TOPICS = {
    '/gazebo_cmd_vel',
    '/gazebo_odom',
    '/ground_truth/odom',
    '/clock',
    '/scan',
    '/imu/data',
}


def _entries(path: Path) -> list[dict[str, object]]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert isinstance(document, list)
    return document


def _ros_topics(path: Path) -> set[str]:
    return {
        str(entry['ros_topic_name'])
        for entry in _entries(path)
    }


def test_sensor_profiles_select_only_their_bridge_groups() -> None:
    assert SENSOR_PROFILES == (
        'lidar_nav',
        'head_rgbd',
        'left_wrist',
        'right_wrist',
        'all_cameras',
    )
    assert sensor_profile_bridge_groups('lidar_nav') == ('core', 'lidar')
    assert sensor_profile_bridge_groups('head_rgbd') == ('core', 'head_rgbd')
    assert sensor_profile_bridge_groups('left_wrist') == ('core', 'left_wrist')
    assert sensor_profile_bridge_groups('right_wrist') == (
        'core',
        'right_wrist',
    )
    assert sensor_profile_bridge_groups('all_cameras') == (
        'core',
        'head_rgbd',
        'left_wrist',
        'right_wrist',
    )
    with pytest.raises(ValueError, match='unknown sensor profile'):
        sensor_profile_bridge_groups('all_sensors')


@pytest.mark.parametrize(
    ('suffix', 'transport_namespace'),
    (('', 'ignition.msgs.'), ('_harmonic', 'gz.msgs.')),
)
def test_split_bridge_configs_match_ros_topic_contract(
    suffix: str,
    transport_namespace: str,
) -> None:
    for group, expected_topics in EXPECTED_GROUP_TOPICS.items():
        path = CONFIG_ROOT / f'{group}_bridge{suffix}.yaml'
        entries = _entries(path)
        assert _ros_topics(path) == expected_topics
        assert all(
            str(entry['gz_type_name']).startswith(transport_namespace)
            for entry in entries
        )


@pytest.mark.parametrize(
    ('filename', 'transport_namespace'),
    (
        ('navigation_bridge.yaml', 'ignition.msgs.'),
        ('navigation_bridge_harmonic.yaml', 'gz.msgs.'),
    ),
)
def test_navigation_bridge_exposes_only_runtime_topics(
    filename: str,
    transport_namespace: str,
) -> None:
    path = CONFIG_ROOT / filename
    entries = _entries(path)
    assert _ros_topics(path) == NAVIGATION_TOPICS
    assert all('/camera/' not in topic for topic in _ros_topics(path))
    assert all(
        str(entry['gz_type_name']).startswith(transport_namespace)
        for entry in entries
    )
