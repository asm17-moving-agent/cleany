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
FORTRESS_ODOMETRY_TOPIC = '/model/cleany_mecanum/ground_truth'
HARMONIC_ODOMETRY_TOPIC = '/model/cleany_mecanum/odometry'


def _entries(path: Path) -> list[dict[str, object]]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert isinstance(document, list)
    return document


def _ros_topics(path: Path) -> set[str]:
    return {
        str(entry['ros_topic_name'])
        for entry in _entries(path)
    }


def _entry_for_ros_topic(path: Path, topic: str) -> dict[str, object]:
    matches = [
        entry
        for entry in _entries(path)
        if entry['ros_topic_name'] == topic
    ]
    assert len(matches) == 1
    return matches[0]


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


@pytest.mark.parametrize(
    'filename',
    ('bridge.yaml', 'core_bridge.yaml', 'navigation_bridge.yaml'),
)
def test_fortress_uses_odometry_publisher_fallback(filename: str) -> None:
    path = CONFIG_ROOT / filename
    entry = _entry_for_ros_topic(path, '/gazebo_odom')
    ground_truth = _entry_for_ros_topic(path, '/ground_truth/odom')
    assert entry['gz_topic_name'] == FORTRESS_ODOMETRY_TOPIC
    assert entry['gz_type_name'] == 'ignition.msgs.Odometry'
    assert entry['ros_type_name'] == 'nav_msgs/msg/Odometry'
    assert entry['direction'] == 'GZ_TO_ROS'
    assert ground_truth['gz_topic_name'] == FORTRESS_ODOMETRY_TOPIC
    assert ground_truth['gz_type_name'] == 'ignition.msgs.Odometry'


@pytest.mark.parametrize(
    'filename',
    (
        'bridge_harmonic.yaml',
        'core_bridge_harmonic.yaml',
        'navigation_bridge_harmonic.yaml',
    ),
)
def test_harmonic_uses_mecanum_drive_odometry(filename: str) -> None:
    path = CONFIG_ROOT / filename
    entry = _entry_for_ros_topic(path, '/gazebo_odom')
    ground_truth = _entry_for_ros_topic(path, '/ground_truth/odom')
    assert entry['gz_topic_name'] == HARMONIC_ODOMETRY_TOPIC
    assert entry['gz_type_name'] == 'gz.msgs.Odometry'
    assert entry['ros_type_name'] == 'nav_msgs/msg/Odometry'
    assert entry['direction'] == 'GZ_TO_ROS'
    assert ground_truth['gz_topic_name'] == (
        '/model/cleany_mecanum/ground_truth'
    )
    assert ground_truth['gz_topic_name'] != entry['gz_topic_name']
