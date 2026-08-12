from pathlib import Path

import pytest
import yaml

from cleany_gazebo_sim.sensor_profile_launch import (
    SENSOR_PROFILES,
    sensor_profile_bridge_groups,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PACKAGE_ROOT / 'config'

EXPECTED_GROUP_TOPICS = {
    'core': {
        '/gazebo_cmd_vel',
        '/gazebo_odom',
        '/ground_truth/odom',
        '/joint_states',
        '/clock',
        '/imu/data',
    },
    'lidar': {'/scan', '/scan_12cm', '/scan_45cm', '/scan_70cm'},
    'head_rgbd': {
        '/camera/head/color/image_raw',
        '/camera/head/depth/image_raw',
    },
    'left_wrist': {'/camera/left_wrist/color/image_raw'},
    'right_wrist': {'/camera/right_wrist/color/image_raw'},
}


def _ros_topics(config_path: Path) -> set[str]:
    prefix = 'ros_topic_name:'
    return {
        line.split(prefix, maxsplit=1)[1].strip().strip('"')
        for line in config_path.read_text(encoding='utf-8').splitlines()
        if prefix in line
    }


def test_sensor_profiles_select_expected_bridge_groups():
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
    assert sensor_profile_bridge_groups('right_wrist') == ('core', 'right_wrist')
    assert sensor_profile_bridge_groups('all_cameras') == (
        'core',
        'head_rgbd',
        'left_wrist',
        'right_wrist',
    )


def test_unknown_sensor_profile_is_rejected():
    with pytest.raises(ValueError, match='unknown sensor profile'):
        sensor_profile_bridge_groups('all_sensors')


@pytest.mark.parametrize(
    ('height_cm', 'gz_scan_topic'),
    (
        (12, '/model/cleany_mecanum/lidar_12cm/scan'),
        (26, '/model/cleany_mecanum/lidar/scan'),
        (45, '/model/cleany_mecanum/lidar_45cm/scan'),
        (70, '/model/cleany_mecanum/lidar_70cm/scan'),
    ),
)
def test_height_slam_bridge_activates_only_selected_lidar(
    height_cm: int,
    gz_scan_topic: str,
):
    config_path = CONFIG_ROOT / (
        f'slam_{height_cm}cm_bridge_harmonic.yaml'
    )
    entries = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    scan_entries = [
        entry
        for entry in entries
        if entry['ros_type_name'] == 'sensor_msgs/msg/LaserScan'
    ]

    assert _ros_topics(config_path) == {
        *EXPECTED_GROUP_TOPICS['core'],
        '/scan',
    }
    assert len(scan_entries) == 1
    assert scan_entries[0]['ros_topic_name'] == '/scan'
    assert scan_entries[0]['gz_topic_name'] == gz_scan_topic


@pytest.mark.parametrize(
    ('suffix', 'expected_namespace'),
    (('', 'ignition.msgs.'), ('_harmonic', 'gz.msgs.')),
)
def test_split_bridge_configs_are_complete_and_version_isolated(
    suffix: str,
    expected_namespace: str,
):
    for group, expected_topics in EXPECTED_GROUP_TOPICS.items():
        config_path = CONFIG_ROOT / f'{group}_bridge{suffix}.yaml'
        config = config_path.read_text(encoding='utf-8')

        assert _ros_topics(config_path) == expected_topics
        assert expected_namespace in config
        if suffix:
            assert 'ignition.msgs.' not in config
