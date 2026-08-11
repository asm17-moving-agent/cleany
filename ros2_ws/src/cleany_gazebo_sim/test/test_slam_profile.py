from __future__ import annotations

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SLAM_CONFIG = PACKAGE_ROOT / 'config' / 'slam_toolbox.yaml'
SLAM_LAUNCH = PACKAGE_ROOT / 'launch' / 'slam_mapping.launch.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'
FORTRESS_WORLD = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_prototype.sdf'
HARMONIC_WORLD = PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'


def _slam_params() -> dict[str, object]:
    document = yaml.safe_load(SLAM_CONFIG.read_text(encoding='utf-8'))
    return document['slam_toolbox']['ros__parameters']


def _lidar_range(world: Path) -> tuple[float, float]:
    root = ET.parse(world).getroot()
    lidar = root.find(".//sensor[@name='rplidar_a1']/lidar/range")
    assert lidar is not None
    minimum = lidar.findtext('min')
    maximum = lidar.findtext('max')
    assert minimum is not None
    assert maximum is not None
    return float(minimum), float(maximum)


def test_slam_toolbox_uses_gazebo_navigation_contract() -> None:
    params = _slam_params()

    assert params['mode'] == 'mapping'
    assert params['scan_topic'] == '/scan'
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'
    assert params['transform_publish_period'] > 0.0


def test_async_profile_bounds_scan_queue_and_uses_loop_closure() -> None:
    params = _slam_params()

    assert params['scan_queue_size'] == 1
    assert params['use_scan_matching'] is True
    assert params['do_loop_closing'] is True
    assert params['enable_interactive_mode'] is False


def test_slam_laser_limits_match_both_gazebo_profiles() -> None:
    params = _slam_params()
    expected = (
        float(params['min_laser_range']),
        float(params['max_laser_range']),
    )

    assert _lidar_range(FORTRESS_WORLD) == expected
    assert _lidar_range(HARMONIC_WORLD) == expected


def test_launch_uses_async_slam_toolbox_and_accepts_overrides() -> None:
    source = SLAM_LAUNCH.read_text(encoding='utf-8')
    ast.parse(source)

    assert "package='slam_toolbox'" in source
    assert "executable='async_slam_toolbox_node'" in source
    assert "'slam_params_file'" in source
    assert "'use_sim_time'" in source


def test_package_declares_slam_toolbox_runtime_dependency() -> None:
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {
        element.text
        for tag in ('depend', 'exec_depend')
        for element in root.findall(tag)
    }

    assert 'slam_toolbox' in dependencies
