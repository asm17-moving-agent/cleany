from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


GAZEBO_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
NAVIGATION_PACKAGE_ROOT = GAZEBO_PACKAGE_ROOT.parent / 'cleany_navigation'
SLAM_CONFIG = (
    NAVIGATION_PACKAGE_ROOT / 'config' / 'slam' / 'slam_toolbox.yaml'
)
FORTRESS_WORLD = (
    GAZEBO_PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_fortress.sdf'
)
HARMONIC_WORLD = (
    GAZEBO_PACKAGE_ROOT / 'worlds' / 'cleany_mecanum_harmonic.sdf'
)


def _lidar_range(world: Path) -> tuple[float, float]:
    root = ET.parse(world).getroot()
    lidar = root.find(".//sensor[@name='rplidar_a1']/lidar/range")
    assert lidar is not None
    minimum = lidar.findtext('min')
    maximum = lidar.findtext('max')
    assert minimum is not None
    assert maximum is not None
    return float(minimum), float(maximum)


def test_slam_laser_limits_match_both_gazebo_profiles() -> None:
    document = yaml.safe_load(SLAM_CONFIG.read_text(encoding='utf-8'))
    params = document['slam_toolbox']['ros__parameters']
    expected = (
        float(params['min_laser_range']),
        float(params['max_laser_range']),
    )

    assert _lidar_range(FORTRESS_WORLD) == expected
    assert _lidar_range(HARMONIC_WORLD) == expected
