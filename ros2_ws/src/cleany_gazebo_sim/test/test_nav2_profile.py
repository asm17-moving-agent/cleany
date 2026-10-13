from __future__ import annotations

import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NAV2_CONFIG = PACKAGE_ROOT / 'config' / 'nav2_amcl.yaml'
NAV2_LAUNCH = PACKAGE_ROOT / 'launch' / 'amcl_nav2.launch.py'
MAP_YAML = PACKAGE_ROOT / 'maps' / 'study_cafe_26cm.yaml'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'


def _nav2_params(node_name: str) -> dict[str, object]:
    document = yaml.safe_load(NAV2_CONFIG.read_text(encoding='utf-8'))
    return document[node_name]['ros__parameters']


def _costmap_params(costmap_name: str) -> dict[str, object]:
    document = yaml.safe_load(NAV2_CONFIG.read_text(encoding='utf-8'))
    return document[costmap_name][costmap_name]['ros__parameters']


def test_amcl_uses_omnidirectional_gazebo_contract() -> None:
    params = _nav2_params('amcl')

    assert params['global_frame_id'] == 'map'
    assert params['odom_frame_id'] == 'odom'
    assert params['base_frame_id'] == 'base_link'
    assert params['scan_topic'] == 'scan'
    assert params['robot_model_type'] == 'nav2_amcl::OmniMotionModel'
    assert params['laser_min_range'] == 0.15
    assert params['laser_max_range'] == 12.0
    assert params['set_initial_pose'] is True
    assert params['initial_pose'] == {
        'x': 0.0,
        'y': 0.0,
        'z': 0.0,
        'yaw': 0.0,
    }


def test_mppi_controller_preserves_mecanum_lateral_motion() -> None:
    params = _nav2_params('controller_server')
    controller = params['FollowPath']

    assert params['min_y_velocity_threshold'] == 0.001
    assert controller['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert controller['motion_model'] == 'Omni'
    assert controller['PathAngleCritic']['mode'] == 1
    assert controller['vy_max'] > 0.0
    assert controller['vx_max'] <= 0.3
    assert controller['vy_max'] <= 0.3
    assert controller['wz_max'] <= 0.8


def test_costmaps_share_rectangular_footprint_and_scan_contract() -> None:
    local = _costmap_params('local_costmap')
    global_ = _costmap_params('global_costmap')

    assert local['robot_base_frame'] == 'base_link'
    assert local['global_frame'] == 'odom'
    assert global_['robot_base_frame'] == 'base_link'
    assert global_['global_frame'] == 'map'
    assert local['footprint'] == global_['footprint']
    assert local['obstacle_layer']['scan']['topic'] == '/scan'
    assert global_['obstacle_layer']['scan']['topic'] == '/scan'
    circumscribed_radius = math.hypot(0.30, 0.25) + 0.02
    assert (
        local['inflation_layer']['inflation_radius']
        >= circumscribed_radius
    )
    assert (
        global_['inflation_layer']['inflation_radius']
        >= circumscribed_radius
    )


def test_saved_map_metadata_references_installed_pgm() -> None:
    metadata = yaml.safe_load(MAP_YAML.read_text(encoding='utf-8'))
    image_path = MAP_YAML.parent / metadata['image']

    assert metadata['resolution'] == 0.05
    assert metadata['origin'] == [-0.836, -8.074, 0.0]
    assert image_path.is_file()
    with image_path.open('rb') as stream:
        assert stream.readline().strip() == b'P5'
        assert stream.readline().strip() == b'222 249'


def test_launch_starts_minimal_amcl_nav2_stack() -> None:
    source = NAV2_LAUNCH.read_text(encoding='utf-8')
    ast.parse(source)

    for package in (
        'nav2_map_server',
        'nav2_amcl',
        'nav2_controller',
        'nav2_planner',
        'nav2_behaviors',
        'nav2_bt_navigator',
        'nav2_lifecycle_manager',
    ):
        assert f"package='{package}'" in source
    assert "'node_names': ['map_server', 'amcl']" in source
    assert "'controller_server'" in source
    assert "'bt_navigator'" in source


def test_package_declares_nav2_runtime_dependencies() -> None:
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {
        element.text
        for tag in ('depend', 'exec_depend')
        for element in root.findall(tag)
    }

    assert {
        'nav2_amcl',
        'nav2_behaviors',
        'nav2_bt_navigator',
        'nav2_controller',
        'nav2_lifecycle_manager',
        'nav2_map_server',
        'nav2_mppi_controller',
        'nav2_navfn_planner',
        'nav2_planner',
    } <= dependencies
