from __future__ import annotations

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SLAM_CONFIG = PACKAGE_ROOT / 'config' / 'slam' / 'slam_toolbox.yaml'
SLAM_LAUNCH = PACKAGE_ROOT / 'launch' / 'slam_mapping.launch.py'
PACKAGE_XML = PACKAGE_ROOT / 'package.xml'


def _slam_params() -> dict[str, object]:
    document = yaml.safe_load(SLAM_CONFIG.read_text(encoding='utf-8'))
    return document['slam_toolbox']['ros__parameters']


def test_slam_toolbox_uses_cleany_navigation_contract() -> None:
    params = _slam_params()

    assert params['mode'] == 'mapping'
    assert params['scan_topic'] == '/scan'
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'
    assert params['use_sim_time'] is False
    assert params['transform_publish_period'] > 0.0


def test_async_profile_bounds_scan_queue_and_uses_loop_closure() -> None:
    params = _slam_params()

    assert params['scan_queue_size'] == 1
    assert params['use_scan_matching'] is True
    assert params['do_loop_closing'] is True
    assert params['enable_interactive_mode'] is False


def test_launch_uses_async_slam_toolbox_and_accepts_overrides() -> None:
    source = SLAM_LAUNCH.read_text(encoding='utf-8')
    ast.parse(source)

    assert "package='slam_toolbox'" in source
    assert "executable='async_slam_toolbox_node'" in source
    assert "namespace=''" in source
    assert 'LifecycleNode' in source
    assert 'Transition.TRANSITION_CONFIGURE' in source
    assert 'Transition.TRANSITION_ACTIVATE' in source
    assert "'slam_params_file'" in source
    assert "'use_sim_time'" in source
    assert "'do_loop_closing'" in source
    assert "default_value='false'" in source
    assert "'loop_search_maximum_distance'" in source
    assert "'loop_search_space_dimension'" in source
    assert "'loop_match_minimum_response_coarse'" in source
    assert "'loop_match_minimum_response_fine'" in source


def test_package_declares_slam_toolbox_runtime_dependency() -> None:
    root = ET.parse(PACKAGE_XML).getroot()
    dependencies = {
        element.text
        for tag in ('depend', 'exec_depend')
        for element in root.findall(tag)
    }

    assert 'slam_toolbox' in dependencies
    assert 'lifecycle_msgs' in dependencies
