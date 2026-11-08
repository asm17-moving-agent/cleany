"""Exercise both phone launch graphs and their actual merged controller config."""
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml
from launch import LaunchContext

from cleany_navigation import launch_runtime


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('depth', [False, True])
def test_disabling_slip_recovery_keeps_observation_and_collision_protection(tmp_path, monkeypatch, enabled, depth):
    package = Path(__file__).resolve().parents[1]
    get_share = launch_runtime.get_package_share_directory
    monkeypatch.setattr(launch_runtime, 'get_package_share_directory',
                        lambda name: str(package) if name == 'cleany_navigation' else get_share(name))
    monkeypatch.setattr(launch_runtime, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(launch_runtime, 'GroupAction', lambda actions: {'group': actions})
    monkeypatch.setattr(launch_runtime, 'SetRemap', lambda **kwargs: kwargs)
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    map_file = tmp_path/'map.yaml'
    map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    context = LaunchContext()
    context.launch_configurations.update({
        'map': str(map_file), 'use_sim_time': 'false', 'autostart': 'true',
        'params_file': str(package/'config/nav2_amcl.yaml'), 'localization_params_file': '',
        'robot_params_file': str(package/'config/phone_lidar_robot.yaml'),
        'phone_motion_profile': 'omni', 'slip_recovery': str(enabled).lower(),
        'localization_evidence_directory': str(tmp_path/'localization'),
        'depth_params_file': str(package/'config/phone_depth_local.yaml') if depth else '',
    })
    actions = launch_runtime._launch(context, safety=False, lidar_only=True)
    nodes = [a for a in actions if 'executable' in a]
    nodes += [a for group in actions if 'group' in group for a in group['group'] if 'executable' in a]
    assert sum(n['name'] == 'slip_monitor' for n in nodes) == 1
    assert sum(n['name'] == 'localization_guard' for n in nodes) == 1
    assert sum(n['name'] == 'traction_brake' for n in nodes) == int(enabled)
    assert sum(n.get('namespace') == 'traction_retry' and n['name'] == 'planner_server' for n in nodes) == int(enabled)
    collision = next(n for n in nodes if n['name'] == 'collision_monitor')['parameters'][0]
    assert collision['cmd_vel_in_topic'] == ('motion/traction_cmd_vel' if enabled else 'motion/cmd_vel')
    assert collision['cmd_vel_out_topic'] == 'navigation/safe_cmd_vel'
    assert collision['StopZone']['action_type'] == 'stop'
    assert collision['observation_sources'] == (['scan', 'depth'] if depth else ['scan'])
    assert sum(n['name'] == 'depth_clearance' for n in nodes) == int(depth)
    assert not any(n['name'] in ('voxel_guard', 'obstacle_memory', 'reactive_guard') for n in nodes)
    controller = next(n for n in nodes if n['name'] == 'controller_server')
    config = yaml.safe_load(Path(controller['parameters'][0]).read_text())
    assert ('depth_layer' in config['local_costmap']['local_costmap']['ros__parameters']['plugins']) == depth
    assert 'depth_layer' not in config['global_costmap']['global_costmap']['ros__parameters']['plugins']
    if depth:
        layer = config['local_costmap']['local_costmap']['ros__parameters']['depth_layer']
        assert layer['depth']['sensor_frame'] == 'd435_depth_frame'
        assert layer['depth']['marking'] and not layer['depth']['clearing']
        assert layer['depth_clear']['clearing'] and not layer['depth_clear']['marking']
        producer = next(n for n in nodes if n['name'] == 'depth_clearance')['parameters'][0]
        assert Path(producer['self_filter_boxes_file']).is_file()
        assert not producer['positive_infinity_is_free']
    ctrl = config['controller_server']['ros__parameters']
    assert ctrl['FollowPath']['motion_model'] == 'Omni'
    assert ctrl['FollowPath']['DirectionalMotionCritic']['diagonal_weight'] == 20.
    assert ('Longitudinal' in ctrl) == enabled
    tree_file = config['bt_navigator']['ros__parameters'].get('default_nav_to_pose_bt_xml', '')
    tree = ET.parse(tree_file).getroot() if tree_file else None
    assert (tree is not None and tree.find('.//SlipRecovery') is not None) == enabled
