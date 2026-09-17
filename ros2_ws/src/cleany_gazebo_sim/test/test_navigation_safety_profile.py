"""Cross-check evaluation geometry and sensing limits against their sources."""

import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_camera_extrinsics_match_fixed_gazebo_head_chain():
    model = ET.parse(PACKAGE/'worlds/cleany_mecanum_harmonic.sdf').find('.//model[@name="cleany_mecanum"]')
    entities = {e.get('name'): e for tag in ('link', 'joint') for e in model.findall(tag)}

    def resolve(name):
        if name == 'base_link':
            return np.eye(4)
        pose = entities[name].find('pose')
        x, y, z, roll, pitch, yaw = map(float, pose.text.split())
        cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
        local = np.eye(4)
        local[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
        local[:3, 3] = [x, y, z]
        return resolve(pose.get('relative_to')) @ local

    camera = yaml.safe_load((PACKAGE/'config/nav2_safety.yaml').read_text())['depth_camera_tf']
    transform = resolve('head_camera_link')
    np.testing.assert_allclose(transform[:3, 3], camera['translation'], atol=1e-8)
    np.testing.assert_allclose(transform[:3, :3], np.eye(3), atol=1e-8)
    assert camera['rotation_rpy'] == [0.0, 0.0, 0.0]


def test_clearing_rays_cannot_be_marked_and_fit_the_actual_camera_clip():
    safety = yaml.safe_load((PACKAGE/'config/nav2_safety.yaml').read_text())
    projection = safety['depth_clearance']['ros__parameters']
    layer = safety['local_costmap']['local_costmap']['ros__parameters']['depth_layer']
    sensor = ET.parse(PACKAGE/'worlds/cleany_mecanum_harmonic.sdf').find('.//sensor[@name="head_realsense_depth"]')
    assert projection['sensor_far_clip'] == float(sensor.findtext('camera/clip/far'))
    assert projection['marking_max_range'] == layer['depth']['obstacle_max_range']
    assert layer['depth']['obstacle_max_range'] < projection['clearing_distance'] < projection['sensor_far_clip']
    assert layer['depth']['raytrace_max_range'] >= projection['clearing_distance']
    assert layer['depth']['min_obstacle_height'] < layer['min_obstacle_height']
    assert layer['depth']['max_obstacle_height'] > layer['max_obstacle_height']
    assert layer['origin_z'] < layer['min_obstacle_height'] < layer['max_obstacle_height'] < layer['origin_z']+layer['z_resolution']*layer['z_voxels']


def test_stop_zone_encloses_padded_footprint_and_slow_zone_encloses_stop():
    nav = yaml.safe_load((PACKAGE/'config/nav2_amcl.yaml').read_text())
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    safety = yaml.safe_load((PACKAGE/'config/nav2_safety.yaml').read_text())['collision_monitor']['ros__parameters']
    footprint = ast.literal_eval(local['footprint'])
    stop = ast.literal_eval(safety['StopZone']['points'])
    slow = ast.literal_eval(safety['SlowZone']['points'])
    for i in (0, 1):
        assert max(abs(p[i]) for p in footprint)+local['footprint_padding'] < max(abs(p[i]) for p in stop) < max(abs(p[i]) for p in slow)
    assert 0 < safety['SlowZone']['slowdown_ratio'] < 1
    assert safety['source_timeout'] > 0


def test_lidar_below_base_origin_is_not_filtered_from_either_costmap():
    nav = yaml.safe_load((PACKAGE/'config/nav2_amcl.yaml').read_text())
    lidar_z = 0.30-0.38
    for name in ('local_costmap', 'global_costmap'):
        layer = nav[name][name]['ros__parameters']['obstacle_layer']
        assert layer['min_obstacle_height'] < lidar_z
        assert layer['scan']['min_obstacle_height'] < lidar_z


def test_planning_envelope_includes_monitor_stop_region():
    safety = yaml.safe_load((PACKAGE/'config/nav2_safety.yaml').read_text())
    nav = yaml.safe_load((PACKAGE/'config/nav2_amcl.yaml').read_text())
    stop = ast.literal_eval(safety['collision_monitor']['ros__parameters']['StopZone']['points'])
    for name in ('local_costmap', 'global_costmap'):
        config = safety[name][name]['ros__parameters']
        footprint = ast.literal_eval(config['footprint'])
        padding = nav[name][name]['ros__parameters']['footprint_padding']
        for i in (0, 1):
            assert max(abs(p[i]) for p in footprint) >= max(abs(p[i]) for p in stop)
        radius = max(math.hypot(abs(x)+padding, abs(y)+padding) for x, y in footprint)
        assert config['inflation_layer']['inflation_radius'] >= radius


def test_goal_handoff_matches_available_prediction_distance():
    nav = yaml.safe_load((PACKAGE/'config/nav2_amcl.yaml').read_text())
    safety = yaml.safe_load((PACKAGE/'config/nav2_safety.yaml').read_text())
    controller = nav['controller_server']['ros__parameters']['FollowPath']
    overlay = safety['controller_server']['ros__parameters']['FollowPath']
    distance = overlay['time_steps']*controller['model_dt']*controller['vx_max']
    assert overlay['PathFollowCritic']['threshold_to_consider'] == distance
    assert overlay['GoalCritic']['threshold_to_consider'] == distance
    assert overlay['PathAlignCritic']['threshold_to_consider'] == distance
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    assert distance < min(local['width'], local['height'])/2
