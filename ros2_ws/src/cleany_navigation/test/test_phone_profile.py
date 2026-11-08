from pathlib import Path
import yaml
from cleany_navigation.runtime_config import merge_parameters,longitudinal_lattice
ROOT=Path(__file__).resolve().parents[1]


def test_phone_profiles_keep_tuned_plugin_and_default_disallows_lateral():
    for name,model,vy in [('longitudinal','DiffDrive',0.),('omni','Omni',.5)]:
        p=yaml.safe_load((ROOT/f'config/nav2_phone_{name}.yaml').read_text())
        c=p['controller_server']['ros__parameters']['FollowPath']
        assert c['plugin']=='cleany_navigation::MPPIController'
        assert c['motion_model']==model and c['vy_max']==vy
        planner=p['planner_server']['ros__parameters']['GridBased']
        if name=='omni':
            assert planner['plugin']=='nav2_smac_planner::SmacPlannerLattice'
            assert 'holonomic_direct_path' not in planner
        else:
            assert planner['plugin']=='cleany_navigation::StraightFirstPlanner'
            assert planner['holonomic_direct_path'] is False
            assert planner['lattice_collision_padding']==0.
    launch=(ROOT/'launch/phone_navigation.launch.py').read_text()
    assert "phone_motion_profile='longitudinal'" in launch


def test_lidar_demo_has_only_laser_collision_source():
    p=yaml.safe_load((ROOT/'config/phone_lidar_safety.yaml').read_text())
    monitor=p['collision_monitor']['ros__parameters']
    assert monitor['observation_sources']==['scan']
    assert monitor['cmd_vel_in_topic']=='motion/cmd_vel'
    assert monitor['cmd_vel_out_topic']=='navigation/safe_cmd_vel'
    assert monitor['source_timeout']==.5
    assert 'depth' not in monitor
    launch=(ROOT/'launch/phone_navigation.launch.py').read_text()
    assert 'lidar_only=True' in launch


def test_phone_wheel_speed_and_absolute_obstacle_limit_are_consistent():
    radius, arm = .0635, (.35 + .59) / 2
    for name in ('longitudinal', 'omni'):
        config=yaml.safe_load((ROOT/f'config/nav2_phone_{name}.yaml').read_text())
        controller=config['controller_server']['ros__parameters']['FollowPath']
        if name == 'longitudinal':
            assert abs(controller['vx_max']/radius - 4.) < 1e-12
            assert abs(controller['wz_max']*arm/radius - 4.) < 1e-12
        else:
            assert controller['vx_max'] == controller['vy_max'] == controller['translation_max'] == .5
            assert controller['wz_max'] == .7
    safety=yaml.safe_load((ROOT/'config/phone_lidar_safety.yaml').read_text())
    monitor=safety['collision_monitor']['ros__parameters']
    assert monitor['wheel_limit_rad_s']==10.
    assert monitor['translation_limit_m_s']==.16
    assert monitor['rotation_limit_rad_s']==.3
    assert monitor['wheel_radius_m']==radius
    assert monitor['wheelbase_m']==.35 and monitor['wheel_separation_m']==.59
    assert monitor['absolute_limit_polygon']=='SlowZone'
    assert monitor['SlowZone']['slowdown_ratio']==1.


def test_slow_region_is_wheel_envelope_plus_sixteen_cm():
    import ast
    robot=yaml.safe_load((ROOT/'config/phone_lidar_robot.yaml').read_text())
    safety=yaml.safe_load((ROOT/'config/phone_lidar_safety.yaml').read_text())
    slow=ast.literal_eval(safety['collision_monitor']['ros__parameters']['SlowZone']['points'])
    footprint=ast.literal_eval(robot['local_costmap']['local_costmap']['ros__parameters']['footprint'])
    for axis in (0,1):
        assert abs(min(p[axis] for p in slow) - min(p[axis] for p in footprint) + .16)<1e-9
        assert abs(max(p[axis] for p in slow) - max(p[axis] for p in footprint) - .16)<1e-9


def test_longitudinal_profile_scores_in_place_path_rotations():
    config=yaml.safe_load((ROOT/'config/nav2_phone_longitudinal.yaml').read_text())
    controller=config['controller_server']['ros__parameters']['FollowPath']
    assert controller['PathAngleCritic']['respect_in_place_rotations'] is True
    assert controller['TwirlingCritic']['enabled'] is True
    assert controller['path_rotate_enabled'] is True
    assert controller['GoalAngleCritic']['threshold_to_consider'] <= config['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']

def test_omni_profile_separates_body_heading_from_travel_and_projects_wheel_limits():
    config=yaml.safe_load((ROOT/'config/nav2_phone_omni.yaml').read_text())
    controller=config['controller_server']['ros__parameters']['FollowPath']
    assert controller['PathAngleCritic']['enabled'] is False
    assert controller['PathAlignCritic']['use_path_orientations'] is False
    assert controller['path_rotate_enabled'] is False
    assert controller['goal_rotate_enter_radius']==controller['goal_rotate_exit_radius']==0.
    assert controller['validate_output'] is True
    assert controller['vx_std']==controller['vy_std']>0.
    assert controller['wheel_max_rad_s']==10.
    assert controller['translation_max']==.5
    assert controller['wz_max']==.7
    assert controller['proximity_translation_max']==.16
    assert controller['proximity_yaw_max']==.3
    assert controller['goal_translation_max']==.06
    assert controller['goal_yaw_max']==.15
    assert controller['wheel_radius_m']==.0635
    assert controller['wheelbase_m']==.35 and controller['wheel_separation_m']==.59
    assert controller['GoalCritic']['threshold_to_consider']==controller['PathFollowCritic']['threshold_to_consider']
    assert controller['GoalCritic']['threshold_to_consider'] < controller['time_steps']*.05*controller['vx_max']
    planner=config['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin']=='nav2_smac_planner::SmacPlannerLattice'
    assert not {'holonomic_direct_path','require_exact_goal','prefix_shortcut_distance'} & planner.keys()
    assert planner['max_planning_time']==2.


def test_stop_region_is_cad_wheel_envelope_plus_five_cm():
    import ast
    robot=yaml.safe_load((ROOT/'config/phone_lidar_robot.yaml').read_text())
    safety=yaml.safe_load((ROOT/'config/phone_lidar_safety.yaml').read_text())
    stop=ast.literal_eval(safety['collision_monitor']['ros__parameters']['StopZone']['points'])
    for name in ('local_costmap','global_costmap'):
        params=robot[name][name]['ros__parameters']
        points=ast.literal_eval(params['footprint'])
        assert params['footprint_padding']==.05
        for axis in (0,1):
            assert abs(min(p[axis] for p in stop)-(min(p[axis] for p in points)-.05))<1e-9
            assert abs(max(p[axis] for p in stop)-(max(p[axis] for p in points)+.05))<1e-9


def test_balanced_primitives_preserve_geometry_and_translation_lengths():
    import json
    import math
    original=json.loads((ROOT/'config/lattice/omni-r10cm.json').read_text())
    balanced=json.loads((ROOT/'config/lattice/omni-r10cm-balanced.json').read_text())
    assert balanced['lattice_metadata']==original['lattice_metadata']
    corrected=0
    for before,after in zip(original['primitives'],balanced['primitives'],strict=True):
        changed={key for key in before if before[key]!=after[key]}
        assert changed <= {'trajectory_length','straight_length'}
        corrected+=bool(changed)
        if after['arc_length']==0. and after['start_angle_index']==after['end_angle_index']:
            poses=[[0.,0.],*after['poses']]
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))
            assert abs(after['trajectory_length']-length)<1e-5
            assert abs(after['straight_length']-length)<1e-5
    assert corrected==16
    assert len(balanced['primitives'])==144
    launch=(ROOT/'cleany_navigation/launch_runtime.py').read_text()
    assert "resolve_default_lattice(parameters, share/'config/lattice/omni-r10cm-balanced.json')" in launch


def test_directional_cost_preserves_collision_and_all_holonomic_candidates():
    base=yaml.safe_load((ROOT/'config/nav2_amcl.yaml').read_text())
    expected=base['controller_server']['ros__parameters']['FollowPath']['critics']
    merge_parameters(base,yaml.safe_load((ROOT/'config/nav2_phone_omni.yaml').read_text()))
    c=base['controller_server']['ros__parameters']['FollowPath']
    assert set(expected) < set(c['critics'])
    assert c['DirectionalMotionCritic']==dict(enabled=True,lateral_weight=1.,diagonal_weight=20.)
    assert c['motion_model']=='Omni' and c['vy_max']>0 and c['vy_std']>0
