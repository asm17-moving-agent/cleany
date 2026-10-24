#!/usr/bin/env python3
from pathlib import Path
import json,hashlib,yaml
ROOT=Path(__file__).resolve().parent
paths=sorted((ROOT/'final_validation').glob('*/result.json'));assert len(paths)==8
rows=[json.loads(p.read_text()) for p in paths];assert [r['goal_index'] for r in rows]==list(range(8));assert all(r.get('navigation_goal_sent') for r in rows)
assert len({json.dumps(r['loaded_mppi_libraries_sha256'],sort_keys=True) for r in rows})==1
configs=[]
for p,r in zip(paths,rows):
 c=yaml.safe_load((p.parent/'controller_server.yaml').read_text());c['/controller_server']['ros__parameters']['FollowPath'].pop('benchmark_timing_file',None);configs.append(c)
 assert r['verified_relaxed_parent_parameters']['goal_checker.xy_goal_tolerance']==.30
 assert r['verified_relaxed_parent_parameters']['goal_checker.yaw_goal_tolerance']==.17453292519943295
 assert r['verified_costmap_resolution']['resolution']==.025
 assert r['verified_common_planner_parameters']['GridBased.body_heading_grid_bins']==32
 assert r['verified_common_planner_parameters']['GridBased.max_planning_time']==80.
 assert not r['server_scene_loading']['missing_models'] and not r['server_scene_loading']['missing_mesh_errors']
 assert len(r['server_scene_loading']['model_names'])==r['scene_validation']['preserved_model_count']
assert all(c==configs[0] for c in configs)
analyses=json.loads((ROOT/'results.json').read_text());assert len(analyses)==8
limits={}
for r in analyses:
 limits[r['goal_name']]={'raw_command_max':r.get('max_nav2_cmd_vel_wheel_rad_s'),'final_command_max':r.get('max_cmd_vel_wheel_rad_s'),'joint_max':r.get('max_measured_wheel_rad_s'),'raw_exceedance_samples':r.get('nav2_cmd_vel_coupled_wheel_exceedances'),'final_exceedance_samples':r.get('cmd_vel_coupled_wheel_exceedances')}
(ROOT/'final_evidence_audit.json').write_text(json.dumps({'indices_each_once':True,'navigation_goals_sent':8,'actual_controller_parameters_identical_except_timing_output_path':True,'loaded_library_hashes_identical':True,'xy30cm_yaw10deg_verified_all':True,'matching_global_resolution_025_verified_all':True,'shared_planner_80s_32headings_verified_all':True,'actual_all_scene_models_and_meshes_verified':True,'wheel_limit_observations':limits},indent=2))
(ROOT/'final_controller_profile.yaml').write_text(yaml.safe_dump(configs[0],sort_keys=False));print('8-goal evidence consistency audited')
