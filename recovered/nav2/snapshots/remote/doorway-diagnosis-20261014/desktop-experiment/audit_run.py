"""Verify saved run invariants; report measured dynamics separately from command limits."""
from pathlib import Path
import argparse,ast,json,math,yaml
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=a.run
params=yaml.safe_load((r/'parameters.yaml').read_text());result=json.loads((r/'result.json').read_text());scenario=json.loads((r/'scenario.json').read_text())
c=params['controller_server']['ros__parameters'];f=c['FollowPath'];checks={}
def record(k,v):checks[k]=bool(v)
fp=[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]
for k in ['local_costmap','global_costmap']:
 v=params[k][k]['ros__parameters'];record(k+'_full_footprint',ast.literal_eval(v['footprint'])==fp);record(k+'_zero_padding',v['footprint_padding']==0)
record('final_goal_30cm_10deg',c['goal_checker']['xy_goal_tolerance']==.30 and c['goal_checker']['yaw_goal_tolerance']==math.radians(10))
if 'stage_goal_checker' in c:
 loaded=yaml.safe_load((r/'controller_server.yaml').read_text())['/controller_server']['ros__parameters']
 record('stage_checker_loaded_as_configured',all(loaded['stage_goal_checker'].get(k)==v for k,v in c['stage_goal_checker'].items()))
record('mecanum_wheel_configuration',all(f[k]==v for k,v in dict(wheel_max_rad_s=5.,wheel_radius_m=.0635,wheelbase_m=.35,wheel_separation_m=.6038).items()))
record('pathalign_nonzero_offset',f['PathAlignCritic']['offset_from_furthest']==2)
record('native_footprint_critic',f['CostCritic']['enabled'] and f['CostCritic']['consider_footprint'])
planner=params['planner_server']['ros__parameters']['GridBased']
record('stock_smac_lattice',planner['plugin']=='nav2_smac_planner/SmacPlannerLattice' and planner['allow_reverse_expansion'] and not planner['smooth_path'] and planner['max_iterations']==-1 and planner['max_planning_time']==80.)
lattice=json.loads(Path(planner['lattice_filepath']).read_text());meta=lattice['lattice_metadata']
record('lattice_5cm_0p5m',meta['grid_resolution']==.05 and meta['turning_radius']==.5 and meta['motion_model']=='omni')
record('amcl_motion_thresholds',params['amcl']['ros__parameters']['update_min_d']==.01 and params['amcl']['ros__parameters']['update_min_a']==.01)
record('scene_geometry_preserved',scenario['physical_models_unchanged_before_contact_sensors'])
record('actual_scene_loaded',not result.get('server_scene_loading',{}).get('missing_models',['unverified']))
command=result.get('max_command_wheel_rad_s',{}).get('/cmd_vel')
record('emitted_command_wheels_le_5',command is not None and command<=5.+1e-9)
if f.get('path_progress_timeout',0)>0:
 record('bounded_failure',c['failure_tolerance']==.5 and c['progress_checker']['movement_time_allowance']==20. and f['path_progress_timeout']==12.)
 record('output_validation_enabled',f['validate_output'])
out={'checks':checks,'passed':all(checks.values()),'failed':[k for k,v in checks.items() if not v],'measured_joint_limit_note':'Check metrics.json measured_joint_max_rad_s separately; command projection does not clamp physics joint impulses.'}
(r/'audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
raise SystemExit(0 if out['passed'] else 1)
