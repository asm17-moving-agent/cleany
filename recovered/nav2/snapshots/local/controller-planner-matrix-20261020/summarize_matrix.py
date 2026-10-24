"""Summarize observed runs without inventing pooled trial statistics."""
from pathlib import Path
import ast,csv,json,math,statistics,hashlib,copy
import yaml
ROOT=Path(__file__).resolve().parent
COMBOS=['dwb_lattice','dwb_theta','mppi_lattice','mppi_theta']
NAMES=['straight','turn_arrival','chair_gap','far_heading','distant','lateral','narrow_seat','other_room']
rows=[]; audits=[];failures=[]
def pct(v,p):
 if not v:return None
 v=sorted(v); x=(len(v)-1)*p/100; a=int(x); b=min(a+1,len(v)-1)
 return v[a]+(v[b]-v[a])*(x-a)
def strip_paths(v):
 if isinstance(v,dict):return {k:strip_paths(x) for k,x in v.items() if k not in ['benchmark_timing_file','timing_trace_file','lattice_filepath','failure_snapshot_directory']}
 if isinstance(v,list):return [strip_paths(x) for x in v]
 return v
for i,name in enumerate(NAMES):
 contracts=[];configs=[];results=[]
 for combo in COMBOS:
  folder=ROOT/combo/f'{i}_{name}'
  r=json.loads((folder/'result.json').read_text());m=json.loads((folder/'metrics.json').read_text());c=json.loads((folder/'comparison_contract.json').read_text());p=yaml.safe_load((folder/'parameters.yaml').read_text())
  assert r['completed'] and r['navigation_goal_sent'],folder
  contracts.append(c);configs.append(p);results.append(r)
  compute=m.get('compute_timing',{}).get('compute_us',{});sm=r.get('smoothness',{})
  plans=json.loads((folder/'plans.json').read_text());first=plans[0]['poses'] if plans else []
  commands=[x for x in csv.reader((folder/'commands.csv').open()) if x[2]=='/nav2/cmd_vel' and r['navigation_sim_start_s']<=float(x[0])<=r['navigation_sim_end_s']]
  periods={k:[float(b[col])-float(a[col]) for a,b in zip(commands,commands[1:]) if float(b[col])>float(a[col])] for k,col in [('sim',0),('wall',1)]}
  row=dict(combination=combo,controller=combo.split('_')[0],planner=combo.split('_')[1],goal_index=i,goal_name=name,run_date='2026-10-20' if combo.startswith('dwb') else '2026-10-16',host=r.get('host'),nav2_status=r.get('action_status'),gt_tolerance_30cm_10deg=r['position_error_m']<=.30 and r['yaw_error_deg']<=10,gt_xy_cm=100*r['position_error_m'],gt_yaw_deg=r['yaw_error_deg'],sim_s=r['navigation_sim_s'],wall_s=r['navigation_wall_s'],actual_rtf=r['achieved_rtf'])
  row.update({f'compute_{k}_ms':compute.get(k)/1000 if compute.get(k) is not None else None for k in ['median','p95','p99']})
  row.update(planner_compute_ms=None,planner_compute_null_reason='Planner-only wall compute not instrumented',compute_timing_null_reason=None if compute else 'No TimedController compute timing samples recorded',first_plan_length_null_reason=None if first else 'No global /plan message recorded')
  for kind,v in periods.items():
   for k,pctile in [('median',50),('p95',95),('p99',99)]:row[f'command_interval_{kind}_{k}_ms']=pct(v,pctile)*1000 if v else None
  row.update(controller_requested_hz=20.,linear_jerk_rms=sm.get('linear_jerk_rms_m_s3'),linear_jerk_p95=sm.get('linear_jerk_p95_m_s3'),angular_jerk_rms=sm.get('angular_jerk_rms_rad_s3'),angular_jerk_p95=sm.get('angular_jerk_p95_rad_s3'),stationary_s=sm.get('stationary_s'),motion_starts=sm.get('motion_start_count'),yaw_reversals=sm.get('yaw_reversals'),lateral_reversals=sm.get('lateral_reversals'),mean_speed_m_s=sm.get('speed_mean_m_s'),max_wheel_command_rad_s=r.get('max_command_wheel_rad_s',{}).get('/cmd_vel'),max_wheel_joint_rad_s=m.get('measured_joint_max_rad_s'),non_floor_contact_records=m.get('contacts',{}).get('non_floor_non_self_records'),controller_deadline_warnings=r.get('controller_deadline_warning_count'),first_plan_length_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(first,first[1:])) if first else None,plan_count=len(plans),timeout=bool(r.get('timeout')),error=r.get('error'),source_folder=str(folder.resolve()))
  observed={}
  for sample in csv.DictReader((folder/'samples.csv').open()):
   t=float(sample['sim_s'])
   if r['navigation_sim_start_s']<=t<=r['navigation_sim_end_s']:
    values=[float(sample[k]) for k in ['gt_x','gt_y','gt_yaw']]
    if all(math.isfinite(v) for v in values):observed[t]=values
  observed=sorted(observed.items());near=unaligned=covered=0.
  for (ta,a),(tb,b) in zip(observed,observed[1:]):
   dt=tb-ta;covered+=dt
   if math.dist(a[:2],r['goal_map'][:2])<=.3:
    near+=dt
    if abs(math.atan2(math.sin(a[2]-r['goal_map'][2]),math.cos(a[2]-r['goal_map'][2])))>math.radians(10):unaligned+=dt
  row.update(gt_dwell_observed_sim_s=covered if len(observed)>1 else None,near_goal_xy_sim_s=near if len(observed)>1 else None,near_goal_unaligned_sim_s=unaligned if len(observed)>1 else None,near_goal_dwell_null_reason=None if len(observed)>1 else 'Insufficient GT samples for elapsed-time integration')
  rows.append(row)
  if r.get('action_status')!=4:
   log=(folder/'launch.log').read_text(errors='replace');terms=['No valid trajectories','No legal trajectories','Failed to make progress','Collision Ahead','no collision-free emitted rollout','backup failed','Goal failed','timed out','Aborting']
   failures.append(dict(combination=combo,goal=name,final_truth=r.get('final_truth'),remaining_m=r['position_error_m'],timeout=r.get('timeout',False),symptoms=[line for line in log.splitlines() if any(t in line for t in terms)][-35:],root_cause=None,root_cause_null_reason='Logs show symptoms; no causal tuning rerun in this comparison'))
 checks={k:len({str(c[k]) for c in contracts})==1 for k in ['bt_sha256','map_sha256','world_sha256','manual_waypoints','requested_rtf']}
 checks['same_goal']=all(r['goal_map']==results[0]['goal_map'] for r in results)
 checks['same_initial_gt']=all(math.dist(r['initial_truth_world'],results[0]['initial_truth_world'])<1e-6 for r in results)
 parents=[{k:v for k,v in p['controller_server']['ros__parameters'].items() if k!='FollowPath'} for p in configs]
 checks['same_parent_controller_envelope']=all(x==parents[0] for x in parents)
 for combo,p in zip(COMBOS,configs):
  ctrl=p['controller_server']['ros__parameters'];checks[combo+'_arrival']=ctrl['goal_checker']['xy_goal_tolerance']==.3 and ctrl['goal_checker']['yaw_goal_tolerance']==math.radians(10)
  for cm in ['local_costmap','global_costmap']:
   cfg=p[cm][cm]['ros__parameters'];checks[combo+'_'+cm+'_physical_footprint']=ast.literal_eval(cfg['footprint'])==[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]] and cfg['footprint_padding']==0
 for a,b in [(0,1),(2,3)]:
  key='loaded_dwb_libraries_sha256' if a==0 else 'loaded_mppi_libraries_sha256'
  checks[COMBOS[a].split('_')[0]+'_same_loaded_library_hashes']=bool(results[a].get(key)) and results[a].get(key)==results[b].get(key)
  checks[COMBOS[a].split('_')[0]+'_same_controller_profile']=strip_paths(configs[a]['controller_server'])==strip_paths(configs[b]['controller_server'])
 for a,b in [(0,2),(1,3)]:checks[COMBOS[a].split('_')[1]+'_same_planner_profile']=strip_paths(configs[a]['planner_server'])==strip_paths(configs[b]['planner_server'])
 audits.append(dict(goal=name,checks=checks))
common=[n for n in NAMES if all(r['nav2_status']==4 and not r['timeout'] for r in rows if r['goal_name']==n)]
summary=[]
for combo in COMBOS:
 allrows=[r for r in rows if r['combination']==combo];matched=[r for r in allrows if r['goal_name'] in common]
 s=dict(combination=combo,nav2_arrivals=sum(r['nav2_status']==4 for r in allrows),runs=8,gt_within_tolerance=sum(r['gt_tolerance_30cm_10deg'] for r in allrows),non_floor_contact_records=sum(r['non_floor_contact_records'] or 0 for r in allrows),max_wheel_command_rad_s=max((r['max_wheel_command_rad_s'] for r in allrows if r['max_wheel_command_rad_s'] is not None),default=None),max_wheel_joint_rad_s=max((r['max_wheel_joint_rad_s'] for r in allrows if r['max_wheel_joint_rad_s'] is not None),default=None),common_arrival_routes=common,common_arrival_n=len(matched))
 s['gt_xy_within_30cm']=sum(r['gt_xy_cm']<=30 for r in allrows)
 s['gt_yaw_within_10deg']=sum(r['gt_yaw_deg']<=10 for r in allrows)
 s['total_near_goal_unaligned_sim_s']=sum(r['near_goal_unaligned_sim_s'] for r in allrows) if all(r['near_goal_unaligned_sim_s'] is not None for r in allrows) else None
 for k in ['near_goal_unaligned_sim_s','sim_s','gt_xy_cm','gt_yaw_deg','linear_jerk_rms','linear_jerk_p95','angular_jerk_rms','stationary_s','compute_median_ms','compute_p95_ms','compute_p99_ms','actual_rtf']:
  v=[r[k] for r in matched if r[k] is not None];s['common_routes_mean_'+k]=statistics.mean(v) if v else None
 summary.append(s)
audit=dict(per_goal=audits,all_frozen_input_checks_equal=all(all(a['checks'].values()) for a in audits),execution_load_equal=False,execution_load_note='MPPI reused 2026-10-16, DWB 2026-10-20; same remote CPU host but unrelated GPU training active in new runs; concurrency and machine load differ')
(ROOT/'condition-audit.json').write_text(json.dumps(audit,indent=2))
if not audit['all_frozen_input_checks_equal']:raise RuntimeError('Condition mismatch: inspect condition-audit.json')
for name,data in [('comparison',rows),('summary',summary)]:
 with (ROOT/(name+'.csv')).open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
(ROOT/'comparison.json').write_text(json.dumps(dict(runs=rows,summary=summary,failures=failures,condition_audit=audit,replicates_per_route=1,jerk_method='GT twist, sim-time 20Hz resampling, centered five-sample mean, numerical acceleration/jerk gradient, 0.25s endpoint trim; linear magnitude RMS/p95 m/s^3, angular abs RMS/p95 rad/s^3',compute_scope='TimedController inner compute steady wall time incl enabled telemetry and collision validation, failed calls retained; excludes planner and simulator',command_interval_scope='Positive differences between observed /nav2/cmd_vel messages during navigation; wall and simulation times separate; requested 20Hz uses wall time; not controller-only scheduler timing',contact_scope='Observed non-floor/non-self sensor records; 0 is not independent geometric collision proof',limits='Goal tolerance 30cm/10deg; coupled output wheel command cap5rad/s, DWB internal candidate cap4.99; measured joint response may overshoot',safety_bypass='Simulator only: pan alignment, CollisionMonitor and 3D guard shadow outputs; native controller/costmap collision checks and output wheel limiter retained',notes=['Near-goal dwell integrates positive elapsed simulation time between recorded GT samples, using the left sample: XY<=0.30m and yaw error>10deg; approximate sampled duration, includes failed runs','Stationary deadband: GT linear speed<0.01m/s and abs yaw speed<0.02rad/s; tiny nonzero motion can be counted stationary; yaw reversal deadband0.02rad/s, GT vy reversal deadband0.01m/s','No composite score or pass threshold','One observed run per route, not repeated success-rate verification','Early abort/stagnation with low jerk is not a smoothness improvement','Timing affected by different dates, shared load and parallel simulation','Legacy stock144 Lattice radius50cm preserved to match prior MPPI comparison; not the later radius10cm regenerated primitive set']),indent=2))
print(json.dumps(dict(summary=summary,audit=audit['all_frozen_input_checks_equal']),indent=2))
