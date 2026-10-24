#!/usr/bin/env python3
import argparse,collections,csv,importlib.util,json,math
from pathlib import Path
import numpy as np
spec=importlib.util.spec_from_file_location('common_analysis',Path(__file__).parents[1]/'analyze_runs.py');common=importlib.util.module_from_spec(spec);spec.loader.exec_module(common)
common.STAGES=['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/shadow/cmd_vel','/cmd_vel','/gazebo_cmd_vel']

def contacts(p,start,end):
 records=common.read_json(p/'contacts.json',[]);counts=common.read_json(p/'contact_topic_counts.json',{})
 categories=collections.Counter();pairs={};events=[]
 for row in records:
  t=row['stamp_s'] if row.get('stamp_s',0)>0 else row['sim_s']
  if not start<=t<=end:continue
  for c in row['contacts']:
   a,b=c['collision1'],c['collision2'];names=a+' '+b
   if 'ground_plane' in names:category='wheel_ground' if 'wheel' in names else 'body_ground'
   elif 'cleany_mecanum' in a and 'cleany_mecanum' in b:category='self_contact'
   elif 'cleany_mecanum' in names:category='environment_contact'
   else:category='unclassified'
   categories[category]+=1;pair=tuple(sorted([a,b]));key=category+'|'+pair[0]+'|'+pair[1]
   d=pairs.setdefault(key,{'category':category,'collision_pair':list(pair),'first_sim_s':t,'last_sim_s':t,'sample_count':0,'episode_count':0,'max_depth_m':None})
   if not d['sample_count'] or t-d['last_sim_s']>.1:d['episode_count']+=1
   d['sample_count']+=1;d['last_sim_s']=t
   if c['depths']:d['max_depth_m']=max(max(c['depths']),d['max_depth_m'] or 0.)
   if category in ['environment_contact','body_ground','unclassified']:events.append({'sim_s':t,'category':category,'pair':list(pair),'GT':row.get('GT'),'depths':c['depths'],'positions':c['positions']})
 available=sum(counts.values())>0 and categories['wheel_ground']>0
 result={'available':available,'unavailable_reason':None if available else 'No wheel-ground physics-contact heartbeat; contact absence cannot be interpreted as zero collisions','configured_sensor_update_rate_hz':20,'physics_update_rate_hz':500,'floor_json_max_log_rate_hz':20,'raw_bag_retains_unthrottled_contacts':True,'penetration_depth_m':None,'penetration_depth_unavailable_reason':'ROS bridge Contact depths arrays were empty','configured_sensor_count':len(common.read_json(p/'contact_instrumentation.json',{}).get('sensors',[])),'topic_message_counts':counts,'navigation_sample_counts':dict(categories),'environment_contact_samples':categories['environment_contact'] if available else None,'body_ground_contact_samples':categories['body_ground'] if available else None,'environment_contact_episodes':sum(v['episode_count'] for v in pairs.values() if v['category']=='environment_contact') if available else None,'pairs':list(pairs.values()),'events':events,'limitations':'Fortress publishes persistent physics contacts at approximately 500Hz despite the 20Hz sensor setting; floor-only JSON is subsampled to 20Hz, raw bag is unthrottled, and environment/body contacts are unthrottled. DDS loss or missed transients remain possible. No physical proof. Zero events means zero reported events, not an absolute continuous collision-free certificate.'}
 (p/'contact_analysis.json').write_text(json.dumps(result,indent=2));return result

def summarize(p):
 r=common.summarize(p)
 if r.get('completed'):r['timeout']=bool(r.get('timeout',False));r.setdefault('soft_240_sim_s_crossed_at',None);r['soft_240_timestamp_null_reason']='No240sim-second crossing' if r['soft_240_sim_s_crossed_at'] is None else None
 r['pose_snapshot_note']='Reported terminal errors use saved settled snapshot2sim seconds after action; GT and map TF samples are asynchronous and not an atomic capture of the goal-checker pose'
 if r.get('action_pose_snapshot'):
  snap=r['action_pose_snapshot'];goal=r['goal_map'];r['action_GT_xy_error_m']=math.dist(snap['GT'][:2],goal[:2]);r['action_map_xy_error_m']=math.dist(snap['map']['pose'][:2],goal[:2]) if snap.get('map') else None
 start=r.get('navigation_sim_start_s',float('-inf'));end=r.get('navigation_sim_end_s',float('inf'))
 for s in r['command_stages'].values():s.pop('box_observed',None);s['wheel_hard_cap_observed']=s['max_predicted_wheel_rad_s']<=5.+1e-8
 r['independent_vx_vy_wz_box_applied']=False;r['collision_measurement']=contacts(p,start,end)
 relay=[]
 for x in common.read_json(p/'relay_diagnostics.json',[]):
  if start<=x['sim_s']<=end:relay.append(json.loads(x['data']))
 r['relay_reason_counts']=dict(collections.Counter(x['reason'] for x in relay));r['relay_additional_scale_count']=sum(x['scale']<1.-1e-9 for x in relay)
 r['controller_failure_events']=[]
 if (p/'compute_timing.csv').exists() and (p/'compute_timing.csv').stat().st_size>100:
  for x in csv.DictReader((p/'compute_timing.csv').open()):
   t=int(x['sim_time_ns'])*1e-9
   if start<=t<=end and x['success']=='0':r['controller_failure_events'].append({'sim_s':t,'steady_wall_start_ns':int(x['wall_start_ns']),'compute_ms':float(x['compute_us'])/1000,'reason':'controller exception; inspect launch.log for Nav2 detail'})
 return r

def main():
 a=argparse.ArgumentParser();a.add_argument('directory',type=Path);args=a.parse_args();root=args.directory
 rows=[summarize(p.parent) for p in sorted(root.rglob('result.json')) if 'bag' not in p.parts]
 (root/'metrics.json').write_text(json.dumps(rows,indent=2,allow_nan=False))
 fields=['goal_name','completed','action_status','timeout','position_error_m','yaw_error_deg','estimated_xy_error_m','GT_estimate_delta_m','GT_odom_delta_m','navigation_sim_s','navigation_wall_s','achieved_rtf','compute_median_ms','compute_p95_ms','compute_p99_ms','compute_exceptions','compute_over_50ms','CPU_percent_one_core','RSS_MiB','command_wheel_max_rad_s','joint_wheel_max_rad_s','reported_environment_contact_episodes','body_ground_contact_samples','contact_measurement_available','linear_jerk_rms','linear_jerk_p95','stationary_sim_s','motion_start_count','motion_stop_count','soft_240_sim_s_crossed_at','run_directory']
 with (root/'metrics.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
  for r in rows:
   x={k:r.get(k) for k in fields};c=r.get('compute_ms') or {};sm=r.get('smoothness_recomputed') or {};cpu=(r.get('controller_process_resources') or [{}])[0];contact=r['collision_measurement'];st=r['command_stages'].get('/cmd_vel',{})
   x.update(compute_median_ms=c.get('median'),compute_p95_ms=c.get('p95'),compute_p99_ms=c.get('p99'),compute_exceptions=c.get('exception_count'),compute_over_50ms=c.get('over_50ms'),CPU_percent_one_core=cpu.get('cpu_percent_one_core'),RSS_MiB=cpu.get('RSS_max_MiB'),command_wheel_max_rad_s=st.get('max_predicted_wheel_rad_s'),joint_wheel_max_rad_s=r.get('measured_wheel_max_rad_s'),reported_environment_contact_episodes=contact['environment_contact_episodes'],body_ground_contact_samples=contact['body_ground_contact_samples'],contact_measurement_available=contact['available'])
   x.update({k:sm.get(k) for k in ['linear_jerk_rms','linear_jerk_p95','stationary_sim_s','motion_start_count','motion_stop_count']});w.writerow(x)
 print(json.dumps([{k:r.get(k) for k in ['goal_name','completed','action_status','navigation_sim_s','position_error_m','compute_ms','measured_wheel_max_rad_s']} for r in rows],indent=2))
if __name__=='__main__':main()
