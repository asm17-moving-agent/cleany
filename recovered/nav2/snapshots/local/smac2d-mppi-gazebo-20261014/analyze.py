#!/usr/bin/env python3
"""Reproducible descriptive metrics; action status, truth and failures kept separate."""
import csv,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
BASE=ROOT.parent.parent.parent/'controller-comparison-20261013'
sys.path.insert(0,str(BASE))
from analyze_comparison import metrics

def summary(path):
 row=metrics(path);d=json.loads(path.read_text());p=path.parent
 start=d.get('navigation_sim_start_s');end=d.get('navigation_sim_end_s')
 if start is None:
  events=json.loads((p/'events.json').read_text()) if (p/'events.json').exists() else []
  start=next((e['sim_s'] for e in events if e['phase']=='comparison_navigation'),None)
  end=start+d.get('navigation_sim_s',0) if start is not None else None
 row['stage']=d.get('stage','relaxed_gazebo_only' if '/relaxed/' in str(path) else 'conservative')
 row['nav2_arrival']=d.get('action_status')==4
 row['gt_xy_within_30cm']=d.get('position_error_m',math.inf)<=.30
 row['gt_yaw_within_10deg']=d.get('yaw_error_deg',math.inf)<=10
 row['outcome']='preparation_only' if d.get('preparation_only') else 'arrival' if row['nav2_arrival'] else 'timeout' if d.get('timeout') else 'abort' if d.get('completed') else 'infrastructure_error'
 row['command_sim_interval_mean_s']=(d.get('command_sim_interval_s',{}).get('/nav2/cmd_vel') or {}).get('mean')
 row['command_sim_interval_p95_s']=(d.get('command_sim_interval_s',{}).get('/nav2/cmd_vel') or {}).get('p95')
 tfile=p/'compute_timing.csv'
 if tfile.exists():
  calls=list(csv.DictReader(tfile.open()))
  calls=[r for r in calls if start is None or start<=int(r['sim_time_ns'])*1e-9<=end]
  values=np.array([float(r['compute_us'])/1000 for r in calls])
  for label,pct in [('median',50),('p95',95),('p99',99)]:row['compute_'+label+'_ms']=float(np.percentile(values,pct)) if len(values) else None
  row['compute_calls']=len(calls);row['compute_exception_calls']=sum(r['success']!='1' for r in calls);row['compute_over_50ms']=int(np.sum(values>50))
  good=np.array([float(r['compute_us'])/1000 for r in calls if r['success']=='1'])
  row['compute_success_p95_ms']=float(np.percentile(good,95)) if len(good) else None
  row['compute_success_p99_ms']=float(np.percentile(good,99)) if len(good) else None
  for key,scale,label in [('sim_time_ns',1e-9,'sim'),('wall_start_ns',1e-9,'wall')]:
   intervals=np.diff([int(x[key])*scale for x in calls]);intervals=intervals[intervals>0]
   for suffix,percentile in [('median',50),('p95',95),('p99',99)]:row['compute_call_'+label+'_period_'+suffix+'_s']=float(np.percentile(intervals,percentile)) if len(intervals) else None
 cfile=p/'commands.csv'
 if cfile.exists():
  commands=list(csv.reader(cfile.open()));active=[r for r in commands if start is not None and start<=float(r[0])<=end]
  for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel']:
   vals=np.array([[float(r[i]) for i in [3,4,5]] for r in active if r[2]==topic]);name=topic.strip('/').replace('/','_')
   if len(vals):
    for i,key in enumerate(['vx','vy','wz']):row['max_'+name+'_'+key]=float(np.max(np.abs(vals[:,i])))
    row[name+'_coupled_wheel_exceedances']=int(np.sum((np.abs(vals[:,0])+np.abs(vals[:,1])+.4769*np.abs(vals[:,2]))/.0635>5+1e-6))
    row['max_'+name+'_wheel_rad_s']=float(np.max((np.abs(vals[:,0])+np.abs(vals[:,1])+.4769*np.abs(vals[:,2]))/.0635))
    topic_rows=[r for r in active if r[2]==topic]
    for clock_index,clock_name in [(0,'sim'),(1,'wall')]:
     intervals=np.diff([float(r[clock_index]) for r in topic_rows]);intervals=intervals[intervals>0]
     row[name+'_'+clock_name+'_period_mean_s']=float(np.mean(intervals)) if len(intervals) else None
     row[name+'_'+clock_name+'_period_p95_s']=float(np.percentile(intervals,95)) if len(intervals) else None
 ds=json.loads((p/'diagnostics.json').read_text()) if (p/'diagnostics.json').exists() else []
 active=[x for x in ds if start is not None and start<=x['received_sim_s']<=end]
 for topic,key in [('/head_pan/state','reason'),('/body_guard_3d/state','decision'),('/wheel_limit/state','reason')]:
  selected=[x for x in active if x['topic']==topic];counts={}
  for x in selected:counts[x.get(key,'unknown')]=counts.get(x.get(key,'unknown'),0)+1
  row[topic.strip('/').replace('/','_')+'_sample_counts']=counts
  if topic=='/head_pan/state':
   requested=[x for x in selected if max(map(abs,x.get('requested',[0,0,0])))>1e-5]
   row['pan_nonzero_request_samples']=len(requested);row['pan_blocked_nonzero_samples']=sum(max(map(abs,x.get('output',[0,0,0])))<1e-5 for x in requested)
 timeline=[]
 for topic,key in [('/head_pan/state','reason'),('/body_guard_3d/state','decision'),('/wheel_limit/state','reason')]:
  previous=None
  for x in (x for x in active if x['topic']==topic):
   reason=x.get(key,'unknown')
   if reason!=previous:
    timeline.append({k:x[k] for k in ['topic','received_sim_s','sim_s','requested','output'] if k in x}|{'reason':reason})
    previous=reason
 (p/'command_reason_timeline.json').write_text(json.dumps(sorted(timeline,key=lambda x:x['received_sim_s']),indent=2))
 if (p/'velocity.csv').exists() and start is not None:
  a=np.asarray(list(csv.reader((p/'velocity.csv').open())),float);a=a[(a[:,0]>=start)&(a[:,0]<=end)]
  if len(a)>12:
   _,idx=np.unique(a[:,0],return_index=True);a=a[np.sort(idx)];t=np.arange(a[0,0],a[-1,0],.05)
   v=np.column_stack([np.interp(t,a[:,0],a[:,i]) for i in [1,2,3]])
   sm=np.column_stack([np.convolve(v[:,i],np.ones(5)/5,mode='same') for i in range(3)])
   accel=np.gradient(sm,.05,axis=0)[5:-5];speed=np.linalg.norm(v[:,:2],axis=1);moving=(speed>=.01)|(np.abs(v[:,2])>=.02)
   row['gt_speed_max_m_s']=float(np.max(speed))
   row['gt_yaw_rate_max_rad_s']=float(np.max(np.abs(v[:,2])))
   row['full_stationary_s']=float(np.sum(~moving)*.05);row['full_motion_start_count']=int(np.sum(moving[1:]&~moving[:-1]))
   for label,x in [('linear',np.linalg.norm(accel[:,:2],axis=1)),('angular',np.abs(accel[:,2]))]:
    row[label+'_acceleration_rms']=float(np.sqrt(np.mean(x*x)));row[label+'_acceleration_p95']=float(np.percentile(x,95))
 row['minimum_true_footprint_clearance_m']=None
 row['minimum_true_footprint_clearance_null_reason']='Static-world footprint clearance not separately instrumented; contact sensing is not clearance'
 row['environment_contact_samples']=None
 row['environment_contact_episodes']=None
 row['environment_contact_max_depth_m']=None
 row['environment_contact_null_reason']='Contact sensing unavailable' if row['stage']=='conservative' else 'Contact sensing missing or navigation did not start'
 row['environment_contact_depth_null_reason']='No measured environment contact depth'
 contacts_file=p/'contacts.jsonl'
 if contacts_file.exists() and start is not None:
  counts={'ground':0,'environment':0,'self':0,'unknown':0};ages=[];topics=set();pairs=set();depths=[];stamps=set()
  with contacts_file.open() as stream:
   for line in stream:
    c=json.loads(line)
    if not start<=c['stamp_s']<=end:continue
    names=[c['collision1']['name'],c['collision2']['name']]
    kind='unknown' if any(not n for n in names) else 'ground' if any('ground_plane' in n for n in names) else 'self' if all('cleany_mecanum' in n for n in names) else 'environment'
    counts[kind]+=1;ages.append(c['received_sim_s']-c['stamp_s']);topics.add(c['topic'])
    if kind=='environment':pairs.add(tuple(sorted(names)));depths.extend(c['depths']);stamps.add(c['stamp_s'])
  live=counts['ground']>0 and counts['unknown']==0
  row.update(contact_ground_samples=counts['ground'],contact_self_samples=counts['self'],contact_unknown_name_samples=counts['unknown'],contact_sensor_live=counts['ground']>0,contact_topics_observed=sorted(topics),
   contact_received_age_p95_s=float(np.percentile(ages,95)) if ages else None,contact_received_age_max_s=max(ages) if ages else None,
   environment_contact_samples=counts['environment'] if live else None,environment_contact_pairs=sorted(pairs),environment_contact_max_depth_m=max(depths) if depths else None,
   environment_contact_null_reason=None if live else 'No ground heartbeat or unknown collision names',environment_contact_depth_null_reason=None if depths else 'No environment contact depths were provided',soft_240s_exceeded=d.get('soft_240s_exceeded',False))
  stamps=sorted(stamps);row['environment_contact_episodes']=sum(i==0 or t-stamps[i-1]>.05 for i,t in enumerate(stamps)) if live else None
 row['stationary_definition']='|v_xy|<.01 AND |wz|<.02rad/s'
 row['hard_deadline_observed']=('wall_1800s' if d.get('navigation_wall_s',0)>=1800 else 'sim_1200s' if d.get('navigation_sim_s',0)>=1200 else None)
 row['xy_tolerance_m']=.30;row['yaw_tolerance_deg']=10.;row['wheel_limit_rad_s']=5.
 required=['compute_median_ms','compute_p95_ms','compute_p99_ms','compute_exception_calls','compute_over_50ms','max_nav2_cmd_vel_wheel_rad_s','max_cmd_vel_wheel_rad_s','max_measured_wheel_rad_s','full_stationary_s','full_motion_start_count','linear_jerk_rms_m_s3','angular_jerk_rms_rad_s3','controller_cpu_one_core_pct','controller_rss_p95_mb']
 row['unmeasured_metric_reasons']={}
 for key in required:
  row.setdefault(key,None)
  if row[key] is None:row['unmeasured_metric_reasons'][key]='No valid active-navigation samples; see result/error/raw files'
 # Store local outcome metrics alongside each raw run, without rewriting result.json.
 row={k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in row.items()}
 (p/'analysis.json').write_text(json.dumps(row,indent=2,allow_nan=False))
 return row

def main():
 rows=[summary(p) for p in sorted(ROOT.glob('**/result.json')) if 'mppi_ws' not in str(p)]
 (ROOT/'all_attempts.json').write_text(json.dumps(rows,indent=2))
 final=[r for r in rows if '/final_validation/' in r['result_path']]
 for stage in ['conservative','relaxed_gazebo_only']:
  subset=[r for r in final if r['stage']==stage]
  (ROOT/(stage+'_results.json')).write_text(json.dumps(subset,indent=2))
  if subset:
   keys=sorted(set(k for r in subset for k in r))
   with (ROOT/(stage+'_results.csv')).open('w') as f:
    writer=csv.DictWriter(f,keys);writer.writeheader();writer.writerows([{k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in r.items()} for r in subset])
 (ROOT/'results.json').write_text(json.dumps(final,indent=2))
 if final:
  keys=sorted(set(k for r in final for k in r))
  with (ROOT/'results.csv').open('w') as f:
   w=csv.DictWriter(f,keys);w.writeheader();w.writerows([{k:json.dumps(v) if isinstance(v,(dict,list)) else v for k,v in r.items()} for r in final])
 print(json.dumps([{k:r.get(k) for k in ['goal_name','outcome','position_error_cm','yaw_error_deg','sim_s','rtf','compute_p95_ms','linear_jerk_rms_m_s3','angular_jerk_rms_rad_s3','pan_blocked_nonzero_samples','max_measured_wheel_rad_s','max_cmd_vel_vx','max_cmd_vel_vy','max_cmd_vel_wz','result_path']} for r in rows],indent=2))
if __name__=='__main__':main()
