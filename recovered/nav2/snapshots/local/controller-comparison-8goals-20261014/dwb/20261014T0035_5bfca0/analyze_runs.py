#!/usr/bin/env python3
"""Reproducible descriptive metrics; raw results are never overwritten."""
import argparse, bisect, collections, csv, hashlib, json, math
from pathlib import Path
import numpy as np

LIMITS=[.15,.10,.14]
STAGES=['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel','/gazebo_cmd_vel']
def wheel(v):
 x,y,z=v;k=(.35+.6038)/2
 return max(abs(x-y-k*z),abs(x+y+k*z),abs(x+y-k*z),abs(x-y+k*z))/.0635

def read_json(p,default):
 return json.loads(p.read_text()) if p.exists() else default

def distribution(a):
 a=np.asarray(a,float)
 if not len(a):return None
 return {'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'p99':float(np.percentile(a,99)),'max':float(np.max(a)),'count':len(a)}

def smoothness(path,start,end):
 if not path.exists():return None
 a=np.atleast_2d(np.genfromtxt(path,delimiter=','));a=a[(a[:,0]>=start)&(a[:,0]<=end)]
 if len(a)<12:return None
 _,idx=np.unique(a[:,0],return_index=True);a=a[np.sort(idx)];t=np.arange(a[0,0],a[-1,0],.05)
 v=np.column_stack([np.interp(t,a[:,0],a[:,i]) for i in [1,2,3]])
 sm=np.column_stack([np.convolve(v[:,i],np.ones(5)/5,mode='same') for i in range(3)])
 acc=np.gradient(sm,.05,axis=0);jerk=np.gradient(acc,.05,axis=0)
 result={'sample_period_sim_s':.05,'centered_smoothing_points':5,'trim_each_end_s':.25,'velocity_frame':'GT message twist frame'}
 for name,a in [('acceleration',acc[5:-5]),('jerk',jerk[5:-5])]:
  for axis,vals in [('linear',np.linalg.norm(a[:,:2],axis=1)),('angular',np.abs(a[:,2]))]:
   result[axis+'_'+name+'_rms']=float(np.sqrt(np.mean(vals**2)));result[axis+'_'+name+'_p95']=float(np.percentile(vals,95))
 moving=(np.linalg.norm(v[:,:2],axis=1)>=.01)|(np.abs(v[:,2])>=.02)
 result.update(stationary_sim_s=float(np.sum(~moving)*.05),motion_start_count=int(np.sum(moving[1:]&~moving[:-1])),motion_stop_count=int(np.sum(~moving[1:]&moving[:-1])),mean_linear_speed_m_s=float(np.mean(np.linalg.norm(v[:,:2],axis=1))))
 for index,name,dead in [(0,'longitudinal',.01),(1,'lateral',.01),(2,'yaw',.02)]:
  signs=np.sign(v[np.abs(v[:,index])>dead,index]);result[name+'_reversal_count']=int(np.sum(signs[1:]!=signs[:-1]))
 return result

def summarize(p):
 r=read_json(p/'result.json',{});r['run_directory']=str(p)
 events=read_json(p/'events.json',[])
 start=r.get('navigation_sim_start_s',next((x['sim_s'] for x in events if x['phase']=='comparison_navigation'),float('-inf')))
 end=r.get('navigation_sim_end_s',start+r.get('navigation_sim_s',float('inf')))
 timing=p/'compute_timing.csv'
 if timing.exists() and timing.stat().st_size>100:
  a=np.atleast_1d(np.genfromtxt(timing,delimiter=',',names=True))
  a=a[(a['sim_time_ns']*1e-9>=start)&(a['sim_time_ns']*1e-9<=end)]
  us=a['compute_us']/1000;r['compute_ms']=distribution(us)
  if r['compute_ms']:r['compute_ms'].update(exception_count=int(np.sum(a['success']==0)),over_50ms=int(np.sum(us>50)))
  r['compute_success_ms']=distribution(us[a['success']!=0]);r['compute_exception_ms']=distribution(us[a['success']==0]);r['compute_start_wall_intervals_s']=distribution(np.diff(a['wall_start_ns'])*1e-9);r['compute_start_sim_intervals_s']=distribution(np.diff(a['sim_time_ns'])*1e-9)
 joints=read_json(p/'joints.json',[]);vals=[abs(v) for j in joints if start<=j['t']<=end for n,v in zip(j['names'],j['velocity']) if 'wheel' in n]
 r['measured_wheel_max_rad_s']=max(vals) if vals else None
 commands=[]
 if (p/'commands.csv').exists():
  for z in csv.reader((p/'commands.csv').open()):
   if start<=float(z[0])<=end:commands.append([float(z[0]),float(z[1]),z[2],*[float(v) for v in z[3:]]])
 stages={};times={}
 for topic in STAGES:
  c=[x for x in commands if x[2]==topic]
  if not c:continue
  v=np.asarray([x[3:] for x in c]);maxima=np.max(np.abs(v),axis=0).tolist()
  nonzero=[x for x in c if max(map(abs,x[3:]))>1e-6];last=nonzero[-1][0] if nonzero else None
  stop=next((x[0] for x in c if last is not None and x[0]>=last and max(map(abs,x[3:]))<=1e-6),None)
  stages[topic]={'abs_max_vx_vy_wz':maxima,'box_observed':all(x<=lim+1e-8 for x,lim in zip(maxima,LIMITS)),'max_predicted_wheel_rad_s':max(wheel(x[3:]) for x in c),'count':len(c),'last_nonzero_sim_s':last,'first_zero_after_last_nonzero_sim_s':stop}
  dt=np.diff([x[0] for x in c]);times[topic]=distribution(dt[dt>0])
 r['command_stages']=stages;r['command_sim_intervals_s']=times
 r['command_wall_intervals_s']={topic:distribution(np.diff([x[1] for x in commands if x[2]==topic])) for topic in stages}
 poses=read_json(p/'poses.json',[])
 if 'settled_pose_snapshot' in r:
  snap=r['settled_pose_snapshot'];gt=snap['GT'];goal=r['goal_map'];mp=snap.get('map')
  if mp:
   r['estimated_xy_error_m']=math.dist(mp['pose'][:2],goal[:2]);r['GT_estimate_delta_m']=math.dist(mp['pose'][:2],gt[:2]);r['estimated_yaw_error_deg']=abs(math.degrees(math.atan2(math.sin(mp['pose'][2]-goal[2]),math.cos(mp['pose'][2]-goal[2]))))
  od=snap.get('odom')
  if od:
   home=[-1.865,-4.705,1.5708];ox,oy,ow=od['pose'];ch,sh=math.cos(home[2]),math.sin(home[2]);world_od=[home[0]+ch*ox-sh*oy,home[1]+sh*ox+ch*oy,home[2]+ow]
   r['odom_pose_world_from_initial_pose']=world_od;r['odom_conversion_note']='Odometry is start-local; rotate and translate by locked home before world GT comparison';r['GT_odom_delta_m']=math.dist(world_od[:2],gt[:2])
 elif (p/'samples.csv').exists() and r.get('goal_map'):
  rows=list(csv.DictReader((p/'samples.csv').open()))
  if rows:
   z=rows[-1];est=[float(z['tf_x']),float(z['tf_y'])];gt=[float(z['gt_x']),float(z['gt_y'])]
   r['estimated_xy_error_m']=math.dist(est,r['goal_map'][:2]);r['GT_estimate_delta_m']=math.dist(gt,est)
 for file,key in [('guard_diagnostics.json','guard_decision_counts'),('pan_diagnostics.json','pan_reason_counts')]:
  counts=collections.Counter()
  for x in read_json(p/file,[]):
   if start<=x['sim_s']<=end:
    d=json.loads(x['data']);counts[d.get('reason',d.get('decision','unknown'))]+=1
  r[key]=dict(counts)
 raw=read_json(p/'resources.json',[]);by={}
 for x in raw:
  for proc in x['processes']:
   if 'controller_server' in proc['cmd']:by.setdefault(proc['pid'],[]).append((x['wall_s'],proc['cpu_s'],proc['rss_bytes']))
 r['controller_process_resources']=[{'pid':pid,'cpu_percent_one_core':100*(v[-1][1]-v[0][1])/(v[-1][0]-v[0][0]) if len(v)>1 else None,'RSS_max_MiB':max(x[2] for x in v)/2**20} for pid,v in by.items()]
 all_pids={}
 for x in raw:
  for proc in x['processes']:all_pids.setdefault(proc['pid'],[]).append((x['wall_s'],proc['cpu_s']))
 duration=r.get('navigation_wall_s')
 r['owned_launch_resource_summary']={'cpu_percent_one_core_equivalent':100*sum(max(0,v[-1][1]-v[0][1]) for v in all_pids.values())/duration if duration else None,'sum_RSS_peak_MiB':max((sum(proc['rss_bytes'] for proc in x['processes']) for x in raw),default=0)/2**20 if raw else None,'scope':'Only owned launch descendants. Sum RSS double-counts shared pages; bag/probe independent processes excluded. CPU uses first/last recorded per-PID counters divided by navigation wall duration.'}

 trace=read_json(p/'controller_command_dt.json',{}).get('records',[]);trace=[x for x in trace if start<=x['stamp_ns']*1e-9<=end]
 if trace:r['trajectory_count_per_cycle']=distribution([x['trajectories'] for x in trace])
 r['smoothness_recomputed']=smoothness(p/'velocity.csv',start,end)
 if (p/'samples.csv').exists():
  rows=[x for x in csv.DictReader((p/'samples.csv').open()) if start<=float(x['sim_s'])<=end];cnt=collections.Counter()
  for x in rows:
   raw=math.hypot(float(x['raw_vx']),float(x['raw_vy']))>.01;safe=math.hypot(float(x['safe_vx']),float(x['safe_vy']))>.01;actual=math.hypot(float(x['gt_vx']),float(x['gt_vy']))>.01
   if not raw:reason='upstream_translation_zero'
   elif not safe:reason='downstream_translation_zero'
   elif not actual:reason='commanded_translation_with_GT_stopped'
   else:reason='moving'
   cnt[reason]+=1
  r['translation_observer_sample_counts']=dict(cnt) # Translation-only diagnostic, not common stop metric
 r['file_sha256']={name:hashlib.sha256((p/name).read_bytes()).hexdigest() for name in ['map.pgm','world.sdf','world.model.json'] if (p/name).exists()}
 return r

def main():
 a=argparse.ArgumentParser();a.add_argument('input',type=Path);a.add_argument('--output',type=Path);args=a.parse_args();out=args.output or args.input
 out.mkdir(parents=True,exist_ok=True);results=[summarize(x.parent) for x in sorted(args.input.rglob('result.json')) if 'bag' not in x.parts]
 (out/'metrics.json').write_text(json.dumps(results,indent=2,allow_nan=False))
 fields=['goal_name','completed','action_status','timeout','position_error_m','yaw_error_deg','estimated_xy_error_m','GT_estimate_delta_m','GT_odom_delta_m','navigation_sim_s','navigation_wall_s','achieved_rtf','compute_median_ms','compute_p95_ms','compute_p99_ms','compute_exception_count','compute_over_50ms','controller_CPU_percent','controller_RSS_MiB','measured_wheel_max_rad_s','all_observed_stage_boxes','linear_jerk_rms','linear_jerk_p95','stationary_sim_s','motion_start_count','motion_stop_count','run_directory']
 with (out/'metrics.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
  for r in results:
   row={k:r.get(k) for k in fields};c=r.get('compute_ms') or {};sm=r.get('smoothness_recomputed') or {};res=r.get('controller_process_resources') or [{}]
   row.update(compute_median_ms=c.get('median'),compute_p95_ms=c.get('p95'),compute_p99_ms=c.get('p99'),compute_exception_count=c.get('exception_count'),compute_over_50ms=c.get('over_50ms'),controller_CPU_percent=res[0].get('cpu_percent_one_core'),controller_RSS_MiB=res[0].get('RSS_max_MiB'),all_observed_stage_boxes=all(x['box_observed'] for x in r['command_stages'].values()) if r['command_stages'] else None)
   row.update({k:sm.get(k) for k in ['linear_jerk_rms','linear_jerk_p95','stationary_sim_s','motion_start_count','motion_stop_count']});w.writerow(row)
 print(json.dumps([{k:r.get(k) for k in ['goal_name','action_status','position_error_m','navigation_sim_s','compute_ms','pan_reason_counts']} for r in results],indent=2))
if __name__=='__main__':main()
