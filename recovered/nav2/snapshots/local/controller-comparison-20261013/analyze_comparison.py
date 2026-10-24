#!/usr/bin/env python3
"""Produce descriptive metrics, preserving failures and host/RTF boundaries."""
import csv,json,math,statistics
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def metrics(p):
 d=json.loads(p.read_text());events=json.loads((p.parent/'events.json').read_text()) if (p.parent/'events.json').exists() else []
 start=next((e['sim_s'] for e in events if e['phase']=='comparison_navigation'),None)
 end=start+d.get('navigation_sim_s',0) if start is not None else None
 row={'candidate':d['candidate'],'goal_index':d['goal_index'],'goal_name':d['goal_name'],'completed':d['completed'],'action_status':d.get('action_status'),'timeout':d.get('timeout',False),'error':d.get('error',''),'host':d['host'],'position_error_cm':d.get('position_error_m',float('nan'))*100,'yaw_error_deg':d.get('yaw_error_deg'),'sim_s':d.get('navigation_sim_s'),'wall_s':d.get('navigation_wall_s'),'rtf':d.get('achieved_rtf'),'deadline_warning_count':d.get('controller_deadline_warning_count'),'result_path':str(p)}
 for k,v in d.get('smoothness',{}).items():
  if k!='available':row[k]=v
 intervals=d.get('controller_command_sim_interval_s') or {};row['command_sim_interval_mean_s']=intervals.get('mean');row['command_sim_interval_p95_s']=intervals.get('p95')
 for k,v in d.get('max_command_wheel_rad_s',{}).items():row['max_'+k.strip('/').replace('/','_')+'_wheel_rad_s']=v
 joints=json.loads((p.parent/'joints.json').read_text()) if (p.parent/'joints.json').exists() else []
 row['max_measured_wheel_rad_s']=max((abs(v) for j in joints if start is not None and start<=j['t']<=end for n,v in zip(j['names'],j['velocity']) if 'wheel' in n),default=None)
 resource=json.loads((p.parent/'resources.json').read_text()) if (p.parent/'resources.json').exists() else []
 cpu={};rss=[];stamp=[]
 for r in resource:
  for proc in r['processes']:
   if '/nav2_controller/controller_server' in proc['cmd']:
    cpu.setdefault(proc['pid'],[]).append(proc['cpu_s']);rss.append(proc['rss_bytes']/1048576);stamp.append(r['wall_s'])
 total=sum(max(v)-min(v) for v in cpu.values())
 window=max(stamp)-min(stamp) if len(stamp)>1 else None
 row['controller_cpu_s']=total if cpu else None
 row['controller_cpu_one_core_pct']=100*total/window if window else None
 row['controller_rss_p95_mb']=float(np.percentile(rss,95)) if rss else None
 row['controller_cpu_s_per_sim_s']=total/d['navigation_sim_s'] if cpu and d.get('navigation_sim_s') else None
 timing=p.parent/'compute_timing.csv'
 if timing.exists():
  calls=list(csv.DictReader(timing.open()));good=[float(r['compute_us'])/1000 for r in calls if r['success']=='1']
  row['compute_time_per_cycle']='measured'
  row['compute_calls']=len(calls);row['compute_exception_calls']=sum(r['success']!='1' for r in calls)
  row['compute_success_p95_ms']=float(np.percentile(good,95)) if good else None
  row['compute_success_p99_ms']=float(np.percentile(good,99)) if good else None
 else:row['compute_time_per_cycle']='unmeasured'
 rows=list(csv.DictReader((p.parent/'samples.csv').open())) if (p.parent/'samples.csv').exists() else []
 # Existing SafetyProbe saves trace.csv; accept either.
 if not rows and (p.parent/'trace.csv').exists():rows=list(csv.DictReader((p.parent/'trace.csv').open()))
 if start is not None:
  rows=[r for r in rows if start<=float(r['sim_s'])<=end]
 if rows:
  row['path_length_m']=sum(math.hypot(float(b['gt_x'])-float(a['gt_x']),float(b['gt_y'])-float(a['gt_y'])) for a,b in zip(rows,rows[1:]))
  row['rotation_travel_rad']=sum(abs(math.atan2(math.sin(float(b['gt_yaw'])-float(a['gt_yaw'])),math.cos(float(b['gt_yaw'])-float(a['gt_yaw'])))) for a,b in zip(rows,rows[1:]))
 return row

def main():
 paths=list(ROOT.glob('official_*/*/result.json'))+list(ROOT.glob('remote/official_*/*/result.json'))
 attempts=[metrics(p) for p in paths]
 (ROOT/'all_official_attempts.json').write_text(json.dumps(attempts,indent=2))
 chosen={}
 for r in sorted(attempts,key=lambda r:r['result_path']):
  k=(r['candidate'],r['goal_index'])
  if k not in chosen or r['completed']:chosen[k]=r
 rows=list(chosen.values())
 keys=sorted(set(k for r in rows for k in r))
 with (ROOT/'comparison.csv').open('w') as f:
  w=csv.DictWriter(f,keys);w.writeheader();w.writerows(sorted(rows,key=lambda r:(r['candidate'],r['goal_index'])))
 (ROOT/'comparison.json').write_text(json.dumps(rows,indent=2))
 summaries=[]
 for c in sorted(set(r['candidate'] for r in rows)):
  allrows=[r for r in rows if r['candidate']==c];valid=[r for r in allrows if r['completed']]
  summary={'candidate':c,'recorded':len(allrows),'completed':len(valid),'arrived_action_status_4':sum(r['action_status']==4 for r in valid),'hosts':sorted(set(r['host'] for r in allrows))}
  for k in ['position_error_cm','yaw_error_deg','sim_s','controller_cpu_one_core_pct','controller_rss_p95_mb','linear_jerk_rms_m_s3','angular_jerk_rms_rad_s3']:
   subset=[r for r in valid if r.get('action_status')==4] if k in ['position_error_cm','yaw_error_deg'] else valid
   v=[r[k] for r in subset if r.get(k) is not None and math.isfinite(r[k])];summary[('arrival_' if k in ['position_error_cm','yaw_error_deg'] else '')+k+'_median']=statistics.median(v) if v else None
  summaries.append(summary)
 (ROOT/'summary.json').write_text(json.dumps(summaries,indent=2));print(json.dumps(summaries,indent=2))
 notes='''# Nav2 controller comparison — 2026-10-13\n\nFive common static-map routes per candidate; no pass/fail score. Position tolerance 15 cm and heading tolerance 10 degrees. All candidates preserve the common planner/costmaps/BT/safety chain.\n\nWheel limit uses CAD radius 0.0635 m, wheelbase 0.35 m, track 0.6038 m. Conservative common command box vx=0.15, vy=0.10, wz=0.14 gives an inverse-kinematic wheel bound below 5 rad/s. Measured joint maximum is separately reported; a command bound does not prove a physical feedback bound.\n\nOfficial runs request RTF=1; pilots and accelerated exploratory runs are excluded from comparison.csv. Command sim-time intervals and achieved RTF are included to reveal timing differences. Controllers run concurrently with isolated ROS domains/Gazebo partitions; CPU utilization is under concurrent load. Host CPU values cannot directly rank candidates executed on different hardware. Per-cycle controller compute time is measured on the desktop TEB/RPP runs; local DWB/MPPI timings are supplemental straight-route profiling, kept separate from the 20 official runs. ARM64 and x86_64 hosts differ, so absolute times cannot directly rank all four algorithms.\n\nSmoothness uses GT velocity in the message twist frame, 20 Hz simulation-time interpolation, centered 5-point moving average and two gradients; first/last 0.25 seconds discarded. These are body velocity change metrics, not calibrated passenger comfort or physical acceleration measurements. Stationary time includes intentional safety/yield stops; counts are descriptive.\n\nStatic-world contact and minimum true footprint clearance were not instrumented. Existing SafetyProbe clearance is for an injected fixture, which these five routes do not inject. Do not claim collision-free from action success. Gazebo evidence does not prove physical robot performance.\n\nA timed-out/aborted route is included as an outcome; arrival accuracy of such a route is not a successful-arrival score. Startup infrastructure retries are recorded separately. Humble RPP lacks the native translational acceleration limit used by the other candidates; its final speed/yaw are capped by the common guard. No shared velocity smoother was added, so jerk comparisons include that capability difference. TEB uses a recorded ROS 2 compatibility patch and polygon footprint configuration, not an unmodified default.

Five varied cases provide descriptive observations, not a statistically established success rate.\n'''
 (ROOT/'README.md').write_text(notes)
if __name__=='__main__':main()
