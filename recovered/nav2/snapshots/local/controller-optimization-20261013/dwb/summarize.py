#!/usr/bin/env python3
import csv,json,math,sys
from pathlib import Path
import numpy as np

def summary(p):
 r=json.loads((p/'result.json').read_text())
 if not r.get('completed'):return r
 a=np.genfromtxt(p/'compute_timing.csv',delimiter=',',names=True)
 us=np.atleast_1d(a['compute_us'])/1000
 r['compute_ms']={'median':float(np.median(us)),'p95':float(np.percentile(us,95)),'p99':float(np.percentile(us,99)),'max':float(np.max(us)),'exception_count':int(np.sum(a['success']==0)),'over_50ms':int(np.sum(us>50)),'count':len(us)}
 joints=json.loads((p/'joints.json').read_text());wheel=[abs(v) for j in joints for n,v in zip(j['names'],j['velocity']) if 'wheel' in n]
 r['measured_wheel_max_rad_s']=max(wheel) if wheel else None
 stages={}
 for row in csv.reader((p/'commands.csv').open()):
  t=row[2];v=[abs(float(i)) for i in row[3:]];stages[t]=[max(x,y) for x,y in zip(stages.get(t,[0,0,0]),v)]
 r['stage_abs_max_vx_vy_wz']=stages
 r['common_velocity_box_observed']={t:all(v<=lim+1e-8 for v,lim in zip(vals,[.15,.10,.14])) for t,vals in stages.items()}
 rows=list(csv.DictReader((p/'samples.csv').open()));z=rows[-1];est=[float(z['tf_x']),float(z['tf_y'])];gt=[float(z['gt_x']),float(z['gt_y'])]
 r['estimated_final_xy']=est;r['estimated_xy_error_m']=math.dist(est,r['goal_map'][:2]);r['GT_estimate_delta_m']=math.dist(gt,est)
 poses=json.loads((p/'poses.json').read_text());r['last_AMCL_pose']=next((x for x in reversed(poses) if x['source']=='amcl'),None)
 raw=json.loads((p/'resources.json').read_text());by={}
 for sample in raw:
  for proc in sample['processes']:
   if 'controller_server' not in proc['cmd']:continue
   by.setdefault(proc['pid'],[]).append((sample['wall_s'],proc['cpu_s'],proc['rss_bytes']))
 r['controller_process_resources']=[{'pid':pid,'cpu_percent_one_core':100*(v[-1][1]-v[0][1])/(v[-1][0]-v[0][0]) if len(v)>1 else None,'RSS_max_MiB':max(x[2] for x in v)/2**20} for pid,v in by.items()]
 diag=json.loads((p/'guard_diagnostics.json').read_text());reason={}
 for x in diag:
  d=json.loads(x['data']);k=d.get('reason','unknown');reason[k]=reason.get(k,0)+1
 r['guard_reason_sample_counts']=reason
 trace=p/'controller_command_dt.json'
 if trace.exists():
  tr=json.loads(trace.read_text())['records'];r['trajectory_count_median_p95']=list(np.percentile([x['trajectories'] for x in tr],[50,95]))
 return r
if __name__=='__main__':
 root=Path(sys.argv[1]);out=[]
 for result in sorted(root.rglob('result.json')):
  if 'bag' in result.parts:continue
  r=summary(result.parent);r['run_directory']=str(result.parent);out.append(r)
 (root/'metrics.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
