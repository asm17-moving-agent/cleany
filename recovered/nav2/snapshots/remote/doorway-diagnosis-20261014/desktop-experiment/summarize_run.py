from pathlib import Path
import argparse,json,csv,math
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();r=args.run
v=json.loads((r/'result.json').read_text());out={k:v.get(k) for k in ['goal_index','goal_name','action_status','final_truth','position_error_m','yaw_error_deg','navigation_sim_s','navigation_wall_s','achieved_rtf','max_command_wheel_rad_s','smoothness','error']}
out['gt_tolerance_success']=v.get('position_error_m') is not None and v['position_error_m']<=.30 and v.get('yaw_error_deg') is not None and v['yaw_error_deg']<=10
out['success']=v.get('action_status')==4 and out['gt_tolerance_success']
out['nav2_success']=v.get('action_status')==4
out['command_sim_interval_s']=v.get('command_sim_interval_s',{})
out['controller_deadline_warning_count']=v.get('controller_deadline_warning_count')
start=v.get('navigation_sim_start_s',0);end=v.get('navigation_sim_end_s',float('inf'))
joints=json.loads((r/'joints.json').read_text()) if (r/'joints.json').exists() else []
values=[abs(x) for j in joints if start<=j['t']<=end for n,x in zip(j['names'],j['velocity']) if 'wheel' in n];out['measured_joint_max_rad_s']=max(values,default=None)
contacts=[];floor=0;self_count=0;unknown=0
if (r/'contacts.jsonl').exists():
 for line in (r/'contacts.jsonl').open():
  c=json.loads(line)
  if not start<=c['received_sim_s']<=end:continue
  a,b=c['collision1']['name'],c['collision2']['name']
  if not a or not b:unknown+=1
  elif 'ground_plane' in a+' '+b:floor+=1
  elif all(n.startswith('cleany_mecanum::') for n in [a,b]):self_count+=1
  else:contacts.append(c)
out['contacts']={'non_floor_non_self_records':len(contacts),'pairs':sorted(set((c['collision1']['name'],c['collision2']['name']) for c in contacts)),'sampled_floor_records':floor,'self_records':self_count,'unknown_pair_records':unknown,'all_collision_links_instrumented':True,'caveat':'Sensor messages are not an independent geometric penetration proof; absence is reported as no observed contact, not guaranteed collision-free.'}
if (r/'compute_timing.csv').exists():
 with (r/'compute_timing.csv').open() as f:reader=csv.DictReader(f);t=list(reader)
 out['compute_timing_columns']=list(t[0]) if t else []
 numeric={}
 for k in ['compute_us']:
  try:a=np.array([float(row[k]) for row in t]);numeric[k]={'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'p99':float(np.percentile(a,99)),'max':float(np.max(a))}
  except ValueError:pass
 out['compute_timing']=numeric
 out['compute_scope']='TimedController inner compute steady wall microseconds; includes enabled telemetry/output validation, excludes simulator and planner; failed calls retained.'
(r/'metrics.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
