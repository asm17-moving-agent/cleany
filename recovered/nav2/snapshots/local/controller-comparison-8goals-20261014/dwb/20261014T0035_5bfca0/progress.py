from pathlib import Path
import csv,json,time
r=Path(__file__).resolve().parent;campaign=r/'campaign';finished=[];active=[]
for p in sorted(campaign.iterdir()):
 if not p.is_dir() or not (p/'scenario.json').exists():continue
 if (p/'result.json').exists():
  result=json.loads((p/'result.json').read_text());finished.append({k:result.get(k) for k in ['goal_index','goal_name','completed','action_status','timeout','position_error_m','yaw_error_deg','navigation_sim_s','navigation_wall_s']})
 else:
  events=json.loads((p/'events.json').read_text()) if (p/'events.json').exists() else [];start=next((x['sim_s'] for x in events if x['phase']=='comparison_navigation'),None);q={'goal':p.name,'navigation_started':start is not None,'navigation_sim_start_s':start}
  if start is not None and (p/'compute_timing.csv').exists():
   with (p/'compute_timing.csv').open() as f:
    records=list(csv.DictReader(f));records=[x for x in records if x.get('sim_time_ns') and float(x['sim_time_ns'])*1e-9>=start and x.get('wall_start_ns')]
   if records:
    q.update(last_compute_sim_s=float(records[-1]['sim_time_ns'])*1e-9,approx_navigation_wall_s=time.monotonic()-float(records[0]['wall_start_ns'])*1e-9,wall_limit_s=1800,sim_limit_s=1200,compute_exceptions_to_now=sum(x['success']=='0' for x in records));q['approx_remaining_wall_s']=max(0,1800-q['approx_navigation_wall_s'])
  if start is not None and events and events[-1]['phase']=='comparison_navigation':
   elapsed=time.time()-(p/'events.json').stat().st_mtime;q['approx_navigation_wall_s']=elapsed;q['approx_remaining_wall_s']=max(0,1800-elapsed);q['wall_estimate_source']='navigation event file write time; approximate, includes initial planner wait; no live database reads'
  active.append(q)
x={'updated_epoch_s':time.time(),'completed':finished,'active':active,'task_complete':len(finished)==8 and not active};(r/'STATUS.json').write_text(json.dumps(x,indent=2));print(json.dumps({'completed_count':len(finished),'active':active,'task_complete':x['task_complete']}))
