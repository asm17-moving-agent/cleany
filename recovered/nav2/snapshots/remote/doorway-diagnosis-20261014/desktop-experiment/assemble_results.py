"""One inventory of normal, development, and fault-injection runs."""
from pathlib import Path
import json,csv
R=Path(__file__).resolve().parent;rows=[];records=[]
for p in sorted(R.rglob('metrics.json')):
 m=json.loads(p.read_text());name=str(p.parent.relative_to(R));phase='common_8_once_initial' if name.startswith('regression8/') else ('fault_injection' if name=='fault_recovery' else 'baseline_or_targeted_diagnostic')
 record={'run':name,'phase':phase,**m};records.append(record)
 row={k:record.get(k) for k in ['run','phase','goal_index','goal_name','action_status','nav2_success','gt_tolerance_success','success','position_error_m','yaw_error_deg','navigation_sim_s','navigation_wall_s','achieved_rtf','measured_joint_max_rad_s','controller_deadline_warning_count','error']}
 row['command_wheel_max_rad_s']=(m.get('max_command_wheel_rad_s') or {}).get('/cmd_vel')
 for k,v in (m.get('smoothness') or {}).items():row[k]=v
 for k,v in m.get('compute_timing',{}).get('compute_us',{}).items():row['compute_'+k+'_us']=v
 for k,v in (m.get('command_sim_interval_s',{}).get('/cmd_vel') or {}).items():row['command_sim_interval_'+k+'_s']=v
 row['non_floor_non_self_contacts']=m['contacts']['non_floor_non_self_records'];rows.append(row)
(R/'RESULTS.json').write_text(json.dumps({'common_campaign_runs':8,'additional_five_repeat_batch':False,'success_definition':'Nav2 status4 AND measured GT XY<=0.30m AND abs yaw<=10deg; contacts and measured wheel dynamics separately reported','runs':records},ensure_ascii=False,indent=2))
keys=list(dict.fromkeys(k for row in rows for k in row))
with (R/'RESULTS.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
print({'total_recorded_runs':len(rows),'common_campaign':sum(r['phase']=='common_8_once_initial' for r in rows),'final_m1_success':next(r['success'] for r in rows if r['run']=='portal_hold_m1')})
