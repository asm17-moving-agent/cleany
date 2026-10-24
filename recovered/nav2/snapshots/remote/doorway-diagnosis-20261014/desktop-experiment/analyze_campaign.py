"""Flatten recorded outcomes without substituting Nav2 success for GT success."""
from pathlib import Path
import csv,json,argparse
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args()
rows=[]
for path in sorted(a.directory.glob('*/metrics.json')):
 m=json.loads(path.read_text());row={'run':path.parent.name}
 for key in ['goal_index','goal_name','action_status','nav2_success','gt_tolerance_success','success','position_error_m','yaw_error_deg','navigation_sim_s','navigation_wall_s','achieved_rtf','measured_joint_max_rad_s','error']:row[key]=m.get(key)
 for k,v in (m.get('max_command_wheel_rad_s') or {}).items():row['max_wheel'+k]=v
 for k,v in (m.get('smoothness') or {}).items():row[k]=v
 for k,v in m.get('compute_timing',{}).get('compute_us',{}).items():row['compute_'+k+'_us']=v
 for k,v in (m.get('command_sim_interval_s',{}).get('/cmd_vel') or {}).items():row['cmd_sim_interval_'+k+'_s']=v
 row['controller_deadline_warning_count']=m.get('controller_deadline_warning_count')
 for k,v in m.get('contacts',{}).items():
  if isinstance(v,(int,float,bool)):row['contact_'+k]=v
 rows.append(row)
(a.directory/'summary.json').write_text(json.dumps(rows,indent=2))
if rows:
 keys=list(dict.fromkeys(k for row in rows for k in row))
 with (a.directory/'summary.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
 print(json.dumps([{'run':r['run'],'nav':r['action_status'],'gt_ok':r['success'],'xy':r['position_error_m'],'yaw':r['yaw_error_deg']} for r in rows],indent=2))
