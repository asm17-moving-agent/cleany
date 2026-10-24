"""Cross-check BackUp logs against emitted commands and actual GT motion."""
from pathlib import Path
import argparse,csv,json,math,re
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=a.run
commands=[]
with (r/'commands.csv').open() as f:
 for row in csv.reader(f):
  if row[2]=='/cmd_vel' and abs(float(row[3])+.1)<1e-5 and abs(float(row[4]))<1e-5 and abs(float(row[5]))<1e-5:commands.append(float(row[0]))
groups=[]
for t in commands:
 if not groups or t-groups[-1][-1]>.3:groups.append([])
 groups[-1].append(t)
with (r/'samples.csv').open() as f:truth=[dict(sim=float(v['sim_s']),pose=[float(v[k]) for k in ['gt_x','gt_y','gt_yaw']]) for v in csv.DictReader(f)]
rows=[]
for g in groups:
 if g[-1]-g[0]<.2:continue
 start=min(truth,key=lambda v:abs(v['sim']-g[0]));end=min(truth,key=lambda v:abs(v['sim']-(g[-1]+.1)))
 rows.append({'command_start_sim_s':g[0],'command_end_sim_s':g[-1],'command_duration_s':g[-1]-g[0],'gt_start':start,'gt_end':end,'gt_translation_m':math.dist(start['pose'][:2],end['pose'][:2])})
log=(r/'launch.log').read_text(errors='replace')
out={'backup_command_matches':rows,'backup_completed_log_count':log.count('backup completed successfully'),'backup_collision_rejected_log_count':log.count('Collision Ahead - Exiting DriveOnHeading'),'route_timeout_log_count':log.count('Path progress timeout: 12.00s'),'recovery_log_lines':[line for line in log.splitlines() if any(t in line for t in ['Running backup','backup completed','Path progress timeout:','Collision Ahead - Exiting DriveOnHeading'])]}
(r/'recovery_evidence.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
