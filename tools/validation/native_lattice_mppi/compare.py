"""Run the isolated ideal-kinematics fixture; never publish robot commands."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import numpy as np

p = argparse.ArgumentParser()
p.add_argument('binary', type=Path)
p.add_argument('artifacts', type=Path)
p.add_argument('configuration')
p.add_argument('cases', nargs='+')
p.add_argument('--preview-goal', action='store_true', help='Use the preview endpoint as the displayed goal')
a = p.parse_args()
results = []
for name in a.cases:
    fixture = a.artifacts/name
    run = a.artifacts/(a.configuration+'-'+name)
    run.mkdir(exist_ok=True)
    for item in ('map.txt', 'map.bin', 'poses.txt'):
        shutil.copy2(fixture/item, run/item)
    try:
        proc = subprocess.run([str(a.binary), str(run), '1', '--ros-args', '--params-file',
            str(a.artifacts/(a.configuration+'.yaml'))], env=dict(os.environ, ROS_DOMAIN_ID='188',
                **({'SNAP_PREVIEW_GOAL':'1'} if a.preview_goal else {})),
            capture_output=True, text=True, timeout=150)
    except subprocess.TimeoutExpired as exc:
        results.append({'case': name, 'timeout': True})
        continue
    (run/'output.log').write_text(proc.stdout+proc.stderr)
    rows=[]
    for line in proc.stdout.splitlines():
        if line and line[0].isdigit() and line.count(',')==6:
            try: rows.append(list(map(float,line.split(','))))
            except ValueError: pass
    data=np.array(rows)
    result={'case':name,'returncode':proc.returncode,
        'result':next((x for x in proc.stdout.splitlines() if x.startswith('RESULT ')),None),
        'goal_valid':next((x for x in proc.stdout.splitlines() if x.startswith('GOAL_VALID ')),None),
        'request_error':next((x for x in proc.stdout.splitlines() if x.startswith('REQUEST_ERROR ')),None),
        'goal_error':next((x for x in proc.stdout.splitlines() if x.startswith('GOAL_ERROR ')),None),
        'planning':next((x for x in proc.stdout.splitlines() if x.startswith('PLANNING ')),None)}
    if len(data)>1:
        vx,vy,wz=data[:,4],data[:,5],data[:,6]
        wheels=np.stack([vx-vy-.47*wz,vx+vy+.47*wz,vx+vy-.47*wz,vx-vy+.47*wz],axis=1)/.0635
        result.update(duration_s=len(data)*.05,max_wheel_rad_s=float(abs(wheels).max()),
            max_wheel_accel=float(abs(np.diff(np.vstack([np.zeros((1,4)),wheels]),axis=0)).max()/.05),
            total_rotation_rad=float(abs(wz).sum()*.05),
            endpoint=data[-1,1:4].tolist(),
            path_length_m=float(np.linalg.norm(np.diff(data[:,1:3],axis=0),axis=1).sum()))
        np.savetxt(run/'motion.csv',data,delimiter=',',header='step,x,y,yaw,vx,vy,wz',comments='')
    results.append(result)
    print(json.dumps(result),flush=True)
    (a.artifacts/(a.configuration+'-results.json')).write_text(json.dumps(results,indent=2))
