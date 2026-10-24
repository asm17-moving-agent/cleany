from pathlib import Path
import subprocess,json,time
R=Path(__file__).resolve().parent;started=time.monotonic()
while not (R/'xy_only/result.json').exists():
 if time.monotonic()-started>1800:raise TimeoutError('Baseline unfinished')
 time.sleep(2)
r=json.loads((R/'xy_only/result.json').read_text());assert r.get('navigation_goal_sent'),r.get('error')
with (R/'clearance-supervisor.log').open('w') as log:
 code=subprocess.run(['bash',str(R/'run_one.sh'),'--planner','theta','--goal','7','--variant','clearance','--output',str(R/'clearance')],stdout=log,stderr=subprocess.STDOUT).returncode
print('clearance exited',code,flush=True)
