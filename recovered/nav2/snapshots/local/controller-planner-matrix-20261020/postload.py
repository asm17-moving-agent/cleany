from pathlib import Path
import subprocess,time,json,argparse,shutil
R=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--planner',choices=['lattice','theta'],required=True);a=p.parse_args()
while not (R/('completed-dwb_'+a.planner+'.json')).exists():time.sleep(10)
i,name=(1,'turn_arrival') if a.planner=='lattice' else (0,'straight')
folder=R/('dwb_'+a.planner)/f'{i}_{name}';pilot=R/'load-pilot'/f'dwb_{a.planner}-{i}_{name}-mixed-load'
r=json.loads((folder/'result.json').read_text());assert r['navigation_goal_sent'] and r['completed']
folder.rename(pilot);log=folder.parent/f'{i}-supervisor.log';shutil.copy2(log,pilot/'original-supervisor.log')
(pilot/'excluded-from-comparison.json').write_text(json.dumps({'reason':'Known exposure to4sim CPU overbooking withdeadline warnings. Declared exclusion before rerun, regardless of outcome. Fresh run after campaign with max2sims replaces performance row; original preserved','original_status':r['action_status']},indent=2))
with log.open('w') as f:code=subprocess.run(['bash',str(R/'run_one.sh'),'--planner',a.planner,'--goal',str(i),'--output',str(folder)],stdout=f,stderr=subprocess.STDOUT).returncode
r=json.loads((folder/'result.json').read_text());assert r['navigation_goal_sent'] and r['completed'],r
subprocess.run(['python3','/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment/summarize_run.py',str(folder)],stdout=(folder/'metrics-summary.log').open('w'),check=True)
(R/('postload-completed-dwb_'+a.planner+'.json')).write_text(json.dumps({'planner':a.planner,'rerun_goal':i,'original_preserved':str(pilot),'new_status':r['action_status'],'finished':time.time(),'returncode':code},indent=2))
