from pathlib import Path
import subprocess,json,time,argparse
R=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--planner',choices=['lattice','theta'],required=True);a=p.parse_args()
goals=['straight','turn_arrival','chair_gap','far_heading','distant','lateral','narrow_seat','other_room']
for i,name in enumerate(goals):
 out=R/('dwb_'+a.planner)/(str(i)+'_'+name)
 if (out/'result.json').exists():
  r=json.loads((out/'result.json').read_text())
  if r.get('navigation_goal_sent') and r.get('completed'):continue
  raise RuntimeError('Preserve incomplete run for diagnosis: '+str(out))
 out.parent.mkdir(exist_ok=True)
 with (out.parent/(str(i)+'-supervisor.log')).open('w') as f:
  code=subprocess.run(['bash',str(R/'run_one.sh'),'--planner',a.planner,'--goal',str(i),'--output',str(out)],stdout=f,stderr=subprocess.STDOUT).returncode
 r=json.loads((out/'result.json').read_text());print(json.dumps({'planner':a.planner,'goal':name,'code':code,'status':r.get('action_status'),'sim_s':r.get('navigation_sim_s'),'error':r.get('error')}),flush=True)
 if not r.get('navigation_goal_sent'):raise RuntimeError('Preflight failure: '+str(r.get('error')))
 subprocess.run(['python3','/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment/summarize_run.py',str(out)],stdout=(out/'metrics-summary.log').open('w'),check=True)
(R/('completed-dwb_'+a.planner+'.json')).write_text(json.dumps({'planner':a.planner,'finished':time.time(),'goals':8}))
