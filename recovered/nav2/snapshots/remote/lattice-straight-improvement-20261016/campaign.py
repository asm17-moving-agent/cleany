from pathlib import Path
import argparse,json,subprocess,time
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--variant',choices=['baseline','improved','refined'],required=True);args=p.parse_args()
for i,name in [(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')]:
 out=ROOT/args.variant/f'{i}_{name}'
 if out.exists():raise RuntimeError('Preserve existing trial: '+str(out))
 out.parent.mkdir(exist_ok=True)
 with (out.parent/f'{i}_supervisor.log').open('w') as log:
  code=subprocess.run(['bash',str(ROOT/'run_one.sh'),'--planner','lattice','--variant',args.variant,'--goal',str(i),'--output',str(out)],stdout=log,stderr=subprocess.STDOUT).returncode
 r=json.loads((out/'result.json').read_text());print(json.dumps({'variant':args.variant,'goal':i,'status':r.get('action_status'),'error':r.get('error'),'sim_s':r.get('navigation_sim_s')}),flush=True)
 if not r.get('navigation_goal_sent'):raise RuntimeError(r.get('error'))
 subprocess.run(['python3','/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment/summarize_run.py',str(out)],check=True,stdout=subprocess.DEVNULL)
(ROOT/(args.variant+'-completed.json')).write_text(json.dumps({'variant':args.variant,'finished_wall':time.time()}))
