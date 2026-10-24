"""Reserve one long route early; the original workers refuse duplicate folders."""
from pathlib import Path
import argparse
import json
import subprocess
import time

ROOT=Path(__file__).resolve().parent
BASE=Path('/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment')
parser=argparse.ArgumentParser();parser.add_argument('--planner',choices=['lattice','theta'],required=True)
args=parser.parse_args()
out=ROOT/args.planner/'7_other_room'
if out.exists():raise RuntimeError(f'Route already reserved: {out}')
(ROOT/f'parallel-{args.planner}-reservation.json').write_text(json.dumps({'planner':args.planner,'goal_index':7,'started_wall':time.time(),'reason':'User requested additional parallel simulation workers'},indent=2))
with (ROOT/args.planner/'7_supervisor.log').open('w') as log:
    code=subprocess.run(['bash',str(ROOT/'run_one.sh'),'--planner',args.planner,'--goal','7','--output',str(out)],stdout=log,stderr=subprocess.STDOUT).returncode
result=json.loads((out/'result.json').read_text())
if not result.get('navigation_goal_sent'):raise RuntimeError(result.get('error'))
subprocess.run(['python3',str(BASE/'summarize_run.py'),str(out)],check=True,stdout=subprocess.DEVNULL)
(ROOT/f'parallel-{args.planner}-completed.json').write_text(json.dumps({'goal_index':7,'status':result.get('action_status'),'exit':code,'finished_wall':time.time()},indent=2))
print(json.dumps({'planner':args.planner,'goal':7,'status':result.get('action_status')}),flush=True)
