"""Two isolated simulation workers; identical common controller, fresh world per goal."""
from pathlib import Path
import concurrent.futures
import hashlib
import json
import subprocess
import time
import argparse

ROOT = Path(__file__).resolve().parent
BASE = Path('/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment')
LOCK = json.loads((BASE.parents[1] / 'controller-comparison-8goals-20261014/common/goals.locked.json').read_text())

def worker(planner):
    folder = ROOT / planner
    folder.mkdir(exist_ok=True)
    for goal in LOCK['goals']:
        index = goal['index']
        out = folder / f"{index}_{goal['id']}"
        if out.exists():
            result_file=out/'result.json'
            if not result_file.exists():
                raise RuntimeError(f'Unfinished run requires inspection: {out}')
            previous=json.loads(result_file.read_text())
            if previous.get('navigation_goal_sent'):
                continue
            archived=folder/f"startup-failed-{out.name}-{int(time.time())}"
            out.rename(archived)
        (folder / 'active.json').write_text(json.dumps({'goal': goal, 'started_wall': time.time()}))
        with (folder / f'{index}_supervisor.log').open('w') as log:
            status = subprocess.run(['bash', str(ROOT / 'run_one.sh'), '--planner', planner,
                '--goal', str(index), '--output', str(out)], stdout=log, stderr=subprocess.STDOUT).returncode
        raw = json.loads((out / 'result.json').read_text()) if (out / 'result.json').exists() else {}
        if not raw.get('navigation_goal_sent'):
            raise RuntimeError(f"{planner}/{index} startup failed: {raw.get('error')}; exit {status}")
        subprocess.run(['python3', str(BASE / 'summarize_run.py'), str(out)], check=True, stdout=subprocess.DEVNULL)
        print(json.dumps({'planner': planner, 'goal': index, 'status': raw.get('action_status'),
            'xy_m': raw.get('position_error_m'), 'yaw_deg': raw.get('yaw_error_deg')}), flush=True)
    (folder / 'completed.json').write_text(json.dumps({'goal_count': 8, 'finished_wall': time.time()}))

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--only',choices=['lattice','theta'])
    options=parser.parse_args()
    failures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        futures = {executor.submit(worker, p): p for p in ([options.only] if options.only else ['lattice', 'theta'])}
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as exc:
                failures.append({'planner': futures[future], 'error': repr(exc)})
                print(json.dumps(failures[-1]), flush=True)
    (ROOT / ('campaign-finished'+('-'+options.only if options.only else '')+'.json')).write_text(json.dumps({'failures': failures, 'finished_wall': time.time()}, indent=2))
    raise SystemExit(1 if failures else 0)
