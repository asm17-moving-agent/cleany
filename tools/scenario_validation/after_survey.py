"""Confirm full seat coverage, then execute and judge every scenario smoke."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from audit import assess
from survey import process_identity


def survey_coverage(directory: Path) -> dict:
    inventory=json.loads((directory/'fixture/seat_inventory.json').read_text())
    jobs=json.loads((directory/'survey_jobs.json').read_text())
    records=json.loads((directory/'survey_jobs.status.json').read_text())
    if len(records)!=len(jobs) or any(r.get('exit_code')!=0 or 'end' not in r for r in records):
        raise RuntimeError('Survey queue did not reach a terminal result for every batch')
    seen={};failures=[]
    for job in jobs:
        for worker in sorted((directory/job['name']).glob('worker-*')):
            if not worker.is_dir():continue
            result=json.loads((worker/'result.json').read_text())
            seat=result['world_manifest']['seat']
            if seat in seen:raise RuntimeError(f'Duplicate survey seat: {seat}')
            shutdown=json.loads((worker/'shutdown.json').read_text())
            verdict=assess(result,shutdown)
            seen[seat]={'worker':str(worker),'verdict':verdict}
            if not verdict['passed']:failures.append(seat)
    if set(seen)!=set(inventory['seats']):
        raise RuntimeError('Survey does not cover the complete seat inventory')
    report={'seat_count':len(seen),'successful_seats':len(seen)-len(failures),'failed_seats':failures,'evidence':seen}
    (directory/'survey_coverage.json').write_text(json.dumps(report,indent=2))
    return report


def main() -> None:
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--wait-pid',type=int,required=True);args=p.parse_args()
    identity=process_identity(args.wait_pid)
    if not identity or str(args.root/'tools/scenario_validation/survey.py') not in identity:
        raise RuntimeError('Expected live survey driver is absent; inspect terminal state')
    directory=args.root/'artifacts/scenarios'
    print('WAITING FOR SURVEY',args.wait_pid,flush=True)
    while process_identity(args.wait_pid)==identity:time.sleep(10)
    coverage=survey_coverage(directory)
    print('SURVEY COVERAGE',coverage['seat_count'],'success',coverage['successful_seats'],'fail',coverage['failed_seats'],flush=True)
    smoke=args.root/'tools/scenario_validation/scenario_smoke_jobs.json'
    subprocess.run([sys.executable,str(Path(__file__).parent/'queue.py'),'--root',str(args.root),'--jobs',str(smoke)],check=True)
    verdicts={}
    for job in json.loads(smoke.read_text()):
        worker=directory/job['name']/'worker-00'
        verdicts[job['scenario']]=assess(json.loads((worker/'result.json').read_text()),json.loads((worker/'shutdown.json').read_text()))
    (directory/'scenario_smoke_verdicts.json').write_text(json.dumps(verdicts,indent=2))
    print('SCENARIO SMOKE',[(name,v['passed']) for name,v in verdicts.items()],flush=True)
    # Physical smoke evidence must be reviewed before scheduling repeated stimuli.


if __name__=='__main__':main()
