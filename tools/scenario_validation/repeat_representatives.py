"""Repeat measured routes in fresh instances and one continuing Gazebo scene."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from after_smoke_recheck import require_completed_queue
from audit import assess
from survey import process_identity


def fresh_jobs(seats: list[str], runs: int, overlay: str) -> list[dict]:
    jobs=[]
    for repeat in range(1,runs+1):
        for offset in range(0,len(seats),3):
            group=seats[offset:offset+3]
            jobs.append({'count':len(group),'name':f'representative-v2-fresh-r{repeat:02d}-g{offset//3}',
                         'seats':','.join(group),'navigation_overlay':overlay})
    return jobs


def run_jobs(root: Path, name: str, jobs: list[dict]) -> dict:
    directory=root/'artifacts/scenarios';file=directory/(name+'.json')
    with file.open('x') as stream:json.dump(jobs,stream,indent=2)
    subprocess.run([sys.executable,str(root/'tools/scenario_validation/queue.py'),
                    '--root',str(root),'--jobs',str(file)],check=True)
    require_completed_queue(directory,jobs,name+'.status.json')
    verdicts={}
    for job in jobs:
        for index in range(job['count']):
            worker=directory/job['name']/f'worker-{index:02d}'
            verdicts[str(worker.relative_to(directory))]=assess(
                json.loads((worker/'result.json').read_text()),
                json.loads((worker/'shutdown.json').read_text()))
    (directory/(name+'.verdicts.json')).write_text(json.dumps(verdicts,indent=2))
    return verdicts


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--wait-pid',type=int,required=True)
    parser.add_argument('--runs',type=int,default=10)
    parser.add_argument('--container',default='cleany-ros-humble-cdi')
    args=parser.parse_args()
    if args.runs<1:parser.error('runs must be positive')
    root=args.root.resolve();directory=root/'artifacts/scenarios'
    identity=process_identity(args.wait_pid)
    if not identity or str(root/'tools/scenario_validation/after_smoke_recheck.py') not in identity:
        raise RuntimeError('Expected live navigation recheck driver is absent')
    print('WAITING FOR NAVIGATION RECHECK',args.wait_pid,flush=True)
    (directory/'representative_repeat_wait.json').write_text(json.dumps(
        {'pid':args.wait_pid,'identity':identity,'runs':args.runs},indent=2))
    while process_identity(args.wait_pid)==identity:time.sleep(10)
    corrections=directory/'navigation_recheck_jobs.json'
    require_completed_queue(directory,json.loads(corrections.read_text()),'navigation_recheck_jobs.status.json')
    subprocess.run(['podman','exec',args.container,'python3',
                    str(root/'tools/scenario_validation/representatives.py'),str(directory),
                    '--corrections',str(corrections)],check=True)
    selected=json.loads((directory/'representatives.json').read_text())['selected']
    seats=[r['seat'] for r in selected];overlay=str(root/'ros2_ws/install-navigation/setup.bash')
    smoke=[{'count':1,'name':'representative-v2-continuous-smoke','seat':seats[0],
            'sequence':','.join(seats[:2]),'navigation_overlay':overlay}]
    smoke_verdicts=run_jobs(root,'representative_continuous_smoke_jobs',smoke)
    run_jobs(root,'representative_fresh_jobs',fresh_jobs(seats,args.runs,overlay))
    if not all(v['passed'] for v in smoke_verdicts.values()):
        raise RuntimeError('Continuous chair/mission smoke failed; fresh repeats preserved, inspect before continuing')
    continuous=[{'count':1,'name':'representative-v2-continuous','seat':seats[0],
                 'sequence':','.join(seats),'repeats':args.runs,'navigation_overlay':overlay}]
    run_jobs(root,'representative_continuous_jobs',continuous)
    print('REPRESENTATIVE REPEATS TERMINAL; inspect raw outcomes and unresolved seats',flush=True)


if __name__=='__main__':main()
