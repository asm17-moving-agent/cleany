"""Build the isolated navigation correction after baseline jobs shut down."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from after_survey import survey_coverage
from audit import assess
from survey import process_identity


def require_completed_queue(directory: Path, jobs: list[dict], status_name: str) -> None:
    records=json.loads((directory/status_name).read_text())
    if len(records)!=len(jobs) or any(
        r.get('job')!=job or r.get('exit_code')!=0 or 'end' not in r
        for r,job in zip(records,jobs)):
        raise RuntimeError('Baseline smoke queue is incomplete; inspect before starting another stack')
    for job in jobs:
        batch=directory/job['name']
        report=json.loads((batch/'batch.json').read_text())
        if len(report['exit_codes'])!=job['count'] or any(report['exit_codes']):
            raise RuntimeError('Baseline smoke observers did not all exit cleanly')
        for index in range(job['count']):
            shutdown=json.loads((batch/f'worker-{index:02d}'/'shutdown.json').read_text())
            if not shutdown or any(shutdown.values()):
                raise RuntimeError('Baseline smoke launch shutdown is incomplete')


def require_completed_smoke(directory: Path, jobs: list[dict]) -> None:
    require_completed_queue(directory,jobs,'scenario_smoke_jobs.status.json')


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--wait-pid',type=int,required=True)
    parser.add_argument('--container',default='cleany-ros-humble-cdi')
    args=parser.parse_args()
    root=args.root.resolve();directory=root/'artifacts/scenarios'
    identity=process_identity(args.wait_pid)
    if not identity or str(root/'tools/scenario_validation/after_survey.py') not in identity:
        raise RuntimeError('Expected live baseline smoke driver is absent')
    (directory/'navigation_recheck_wait.json').write_text(json.dumps(
        {'pid':args.wait_pid,'identity':identity},indent=2))
    print('WAITING FOR BASELINE SMOKE',args.wait_pid,flush=True)
    while process_identity(args.wait_pid)==identity:time.sleep(10)
    coverage=survey_coverage(directory)
    smoke=json.loads((root/'tools/scenario_validation/scenario_smoke_jobs.json').read_text())
    require_completed_smoke(directory,smoke)
    # Build only after the reference runs have ended, so compiler CPU cannot
    # contaminate their capacity or mission timing. The baseline install stays intact.
    build_log=directory/'navigation_recheck_build.log'
    with build_log.open('x') as stream:
        subprocess.run(['podman','exec','-e','CMAKE_BUILD_PARALLEL_LEVEL=2',args.container,
                        'make','-C',str(root),'test-navigation'],stdout=stream,
                       stderr=subprocess.STDOUT,check=True)
    overlay=str(root/'ros2_ws/install-navigation/setup.bash')
    jobs=[{'count':1,'name':'navigation-v2-single-seat01','seat':'seat-01',
           'navigation_overlay':overlay}]
    failed=coverage['failed_seats']
    for index in range(0,len(failed),3):
        seats=failed[index:index+3]
        jobs.append({'count':len(seats),'name':f'navigation-v2-failed-{index//3:02d}',
                     'seats':','.join(seats),'navigation_overlay':overlay})
    jobs.append({'count':2,'name':'navigation-v2-reference-1343',
                 'seats':'seat-13,seat-43','navigation_overlay':overlay})
    file=directory/'navigation_recheck_jobs.json'
    with file.open('x') as stream:json.dump(jobs,stream,indent=2)
    subprocess.run([sys.executable,str(root/'tools/scenario_validation/queue.py'),
                    '--root',str(root),'--jobs',str(file)],check=True)
    verdicts={}
    for job in jobs:
        for worker in sorted((directory/job['name']).glob('worker-*')):
            if not worker.is_dir():continue
            verdicts[str(worker.relative_to(directory))]=assess(
                json.loads((worker/'result.json').read_text()),
                json.loads((worker/'shutdown.json').read_text()))
    (directory/'navigation_recheck_verdicts.json').write_text(json.dumps(verdicts,indent=2))
    print('NAVIGATION RECHECK',[(name,v['passed']) for name,v in verdicts.items()],flush=True)


if __name__=='__main__':main()
