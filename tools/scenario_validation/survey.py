"""Run the whole configured seat inventory after the live capacity queue ends."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import statistics
from audit import assess


def process_identity(pid: int) -> str | None:
    try:
        stat=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        command=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0',b' ').decode()
    except FileNotFoundError:
        return None
    return stat[19]+' '+command


def choose_count(directory: Path) -> tuple[int,list[dict]]:
    candidates=[]
    for count in (1,2,3):
        rates=[];trials=[]
        for batch in sorted(directory.glob(f'capacity-{count}-long-r*')):
            if not (batch/'batch.json').exists():continue
            report=json.loads((batch/'batch.json').read_text())
            passes=[]
            for worker in sorted(batch.glob('worker-*')):
                if not worker.is_dir():continue
                result=json.loads((worker/'result.json').read_text())
                shutdown=json.loads((worker/'shutdown.json').read_text())
                passes.append(assess(result,shutdown)['passed'])
            good=len(passes)==count and all(passes) and all(code==0 for code in report['exit_codes'])
            trials.append({'batch':batch.name,'passed':good,'elapsed_wall':report['elapsed_wall']})
            if good:rates.append(count*3600/report['elapsed_wall'])
        if rates and all(trial['passed'] for trial in trials):
            candidates.append({'count':count,'missions_per_hour':statistics.median(rates),
                               'aggregation':'median of all completed reference trials','trials':trials})
    if not candidates:raise RuntimeError('No passing capacity configuration')
    return max(candidates,key=lambda c:c['missions_per_hour'])['count'],candidates


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--wait-pid',type=int,required=True);args=parser.parse_args()
    identity=process_identity(args.wait_pid)
    expected=str(args.root/'tools/scenario_validation/queue.py')
    if not identity or expected not in identity or 'capacity_jobs.json' not in identity:
        raise RuntimeError('Expected live capacity queue is missing; inspect its terminal state first')
    directory=args.root/'artifacts/scenarios'
    (directory/'survey_wait.json').write_text(json.dumps({'pid':args.wait_pid,'identity':identity},indent=2))
    print('WAITING FOR CAPACITY',args.wait_pid,flush=True)
    while process_identity(args.wait_pid)==identity:time.sleep(10)
    # Process termination alone does not prove the preceding jobs completed.
    status=args.root/'tools/scenario_validation/capacity_jobs.status.json'
    records=json.loads(status.read_text())
    jobs=json.loads((args.root/'tools/scenario_validation/capacity_jobs.json').read_text())
    if len(records)!=len(jobs) or any(r.get('exit_code')!=0 or 'end' not in r for r in records):
        raise RuntimeError('Capacity queue did not finish; preserve its errors before scheduling survey')
    extra=[{'count':count,'name':f'capacity-{count}-long-r{repeat}','seat':'seat-13'}
           for repeat in (2,3) for count in (2,3)]
    extras=directory/'throughput_jobs.json';extras.write_text(json.dumps(extra,indent=2))
    subprocess.run([sys.executable,str(Path(__file__).parent/'queue.py'),'--root',str(args.root),'--jobs',str(extras)],check=True)
    count,candidates=choose_count(directory)
    inventory=json.loads((directory/'fixture/seat_inventory.json').read_text())
    seats=inventory['seats']
    if len(seats)!=inventory['seat_count'] or len(seats)!=len(set(seats)):
        raise RuntimeError('Invalid seat inventory')
    survey=[]
    for i in range(0,len(seats),count):
        group=seats[i:i+count]
        survey.append({'count':len(group),'name':f'survey-{i//count:02d}-r1','seats':','.join(group)})
    file=directory/'survey_jobs.json';file.write_text(json.dumps(survey,indent=2))
    (directory/'survey_selection.json').write_text(json.dumps({'count':count,'candidates':candidates,
      'metric':'completed reference round trips per hour including startup and shutdown',
      'seat_count':len(seats),'jobs':survey},indent=2))
    print('SURVEY',len(seats),'seats, count',count,flush=True)
    subprocess.run([sys.executable,str(Path(__file__).parent/'queue.py'),'--root',str(args.root),'--jobs',str(file)],check=True)
    subprocess.run([sys.executable,str(Path(__file__).parent/'audit.py'),str(directory)],check=True)


if __name__=='__main__':main()
