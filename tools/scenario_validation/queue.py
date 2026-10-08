"""Execute a reviewable serial list of parallel batches, preserving failures."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True);parser.add_argument('--jobs',type=Path,required=True);args=parser.parse_args()
    jobs=json.loads(args.jobs.read_text());records=[]
    status=args.jobs.with_suffix('.status.json')
    for job in jobs:
        name=job['name']
        if (args.root/'artifacts/scenarios'/name).exists():
            raise RuntimeError(f'Refusing to overwrite or restart existing job: {name}')
        cmd=[sys.executable,str(Path(__file__).parent/'parallel.py'),'--root',str(args.root)]
        for key,value in job.items():cmd.extend(['--'+key.replace('_','-'),str(value)])
        record={'job':job,'command':cmd,'start':time.time()};records.append(record)
        status.write_text(json.dumps(records,indent=2))
        print('JOB',name,flush=True)
        result=subprocess.run(cmd)
        record.update(end=time.time(),exit_code=result.returncode)
        status.write_text(json.dumps(records,indent=2))
        if result.returncode:
            # Do not conceal infrastructure errors or launch another stack over leftovers.
            raise RuntimeError(f'Infrastructure failure in {name}; inspect before continuing')


if __name__=='__main__':main()
