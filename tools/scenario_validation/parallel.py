"""Run isolated ROS/Gazebo stacks with a simultaneous mission start barrier."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time
from processes import signal_owned


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--count',type=int,required=True)
    parser.add_argument('--name',required=True)
    parser.add_argument('--seat',default='seat-43')
    parser.add_argument('--seats',help='Comma-separated seats, exactly count entries')
    parser.add_argument('--sequence')
    parser.add_argument('--scenario',default='normal')
    parser.add_argument('--repeats',type=int,default=1)
    parser.add_argument('--container',default='cleany-ros-humble-cdi')
    parser.add_argument('--navigation-overlay',type=Path,
                        help='Source a tested navigation overlay after the baseline runtime overlay')
    parser.add_argument('--runtime-overlay',type=Path,help='Complete runtime install for another host')
    parser.add_argument('--amcl-overlay',type=Path,help='Optional custom AMCL overlay; otherwise host/system AMCL')
    parser.add_argument('--domain-base',type=int,default=80)
    args=parser.parse_args()
    if not 1<=args.count<=20:parser.error('count must be 1..20')
    if args.repeats<1:parser.error('repeats must be positive')
    if not 0<=args.domain_base<=232-args.count:parser.error('ROS domain range is invalid')
    if args.amcl_overlay and not args.amcl_overlay.exists():parser.error('Explicit AMCL overlay is missing')
    if args.runtime_overlay and not args.runtime_overlay.exists():parser.error('Explicit runtime overlay is missing')
    mission_count=args.repeats*(len(args.sequence.split(',')) if args.sequence else 1)
    seats=args.seats.split(',') if args.seats else [args.seat]*args.count
    if len(seats)!=args.count:parser.error('seats length must equal count')
    root=args.root.resolve();out=root/'artifacts/scenarios'/args.name
    out.mkdir(parents=True,exist_ok=False)
    gate=out/'start';workers=[];logs=[];samples=[];started=time.monotonic()
    overlay=args.runtime_overlay or root.parent/'runtime-accuracy-review/ros2_ws/install/setup.bash'
    amcl=args.amcl_overlay or root.parent/'validation-deps/nav2-amcl-1.1.20/install/setup.bash'
    if not amcl.exists():amcl=None
    try:
        for i in range(args.count):
            output=out/f'worker-{i:02d}'
            cmd=['python3',str(root/'tools/scenario_validation/instance.py'),'--root',str(root),'--fixture',str(root/'artifacts/scenarios/fixture'),'--output',str(output),'--seat',seats[i],'--domain',str(args.domain_base+i),'--partition',f'cleany_{args.name}_{i}','--gate',str(gate),'--repeats',str(args.repeats),'--scenario',args.scenario]
            if args.sequence:cmd.extend(['--sequence',args.sequence])
            setups=[Path('/opt/ros/humble/setup.bash')]
            if amcl:setups.append(amcl)
            setups.append(overlay)
            if args.navigation_overlay:setups.append(args.navigation_overlay)
            shell=' && '.join('source '+shlex.quote(str(p)) for p in setups)+' && exec '+shlex.join(cmd)
            log=(out/f'worker-{i:02d}.log').open('w');logs.append(log)
            p=subprocess.Popen(['podman','exec',args.container,'bash','-c',shell],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            workers.append((p,output))
        released=None
        while any(p.poll() is None for p,_ in workers):
            now=time.monotonic()
            if released is None:
                if any(p.poll() is not None for p,_ in workers):
                    raise RuntimeError('Worker exited before barrier; preserve logs')
                if all((output/'ready.json').exists() for _,output in workers):
                    gate.write_text(str(now));released=now
                    print('START',args.name,'count',args.count,flush=True)
                elif now-started>330:raise TimeoutError('Readiness barrier')
            if now-started>330+mission_count*600:raise TimeoutError('Batch orchestration deadline')
            sample={'wall':now,'cpu':Path('/proc/stat').read_text().splitlines()[0],'memory':Path('/proc/meminfo').read_text()}
            if shutil.which('nvidia-smi'):
                gpu=subprocess.run(['nvidia-smi','--query-gpu=utilization.gpu,memory.used,power.draw','--format=csv,noheader,nounits'],text=True,capture_output=True,timeout=10)
                sample['gpu']=gpu.stdout.strip()
            else:sample['gpu']=None
            samples.append(sample)
            (out/'resources.json').write_text(json.dumps(samples))
            time.sleep(2)
        results=[json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {'error':'missing result'} for _,output in workers]
        report={'count':args.count,'seats':seats,'mission_start_wall':released,'repeats':args.repeats,'results':results,'elapsed_wall':time.monotonic()-started,'exit_codes':[p.returncode for p,_ in workers]}
        (out/'batch.json').write_text(json.dumps(report,indent=2))
        print('DONE',args.name,[(r.get('error'),[(m.get('seat'),(m.get('final',{}).get('last_result') or {}).get('outcome',m.get('reason','NO_TERMINAL')),round(m['measured_rtf'],3) if m.get('measured_rtf') is not None else None) for m in r.get('missions',[])]) for r in results],flush=True)
    finally:
        for p,output in workers:
            if p.poll() is None:
                # Kill only process groups whose ownership was recorded by this instance.
                observer=output/'observer.json'
                if observer.exists():
                    signal_owned(json.loads(observer.read_text())['pid'],output,b'instance.py')
                ready=output/'children.json'
                if ready.exists():
                    for _,pid in json.loads(ready.read_text()):
                        signal_owned(pid,output,b'ros2',group=True)
                try:p.wait(timeout=75)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGTERM)
        for log in logs:log.close()


if __name__=='__main__':main()
