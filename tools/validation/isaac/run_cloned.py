"""Run independent Mission stacks against Isaac Lab clones in one simulator."""
from __future__ import annotations
import argparse
from contextlib import ExitStack
from datetime import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from zoneinfo import ZoneInfo
from build import source_digest
from run import SOURCE, memory, write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-root', type=Path, required=True)
    parser.add_argument('--cache-root', type=Path, required=True)
    parser.add_argument('--isaaclab-root', type=Path, required=True)
    parser.add_argument('--isaaclab-deps', type=Path, required=True)
    parser.add_argument('--facility-map', type=Path, required=True)
    parser.add_argument('--targets', nargs='+', default=['seat-12','seat-03'])
    parser.add_argument('--domain', type=int, default=177)
    parser.add_argument('--physics-device', choices=['cpu', 'cuda:0'], default='cpu')
    parser.add_argument('--gpu-trace', action='store_true')
    parser.add_argument('--timeout', type=float, default=1800)
    args = parser.parse_args()
    if args.output.exists() or not 1 <= args.domain <= 232 or args.domain == 73:
        parser.error('Use a new output path and isolated domain')
    output,build,cache,lab = (p.resolve() for p in (args.output,args.build_root,args.cache_root,args.isaaclab_root))
    record = json.loads((build/'manifest.json').read_text())
    assert record['status']=='passed' and record['source_digest']==source_digest(SOURCE)
    output.mkdir(parents=True)
    for name in ('config','logs','evaluations','provenance'):(output/name).mkdir()
    cache.mkdir(parents=True,exist_ok=True)
    for name in ('driver','compute'):(cache/name).mkdir(exist_ok=True)
    snapshot=output/'provenance/source'
    shutil.copytree(Path(__file__).parent,snapshot,ignore=shutil.ignore_patterns('__pycache__'))
    lab_commit=subprocess.check_output(['git','-C',str(lab),'rev-parse','HEAD'],text=True).strip()
    write(output/'config/clones.json',{'num_envs':len(args.targets),'spacing_m':100.,'isaaclab_commit':lab_commit,'physics_device':args.physics_device,'gpu_trace':args.gpu_trace})
    ros_image=subprocess.check_output(['podman','image','inspect','--format','{{.Id}}','localhost/cleany-mission-isaac-ros:humble'],text=True).strip()
    isaac_image=subprocess.check_output(['podman','image','inspect','--format','{{.Id}}','localhost/cleany-isaac-visual:6.1.0'],text=True).strip()
    assert ros_image==record['image']
    report={'status':'running','started_at':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),'started_wall':time.time(),
            'mode':'Isaac Lab USD clones, one PhysX process, separate ROS namespaces',
            'physics_device':args.physics_device,'gpu_trace':args.gpu_trace,
            'domain':args.domain,'targets':args.targets,'source_digest':record['source_digest'],
            'isaac_image':isaac_image,'ros_image':ros_image,'isaaclab_commit':lab_commit,'commands':[],
            'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.glob('*.py')},'environments':[]}
    write(output/'provenance/run.json',report)
    containers=[f'cleany-e2e-{args.domain}-isaac', f'cleany-e2e-{args.domain}-bringup']+[f'cleany-e2e-{args.domain}-env_{i}-{role}' for i in range(len(args.targets)) for role in ('ros','validate')]
    processes=[];started=time.monotonic()
    with ExitStack() as stack:
        lock=stack.enter_context((build/f'.isaac-domain-{args.domain}.lock').open('a'));fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        lock=stack.enter_context((build/'.build.lock').open('a'));fcntl.flock(lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
        resources=stack.enter_context((output/'evaluations/resources.jsonl').open('w',buffering=1))
        def check():
            values=memory();resources.write(json.dumps({'wall_time':time.time(),'elapsed':time.monotonic()-started,**values})+'\n')
            if values['gpu_free_mib']<1024 or values['host_available_mib']<4096:raise RuntimeError('Memory headroom exhausted')
            if time.monotonic()-started>args.timeout:raise TimeoutError('Cloned attempt timed out')
        def start(command,path):
            report['commands'].append(command);write(output/'provenance/run.json',report)
            log=stack.enter_context(path.open('w'));p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);processes.append(p);return p
        env=['--network','host','-e',f'ROS_DOMAIN_ID={args.domain}','-e','ROS_LOCALHOST_ONLY=0','-e','RMW_IMPLEMENTATION=rmw_fastrtps_cpp','-e','FASTRTPS_DEFAULT_PROFILES_FILE=/output/config/fastdds.xml']
        base=['podman','run','--rm','--entrypoint','bash']
        common=['-v',f'{SOURCE}:/main:ro','-v',f'{snapshot}:/integration:ro']
        setup='source /opt/ros/humble/setup.bash && source /build/install/setup.bash && export PYTHONPATH=/integration:$PYTHONPATH && '
        try:
            headroom=memory();assert headroom['gpu_free_mib']>=2048 and headroom['host_available_mib']>=10000,headroom
            shutil.copyfile(SOURCE/'containers/isaac/fastdds.xml',output/'config/fastdds.xml')
            start([sys.executable,str(snapshot/'profile_system.py'),'--output',str(output),'--domain',str(args.domain),'--cloned-envs',str(len(args.targets)),'--timeout',str(args.timeout)],output/'logs/profile-system.log')
            for i,target in enumerate(args.targets):
                directory=output/f'env_{i}'
                for name in ('assets','config','logs','evaluations','provenance'):(directory/name).mkdir(parents=True,exist_ok=True)
                shutil.copyfile(SOURCE/'containers/isaac/fastdds.xml',directory/'config/fastdds.xml')
                shutil.copyfile(Path.home()/'.ignition/fuel/fuel.gazebosim.org/openrobotics/models/officechairgrey/1/meshes/OfficeChairGrey.obj',directory/'assets/OfficeChairGrey.obj')
                shutil.copyfile(args.facility_map,directory/'assets/facility_integration.yaml');shutil.copyfile(args.facility_map.with_suffix('.pgm'),directory/'assets/facility_integration.pgm')
                write(directory/'provenance/build.json',record)
                proc=start(base+common+['-v',f'{directory}:/output:rw',ros_image,'-c','source /opt/ros/humble/setup.bash && python3 /integration/prepare.py --world facility18 --target-id '+target],directory/'logs/prepare.log')
                assert proc.wait(timeout=90)==0,'Preparation failed'
                report['environments'].append({'index':i,'target_id':target,'namespace':f'env_{i}','run':str(directory)})
            isaac=start(base+['--name',containers[0],'--user','0','--device','nvidia.com/gpu=all','--security-opt','label=disable','--shm-size=2g']+env+common+['-v',f'{output}:/output:rw','-v',f'{lab}:/isaaclab:ro','-v',f'{args.isaaclab_deps.resolve()}:/labdeps:ro','-v',f'{cache}:/root/.cache/ov:rw','-v',f'{cache}/driver:/isaac-sim/kit/cache/nv_shadercache:rw','-v',f'{cache}/compute:/root/.nv/ComputeCache:rw','-e','ACCEPT_EULA=Y','-e','PRIVACY_CONSENT=N','-e','OMNI_KIT_ALLOW_ROOT=1','-e','ROS_DISTRO=humble','-e','PYTHONUNBUFFERED=1','-e','LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.core/humble/lib','-e','OMP_NUM_THREADS=4',isaac_image,'-c','cd /isaac-sim && export PYTHONPATH=/labdeps:/isaaclab/source/isaaclab:/integration:$PYTHONPATH && ./python.sh /integration/cloned_scene.py'],output/'logs/isaac.log')
            while not all((output/f'env_{i}/evaluations/adapter-ready.json').exists() for i in range(len(args.targets))):
                check();assert isaac.poll() is None,'Isaac exited before clone readiness';time.sleep(2)
            graph=start(base+env+common+['-v',f'{output}:/output:rw',ros_image,'-c','source /opt/ros/humble/setup.bash && python3 /integration/inspect_clone_graph.py'],output/'logs/ros-graph.log')
            assert graph.wait(timeout=20)==0, 'ROS graph inspection failed'
            def ros_command(i,role,command):
                return base+['--name',f'cleany-e2e-{args.domain}-env_{i}-{role}']+env+common+['-v',f'{output/f"env_{i}"}:/output:rw','-v',f'{build}:/build:ro','-e',f'CLEANY_ROS_PREFIX=env_{i}','-e','CLEANY_CONTROLLED_BRINGUP=1',ros_image,'-c',setup+command]
            arms=[start(ros_command(i,'validate','python3 /integration/validate_arm.py'),output/f'env_{i}/logs/arm.log') for i in range(len(args.targets))]
            while any(p.poll() is None for p in arms):check();assert isaac.poll() is None;time.sleep(2)
            assert all(p.returncode==0 for p in arms),'Arm validation failed'
            validators=[];ros=[]
            for i in range(len(args.targets)):
                ros.append(start(ros_command(i,'ros','ros2 launch /integration/cloned_mission.launch.py scenario:=success'),output/f'env_{i}/logs/mission.log'))
            bringup=start(base+['--name',f'cleany-e2e-{args.domain}-bringup']+env+common+[
                '-v',f'{output}:/output:rw','-v',f'{build}:/build:ro',ros_image,'-c',
                setup+'python3 /integration/bringup_clones.py'],output/'logs/bringup.log')
            while bringup.poll() is None:
                check();assert isaac.poll() is None and all(p.poll() is None for p in ros),'Simulator/ROS exited during bringup';time.sleep(2)
            assert bringup.returncode==0, 'Controlled lifecycle bringup failed'
            for i in range(len(args.targets)):
                validators.append(start(ros_command(i,'validate','python3 /integration/validate_mission.py --scenario success --require-no-recovery'),output/f'env_{i}/logs/validate.log'))
            while any(p.poll() is None for p in validators):
                check();assert isaac.poll() is None and all(p.poll() is None for p in ros),'Simulator/ROS exited'
                for i,p in enumerate(validators):
                    if p.poll() is not None:(output/f'env_{i}/stop').touch()
                time.sleep(2)
            report['validator_returncodes']=[p.returncode for p in validators]
            assert all(p.returncode==0 for p in validators),'Mission validation failed'
            report['status']='passed'
        except BaseException as error:
            report.update(status='failed',error=repr(error));raise
        finally:
            (output/'stop').touch()
            # Allow the simulator to flush shared tick and publication evidence.
            for name in reversed(containers):
                subprocess.run(['podman','stop','--ignore','--time','3',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=15)
            for p in processes:
                try:p.wait(timeout=15)
                except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
            report.update(elapsed_s=time.monotonic()-started,finished_wall=time.time())
            write(output/'provenance/run.json',report)
            for e in report['environments']:
                directory=Path(e['run']);mission=directory/'evaluations/mission.json'
                passed=mission.exists() and json.loads(mission.read_text())['passed']
                write(directory/'provenance/run.json',{**report,'status':'passed' if passed else 'failed','world':'facility18','target_id':e['target_id'],'shared_simulator':True})
            print(report['status'],output,flush=True)


if __name__=='__main__':main()
