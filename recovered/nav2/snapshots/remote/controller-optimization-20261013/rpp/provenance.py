import hashlib,json,os,platform,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.parent/'rpp-source'
def command(args):
    p=subprocess.run(args,capture_output=True,text=True);return {'exit_code':p.returncode,'stdout':p.stdout.strip(),'stderr':p.stderr.strip()}
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
files=[SOURCE/'ros2_ws/src/cleany_navigation/src/curvature_limited_rpp.cpp',SOURCE/'ros2_ws/src/cleany_navigation/include/cleany_navigation/rpp_yaw_limit.hpp',ROOT/'source.patch']
files.extend(ROOT.glob('sol61_20261013_0620/*.py'));files.extend(ROOT.glob('relaxed_sol61_20261013/*.py'))
partitions={}
for session in ROOT.glob('**/session.json'):
    s=json.loads(session.read_text());partitions[s['partition']]=str(session)
residual=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        env=(p/'environ').read_bytes().split(b'\0');cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if not cmd:continue
        found=[part for part in partitions if ('IGN_PARTITION='+part).encode() in env or ('GZ_PARTITION='+part).encode() in env]
        if found:residual.append({'pid':int(p.name),'cmd':cmd,'partitions':found})
    except OSError:pass
cleanup={'checked_partitions':partitions,'residual_processes':residual,'all_new_experiment_processes_exited':not residual,'legacy_frictions_child_incident':'SIGINT to owned legacy launch77608 also terminated child friction Gazebo77630; recorded separately and acknowledged. No further non-experiment process termination.'}
(ROOT/'cleanup.json').write_text(json.dumps(cleanup,indent=2))
provenance={'client_date':'2026-10-13','host':os.uname().nodename,'platform':platform.platform(),'cpu':command(['lscpu']),'container':'cleany-ros-humble-cdi','source_head':command(['git','-C',str(SOURCE),'rev-parse','HEAD']),'source_status':command(['git','-C',str(SOURCE),'status','--short']),'source_diff_check':command(['git','-C',str(SOURCE),'diff','--check']),'permissions':'danger-full-access / approval never confirmed by active tool instructions','domain':228,'discovery':{'ROS_LOCALHOST_ONLY':'1','IGN_IP':'127.0.0.1','GZ_IP':'127.0.0.1','partition':'UUID per run'},'sha256':{str(p):sha(p) for p in files},'baseline_environment':str(ROOT.parents[1]/'controller-comparison-20261013/environment.json'),'ros_packages':command(['distrobox','enter','cleany-ros-humble-cdi','--','dpkg-query','-W','-f=${Package} ${Version}\n','ros-humble-nav2-regulated-pure-pursuit-controller','ros-humble-nav2-controller','libignition-gazebo6','ros-humble-ros-gz-bridge']),'container_os':command(['distrobox','enter','cleany-ros-humble-cdi','--','cat','/etc/os-release'])}
(ROOT/'provenance.json').write_text(json.dumps(provenance,indent=2))
print('residual',residual)
