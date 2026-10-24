import hashlib
import json
import os
import platform
import subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
WORKSPACE=ROOT.parents[2]
OVERLAY=ROOT.parent/'sol61_20261013'
def command(args):
    return subprocess.check_output(args,text=True).strip()
def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
data={'host':platform.node(),'platform':platform.platform(),
      'client_date':'2026-10-13','client_timezone':'Asia/Seoul',
      'cpu':command(['lscpu']),'container':'cleany-ros-humble-cdi',
      'source_head':command(['git','-C',str(ROOT.parent/'source'),'rev-parse','HEAD']),
      'source_status':command(['git','-C',str(ROOT.parent/'source'),'status','--short']),
      'teb_head':'630a22e88dc9fd45be726a762edbb5b776bef231',
      'packages':command(['dpkg-query','-W','libignition-gazebo6','ros-humble-nav2-controller','ros-humble-nav2-regulated-pure-pursuit-controller']),
      'permission_context':{'sandbox_mode':'danger-full-access','approval_policy':'never'},
      'ros_domain_id':227,'localhost':True,'rtf_target':1,'physics_step_s':.002,
      'source_overlay':str(WORKSPACE/'controller-comparison-20261013/cleany-data/builds/source/install'),
      'teb_overlay':str(OVERLAY/'install'),
      'stage':'relaxed',
      'conservative_environment_reference':str(ROOT.parent/'resume_20261013_01/environment.json'),
      'actuator_layer':{'physical_wheel_cap_rad_s':5.,'requested_cap_rad_s':4.99,'wheel_acceleration_rad_s2':8.,'output_frequency_sim_hz':20,'gazebo_axis_acceleration_limits_removed':True},
      'simultaneous_other_owners':'Observed an existing /tmp/cleany-friction-frames-* Gazebo process; never signaled it. Variable host load is included in wall timing.',
      'patches':{p.name:digest(p) for p in ROOT.glob('*.patch')},
      'runner_sha256':digest(ROOT/'runner.py'),
      'scripts_sha256':{p.name:digest(p) for p in ROOT.glob('*.py')},
      'timing_source_sha256':digest(OVERLAY/'cleany_benchmark/src/timed_controller.cpp'),
      'compile_flags':{}}
for label,path in [('original',Path('/tmp/cleany-teb-humble')),('dedicated',OVERLAY)]:
    flags=path/'build/teb_local_planner/CMakeFiles/teb_local_planner.dir/flags.make'
    cache=path/'build/teb_local_planner/CMakeCache.txt'
    data['compile_flags'][label]={'flags':[s for s in flags.read_text().splitlines() if s.startswith('CXX_FLAGS')],
                                  'cache_build_type':[s for s in cache.read_text().splitlines() if s.startswith('CMAKE_BUILD_TYPE')],
                                  'source_forces_release':True}
    (ROOT/(label+'_compile_flags.txt')).write_text(flags.read_text())
(ROOT/'environment.json').write_text(json.dumps(data,indent=2))
print(json.dumps({k:data[k] for k in ('host','source_head','source_status','compile_flags')}))
