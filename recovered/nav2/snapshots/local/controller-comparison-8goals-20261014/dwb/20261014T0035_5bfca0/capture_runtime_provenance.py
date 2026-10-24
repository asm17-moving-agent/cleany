import hashlib,json,os,sys,time
from pathlib import Path
from ament_index_python.packages import get_package_prefix,get_package_share_directory
r=Path(__file__).resolve().parent;session=json.loads((r/'campaign/2_chair_gap/session.json').read_text());partition=session['partition'];owned=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  env=(p/'environ').read_bytes().split(b'\0')
  if ('IGN_PARTITION='+partition).encode() not in env:continue
  cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace');record={'pid':int(p.name),'cmd':cmd}
  if 'controller_server' in cmd or 'planner_server' in cmd:
   libs=sorted(set(x.split()[-1] for x in (p/'maps').read_text().splitlines() if any(k in x for k in ['libaxis_generator.so','libtimed_controller.so','libstraight_first_planner.so','libobserved_footprint','libswept_footprint'])));record['loaded_libraries']={name:hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in libs};owned.append(record)
 except (OSError,ValueError):pass
packages={}
for package in ['cleany_gazebo_sim','cleany_navigation','cleany_benchmark','cleany_safety_guard','cleany_body_guard_3d']:
 try:
  prefix=Path(get_package_prefix(package));share=Path(get_package_share_directory(package));files={}
  candidates=list(share.rglob('*.yaml'))+list((share/'launch').glob('*.py'))+list(prefix.glob('lib/python*/site-packages/'+package+'/*.py'))
  for p in candidates:
   if p.is_file():files[str(p)]={'resolved_source_path':str(p.resolve()),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
  packages[package]={'prefix':str(prefix),'share':str(share),'files':files}
 except Exception as e:packages[package]={'unavailable_reason':str(e)}
out={'captured_wall_epoch_s':time.time(),'architecture':os.uname().machine,'python':sys.version,'ros_distro':os.environ.get('ROS_DISTRO'),'selected_env':{n:os.environ.get(n) for n in ['AMENT_PREFIX_PATH','PYTHONPATH','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_DISCOVERY_MSG_PORT','IGN_DISCOVERY_SRV_PORT']},'owned_loaded_plugins':owned,'installed_package_files':packages};(r/'runtime_environment_provenance.json').write_text(json.dumps(out,indent=2));print({'owned_plugins':owned,'packages':list(packages)})
