"""Read-only common fixture consumption; no goal edits or implicit chair removal."""
import copy,hashlib,json,math,shutil,xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parent
COMMON=ROOT.parents[1]/'common'
PRIOR=Path('/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/final/0_straight')

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def sensor_height(robot,sensor):
 def identity():return ([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],[0.,0.,0.])
 def compose(a,b):
  ar,at=a;br,bt=b
  rotation=[[sum(ar[i][k]*br[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
  translation=[at[i]+sum(ar[i][k]*bt[k] for k in range(3)) for i in range(3)]
  return rotation,translation
 def pose_matrix(element):
  p=element.find('pose');v=[float(x) for x in p.text.split()] if p is not None else [0.]*6
  x,y,z,r,pitch,yaw=v;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
  rotation=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
  return rotation,[x,y,z]
 frames={e.get('name'):e for e in robot if e.tag in ('frame','link')}
 def transform(name,visited=()):
  if name in ('__model__',''):return identity()
  if name in visited:raise ValueError('SDF frame cycle')
  e=frames[name];p=e.find('pose');parent=p.get('relative_to') if p is not None else None
  parent=parent or (e.get('attached_to','__model__') if e.tag=='frame' else '__model__')
  return compose(transform(parent,visited+(name,)),pose_matrix(e))
 p=sensor.find('pose');parent=p.get('relative_to') if p is not None else None
 if not parent:parent=next(link.get('name') for link in robot.findall('link') if sensor in list(link))
 return compose(compose(pose_matrix(robot),transform(parent)),pose_matrix(sensor))[1][2]
def common_path(value):
 p=Path(value)
 if p.is_absolute():
  # Never reuse another host's absolute path: resolve the transmitted suffix.
  parts=p.parts
  if 'common' in parts:p=Path(*parts[parts.index('common')+1:])
  else:raise ValueError('Non-portable common path '+str(p))
 p=(COMMON/p).resolve()
 if not p.is_relative_to(COMMON.resolve()):raise ValueError('Common path escape')
 if not p.is_file():raise FileNotFoundError(p)
 return p

def load_locked():
 if not (COMMON/'READY').is_file():raise RuntimeError('Common READY absent; campaign prohibited')
 p=COMMON/'goals.locked.json'
 data=json.loads(p.read_text())
 assert len(data['goals'])==8
 assert sorted(g['index'] for g in data['goals'])==list(range(8))
 return data

def choose_file(goal,kind):
 names=('world','world_file','world_sdf','canonical_world') if kind=='world' else ('map_yaml','map','matching_map','map_file')
 for key in names:
  value=goal.get(key)
  if isinstance(value,dict):value=value.get('path') or value.get('yaml')
  if value:return common_path(value)
 raise KeyError('Locked '+kind+' path absent; keys '+str(goal.keys()))

def prepare(out,goal,locked):
 import yaml
 shutil.copytree(PRIOR/'config',out/'config')
 shutil.copytree(PRIOR/'scripts',out/'scripts')
 shutil.copytree(COMMON/'fixture',out/'fixture')
 for name in ('wheel_odometry.yaml','sensor_tf.yaml'):
  shutil.copy2(COMMON/'fixture/config'/name,out/'config'/name)
 for name in ('wait_before_detour.xml','bt_manifest.json'):
  if (PRIOR/name).exists():shutil.copy2(PRIOR/name,out/name)
 source=choose_file(goal,'world')
 text=source.read_text()
 # Relocate the three texture paths embedded by the fixture's source host.
 # Preserve bytes from the pinned Cleany source; never strip a material.
 texture_relocations=[]
 for element in ET.fromstring(text).iter('albedo_map'):
  value=element.text or ''
  prefix='file:///home/ehdrms/workspace/cleany/feat-runtime-integration/ros2_ws/src/cleany_gazebo_sim/materials/'
  if value.startswith(prefix):
   relative=value[len(prefix):]
   if relative not in ('felt/albedo.png','roly/seat.png','roly/back.png'):raise ValueError('Unknown parent texture '+value)
   source_texture=Path('/home/donglee/workspace/cleany/controller-optimization-20261013/teb/source/ros2_ws/src/cleany_gazebo_sim/materials')/relative
   destination=out/'fixture/materials'/relative;destination.parent.mkdir(parents=True,exist_ok=True)
   shutil.copy2(source_texture,destination)
   replacement=destination.resolve().as_uri();text=text.replace(value,replacement)
   texture_relocations.append({'original':value,'materialized':replacement,'source':str(source_texture),'sha256':digest(destination),'source_commit':'b3ca816ba420452ee7960837bc16ec74148e07b0'})
 if '/home/ehdrms/' in text:raise ValueError('Unresolved parent local absolute path in canonical world')
 template=ET.fromstring(text)
 expected=[m.get('name') for m in template.find('world').findall('model')]
 removed=goal.get('remove_chair')
 if removed is not None and removed in expected:raise ValueError('Destination chair remains in canonical world')
 # Only replace URI roots. Canonical model names/collision geometry are unchanged.
 text=text.replace('fixture://',(out/'fixture').resolve().as_uri()+'/')
 if 'fixture://' in text:raise ValueError('Unmaterialized URI')
 (out/'world.sdf').write_text(text)
 (out/'world.canonical.sdf').write_text(source.read_text())
 refs=[]
 for uri in [e for e in ET.fromstring(text).iter() if e.tag in ('uri','albedo_map','normal_map')]:
  value=uri.text or ''
  if value.startswith('file://'):
   from urllib.parse import unquote,urlparse
   path=Path(unquote(urlparse(value).path))
   if not path.is_file():raise FileNotFoundError(path)
   refs.append({'uri':value,'sha256':digest(path)})
  else:raise ValueError('Non-local asset URI '+value)
 map_source=choose_file(goal,'map');map_yaml=yaml.safe_load(map_source.read_text())
 image=Path(map_yaml['image'])
 if image.is_absolute() and '/home/ehdrms/' in str(image):image=map_source.parent/image.name
 elif not image.is_absolute():image=map_source.parent/image
 if not image.is_file():raise FileNotFoundError(image)
 shutil.copy2(image,out/'map.pgm');map_yaml['image']='map.pgm'
 (out/'map.yaml').write_text(yaml.safe_dump(map_yaml))
 scenario=json.loads((PRIOR/'scenario.json').read_text())
 robot=template.find("world/model[@name='cleany_mecanum']")
 lidar=robot.find("link[@name='base_link']/sensor[@type='gpu_lidar']")
 scenario['spawn_pose']=[float(x) for x in robot.findtext('pose').split()]
 scenario['lidar_height_m']=sensor_height(robot,lidar) if lidar is not None else None
 sensor_tf=yaml.safe_load((out/'config/sensor_tf.yaml').read_text())['gazebo_sensor_tf_publisher']['ros__parameters']
 scenario['lidar_tf_nominal_height_m']=scenario['spawn_pose'][2]+sensor_tf['lidar_translation'][2]
 scenario['lidar_height_note']='SDF sensor relative_to frame chain resolved; canonical TF translation is recorded separately (4cm below actual SDF sensor mount). Both shared inputs retained.'
 scenario['lidar_noise']='canonical fixture SDF; not regenerated from prior-world profile'
 scenario.update(world=template.find('world').get('name'),initial_pose=locked['home_world_map'],initial_pose_covariance=locked['initial_pose_covariance'],map_frame='world aligned',map_source=str(map_source),map_sha256=digest(image),common_goal=goal,canonical_world_sha256=digest(source),locked_sha256=digest(COMMON/'goals.locked.json') if (COMMON/'goals.locked.json').exists() else None,expected_scene_models=expected,destination_removed_chair=removed)
 (out/'world.model.json').write_text((PRIOR/'world.model.json').read_text())
 (out/'asset_materialization.json').write_text(json.dumps({'source':str(source),'canonical_sha256':digest(source),'map_yaml_sha256':digest(map_source),'map_image_sha256':digest(image),'uri_occurrences':refs,'texture_relocations':texture_relocations,'all_local_assets_present':True,'canonical_models':expected,'removed_chair':removed},indent=2))
 (out/'goals.locked.json').write_text(json.dumps(locked,indent=2))
 return scenario
