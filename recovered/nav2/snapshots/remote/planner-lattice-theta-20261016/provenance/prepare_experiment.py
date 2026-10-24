"""Materialize immutable 18F inputs and the actual laptop effective configuration."""
from pathlib import Path
import copy, hashlib, json, math, shutil, os
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml
from ament_index_python.packages import get_package_share_directory
ROOT=Path(__file__).resolve().parent
COMMON=ROOT.parents[1]/'controller-comparison-8goals-20261014/common'
OLD=ROOT/'laptop-runtime'
FP=[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]
LOCK=json.loads((COMMON/'goals.locked.json').read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def canonical(e):
 e=copy.deepcopy(e)
 for n in e.iter():
  n.tail=None
  if n.text:n.text=n.text.strip() or None
 return ET.tostring(e)
def setup(out,candidate,rtf,goal_index):
 g=LOCK['goals'][goal_index]
 shutil.copytree(OLD/'config',out/'config')
 for p in (out/'config').glob('*.yaml'):
  s=p.read_text().replace('/home/ehdrms/workspace/cleany/cleany-data/runs/lattice-mppi-gui-20261014/runtime-v3-restored/7_other_room',str(out))
  p.write_text(s)
 source=COMMON/g['world'];tree=ET.parse(source);world=tree.getroot().find('world')
 full=ET.parse(COMMON/'world.full.template.sdf').getroot().find('world')
 before={m.get('name'):canonical(m) for m in full.findall('model')};after={m.get('name'):canonical(m) for m in world.findall('model')}
 assert set(before)-set(after)==({g['remove_chair']} if g['remove_chair'] else set())
 assert all(v==before[k] for k,v in after.items())
 for e in world.iter():
  if e.tag not in ('uri','albedo_map','normal_map') or not e.text:continue
  v=e.text
  if v.startswith('fixture://'):v=str(COMMON/'fixture'/v.removeprefix('fixture://'))
  if '/materials/' in v and v.startswith('file:///home/ehdrms/'):
   v=str(ROOT.parents[1]/'controller-comparison-20261013/source/ros2_ws/src/cleany_gazebo_sim/materials'/v.split('/materials/')[1])
  assert Path(v.removeprefix('file://')).is_file(),v
  e.text=v
 world.find('physics/real_time_factor').text=str(rtf)
 robot=world.find("model[@name='cleany_mecanum']")
 physical=canonical(robot);topics=[]
 for link in robot.findall('link'):
  collisions=link.findall('collision')
  if not collisions:continue
  topic='/evaluation/contacts/'+link.get('name');topics.append(topic)
  # Existing sensors can stay: add an independent observer to every collision.
  sensor=ET.SubElement(link,'sensor',name='doorway_contact_probe',type='contact')
  contact=ET.SubElement(sensor,'contact')
  for c in collisions:ET.SubElement(contact,'collision').text=c.get('name')
  ET.SubElement(contact,'topic').text=topic
  ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='50'
 if world.find("plugin[@name='ignition::gazebo::systems::Contact']") is None:
  ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='ignition::gazebo::systems::Contact')
 tree.write(out/'world.sdf',encoding='unicode',xml_declaration=True)
 # Exactly the laptop conservative downsample, not the older independent 5cm raster.
 mp=yaml.safe_load((COMMON/g['map']).read_text());a=np.asarray(Image.open((COMMON/g['map']).with_name('map.pgm')))
 assert mp['resolution']==.025
 Image.fromarray(a.reshape(a.shape[0]//2,2,a.shape[1]//2,2).min(axis=(1,3))).save(out/'map.pgm')
 mp.update(resolution=.05,image='map.pgm');(out/'map.yaml').write_text(yaml.safe_dump(mp))
 oldroot='/home/ehdrms/workspace/cleany/cleany-data/runs/lattice-mppi-gui-20261014/runtime-v3-restored/7_other_room'
 params=yaml.safe_load((OLD/'effective_parameters.yaml').read_text().replace(oldroot,str(out)))
 for name in ('local_costmap','global_costmap'):
  params[name][name]['ros__parameters'].update(footprint=str(FP),footprint_padding=0.)
 params['collision_monitor']['ros__parameters']['StopZone']['points']=[v for point in FP for v in point]
 ctrl=params['controller_server']['ros__parameters']
 ctrl['FollowPath'].pop('ObstacleFootprint.failure_snapshot_directory',None)
 planner=params['planner_server']['ros__parameters']['GridBased']
 for k in ['holonomic_direct_path','require_exact_goal','lattice_collision_padding','failure_snapshot_directory']:planner.pop(k,None)
 assert ctrl['FollowPath']['PathAlignCritic']['offset_from_furthest']==2
 params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(out/'behavior.xml')
 (out/'behavior.xml').write_text((OLD/'baseline_bt.xml').read_text())
 if os.environ.get('MPPI_EXPERIMENT_MODE')=='pathseed':
  ctrl['FollowPath'].update(path_seed_count=64,path_seed_max_speed=.25,path_progress_timeout=12.,path_progress_distance=.08)
  ctrl['failure_tolerance']=.5
  ctrl['progress_checker'].update(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.10,movement_time_allowance=20.)
  (out/'behavior.xml').write_text((ROOT/'recovery_bt.xml').read_text())
 if os.environ.get('MPPI_BT_OVERRIDE'):
  shutil.copyfile(os.environ['MPPI_BT_OVERRIDE'],out/'behavior.xml')
 if os.environ.get('MPPI_CONFIG_OVERRIDE'):
  extra=yaml.safe_load(Path(os.environ['MPPI_CONFIG_OVERRIDE']).read_text())
  for k,v in extra.items():
   if isinstance(v,dict):ctrl['FollowPath'].setdefault(k,{}).update(v)
   else:ctrl['FollowPath'][k]=v
 approach_stages=[]
 if os.environ.get('MPPI_APPROACH_CONFIG'):
  approach_config=yaml.safe_load(Path(os.environ['MPPI_APPROACH_CONFIG']).read_text())
  approach_stages=approach_config.get(g['id'],[])
  ctrl['goal_checker_plugins']=['goal_checker','stage_goal_checker']
  ctrl['stage_goal_checker']=dict(plugin='nav2_controller::SimpleGoalChecker',xy_goal_tolerance=.035,yaw_goal_tolerance=math.radians(3),stateful=False)
  ctrl['stage_goal_checker'].update(approach_config.get('stage_goal_checker',{}))
  (out/'stage_behavior.xml').write_text((out/'behavior.xml').read_text().replace('goal_checker_id="goal_checker"','goal_checker_id="stage_goal_checker"'))
  shutil.copyfile(ROOT/'stage_hold_path.xml',out/'stage_hold_path.xml')
  shutil.copyfile(os.environ['MPPI_APPROACH_CONFIG'],out/'approach_config.yaml')
 (out/'parameters.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
 (out/'planner_params.yaml').write_text(yaml.safe_dump({'planner_server':params['planner_server']}))
 (out/'config/comparison_motion.yaml').write_text(yaml.safe_dump({'controller_server':params['controller_server']}))
 scenario=json.loads((OLD/'scenario.json').read_text())
 scenario['approach_stages']=approach_stages
 scenario.update(test_command_hold=os.environ.get('MPPI_TEST_HOLD'),world=world.get('name'),contacts_topics=topics,initial_pose=LOCK['home_world_map'],actual_spawn_world=list(map(float,robot.findtext('pose').split())),initial_pose_covariance=LOCK['initial_pose_covariance'],goal_record=g,removed_chair=g['remove_chair'],canonical_world_sha256=sha(source),runtime_map_sha256=sha(out/'map.pgm'),preserved_model_count=len(after),physical_models_unchanged_before_contact_sensors=True,mesh_assets=[],preserved_chair_models=sorted(k for k in after if 'chair' in k.lower()))
 scenario['model_metadata']['navigation_geometry'].update(stop=FP,planning_padding=0.)
 (out/'scene_validation.json').write_text(json.dumps({k:scenario[k] for k in ['canonical_world_sha256','removed_chair','preserved_model_count','preserved_chair_models','physical_models_unchanged_before_contact_sensors','mesh_assets','actual_spawn_world']},indent=2))
 (out/'scenario.json').write_text(json.dumps(scenario,indent=2))
 launch=(ROOT.parents[1]/'controller-comparison-8goals-20261014/teb/run_20261014_0029/final8/7_other_room/run.launch.py').read_text()
 launch=launch.replace("parameters=str(p/'config/relaxed_parameters.yaml')","parameters=str(p/'parameters.yaml')")
 launch=launch.replace("str(p/'gazebo_relaxed.launch.py')",repr(str(Path(get_package_share_directory('cleany_gazebo_sim'))/'launch/gazebo_fortress.launch.py')))
 start=launch.index(' limiter=');end=launch.index('\n return LaunchDescription',start)
 limiter_path=ROOT/'fault_wheel_limit.py' if os.environ.get('MPPI_TEST_HOLD') else OLD/'wheel_limit_node.py'
 launch=launch[:start]+f" limiter=ExecuteProcess(cmd=['python3',{str(limiter_path)!r}],output='screen')\n contacts=Node(package='ros_gz_bridge',executable='parameter_bridge',name='contact_bridge',arguments={repr([t+'@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' for t in topics])},parameters=[{{'use_sim_time':True}}],output='screen')"+launch[end:]
 launch=launch.replace(' return LaunchDescription([gazebo,*adapters,navigation,counterfactual,limiter,contacts])',f" snapshots=ExecuteProcess(cmd=['python3',{str(ROOT/'snapshot_logger.py')!r},{str(out/'snapshots')!r}],output='screen')\n return LaunchDescription([gazebo,*adapters,navigation,counterfactual,limiter,contacts,snapshots])")
 (out/'run.launch.py').write_text(launch)
 return scenario
