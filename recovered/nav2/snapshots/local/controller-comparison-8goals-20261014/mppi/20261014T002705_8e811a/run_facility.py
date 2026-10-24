#!/usr/bin/env python3
"""Isolated descriptive controller experiments; no acceptance/pass score."""
import traceback
import argparse, csv, hashlib, json, math, os, signal, subprocess, sys, time, uuid
from pathlib import Path
import numpy as np
import yaml
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

ROOT=Path(os.environ.get('CLEANY_COMPARISON_REPO',str(Path(__file__).parent/'source')))
sys.path.insert(0,str(ROOT/'tools/navigation_evaluation'))
from run_safety_evaluation import prepare, stop
from relaxed_safety_probe import SafetyProbe
from ros_gz_interfaces.msg import Contacts
from xml.etree import ElementTree as ET

RUN_ROOT=Path(__file__).resolve().parent
SNAPSHOT=RUN_ROOT/'common_snapshot'
GOALS=[]
GOAL_RECORDS=[]
STARTUP_ONLY=False
HOME=[-1.865,-4.705,1.5708]
LIMITS={'wheel_rad_s':5.,'xy_tolerance':.30,'yaw_tolerance':math.radians(10)}

def load_common(preliminary=False):
 global GOALS,GOAL_RECORDS,HOME,SNAPSHOT
 SNAPSHOT=RUN_ROOT/('preliminary' if preliminary else 'common_snapshot')
 path=SNAPSHOT/('plan.proposed.json' if preliminary else 'goals.locked.json')
 if not preliminary and not (SNAPSHOT/'READY').exists():raise RuntimeError('Common READY missing; campaign prohibited')
 locked=json.loads(path.read_text())
 GOAL_RECORDS=locked['goals']
 GOALS=[(x['id'],x['pose_map_world']) for x in GOAL_RECORDS]
 HOME=locked.get('home_world_map',[-1.865,-4.705,1.5708])
 assert len(GOALS)==8 and HOME==[-1.865,-4.705,1.5708]
 return locked

def canonical(element):
 clone=ET.fromstring(ET.tostring(element))
 for e in clone.iter():
  e.tail=None
  if e.text:e.text=e.text.strip() or None
 return ET.tostring(clone)

def facility_world(out,scenario,goal_index,rtf):
 fixture=SNAPSHOT/'fixture'
 # Canonical snapshot path is selected only after the shared READY lock.
 record=GOAL_RECORDS[goal_index];remove=record.get('remove_chair')
 locked='world' in record
 world_source=SNAPSHOT/record['world'] if locked else fixture/'world.template.sdf'
 tree=ET.parse(world_source);world=tree.getroot().find('world')
 reference=ET.parse(SNAPSHOT/'world.full.template.sdf').getroot().find('world') if locked else world
 before={m.get('name'):hashlib.sha256(canonical(m)).hexdigest() for m in reference.findall('model')}
 if locked:
  assert hashlib.sha256(world_source.read_bytes()).hexdigest()==record['world_sha256']
 if remove and not locked:
  target=world.find("model[@name='"+remove+"']")
  if target is None:raise RuntimeError('Requested chair missing: '+remove)
  world.remove(target)
 after={m.get('name'):hashlib.sha256(canonical(m)).hexdigest() for m in world.findall('model')}
 assert set(before)-set(after)==({remove} if remove else set())
 assert all(after[k]==before[k] for k in after)
 robot=world.find("model[@name='cleany_mecanum']")
 pose=list(map(float,robot.findtext('pose').split()))
 assert np.allclose([pose[0],pose[1],pose[5]],HOME,atol=1e-6)
 assets=[]
 for uri in world.iter('uri'):
  value=uri.text or ''
  if value.startswith('fixture://'):
   resolved=(fixture/value[len('fixture://'):]).resolve()
   if not resolved.is_file():raise RuntimeError('Unresolved mesh: '+value)
   assets.append({'uri':value,'path':str(resolved),'sha256':hashlib.sha256(resolved.read_bytes()).hexdigest()})
   uri.text=str(resolved)
  elif value.startswith('file://'):
   resolved=Path(value[7:]);assert resolved.is_file()
  elif value and not Path(value).is_file():raise RuntimeError('Nonportable scene URI: '+value)
 physics=world.find('physics');physics.find('real_time_factor').text=str(rtf)
 tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
 # Probe GT uses world-aligned map coordinates; this reference is not the spawn.
 scenario.update(world=world.get('name'),spawn_pose=[0.,0.,0.,0.,0.,0.],actual_spawn_world=pose,initial_pose=HOME,
   fixture_source=str(fixture),canonical_world_sha256=hashlib.sha256(world_source.read_bytes()).hexdigest(),
   removed_chair=remove,preserved_model_count=len(after),preserved_chair_models=sorted(k for k in after if 'chair' in k.lower()),
   physical_models_unchanged_before_contact_sensors=True,mesh_assets=assets,goal_record=record)
 if locked:
  assert hashlib.sha256((out/'map.pgm').read_bytes()).hexdigest()==record['map_sha256']
  scenario['locked_map_sha256']=record['map_sha256']
  scenario['locked_goals_sha256']=hashlib.sha256((SNAPSHOT/'goals.locked.json').read_bytes()).hexdigest()
  scenario['initial_pose_covariance']=json.loads((SNAPSHOT/'goals.locked.json').read_text())['initial_pose_covariance']
  assert float(physics.findtext('max_step_size'))==.001
 (out/'scene_validation.json').write_text(json.dumps({k:scenario[k] for k in ['canonical_world_sha256','removed_chair','preserved_model_count','preserved_chair_models','physical_models_unchanged_before_contact_sensors','mesh_assets','actual_spawn_world']},indent=2))
 return scenario

def wheel_rates(v,r=.0635,k=(.35+.6038)/2):
 x,y,z=v
 return [(x-y-k*z)/r,(x+y+k*z)/r,(x+y-k*z)/r,(x-y+k*z)/r]

def describe(out):
 (out/'manifest.json').write_text(json.dumps({'goals':GOALS,'limits':LIMITS,'wheel_geometry':{'radius_m':.0635,'wheelbase_m':.35,'separation_m':.6038},'wheel_projection':'all vx/vy/wz uniformly scaled to max wheel 5rad/s','straight_limit_m_s':.3175,'soft_sim_time_s':240,'hard_sim_timeout_s':1200,'wall_watchdog_s':1800,'filter':'GT velocity interpolated at 20Hz sim-time; 0.25s centered moving average before derivatives; jerk RMS and p95; smoothing boundary 0.25s trimmed','no_pass_fail_thresholds':True},indent=2))

def setup(out,candidate,rtf,goal_index):
 fixture=SNAPSHOT/'fixture'
 map_source=SNAPSHOT/GOAL_RECORDS[goal_index]['map'] if 'map' in GOAL_RECORDS[goal_index] else fixture/'facility_integration.yaml'
 scenario=prepare(out,map_source,robot_model='cad_frame',obstacle_memory=True,guard_3d=True,gazebo_profile='fortress',real_time_factor=rtf)
 config=out/'config'
 if (SNAPSHOT/'materialize_common.py').exists():
  import importlib.util
  spec=importlib.util.spec_from_file_location('common_materializer',SNAPSHOT/'materialize_common.py');materializer=importlib.util.module_from_spec(spec);spec.loader.exec_module(materializer)
  materializer.materialize_goal(goal_index,out,common=SNAPSHOT)
  nav=yaml.safe_load((out/'nav2_common.yaml').read_text())
 else:nav=yaml.safe_load((config/'nav2_amcl.yaml').read_text())
 motion=yaml.safe_load((Path(get_package_share_directory('cleany_navigation'))/'config/nav2_omni_motion.yaml').read_text())
 ctrl=motion['controller_server']['ros__parameters']
 ctrl['goal_checker']={'plugin':'nav2_controller::SimpleGoalChecker','stateful':True,'xy_goal_tolerance':.30,'yaw_goal_tolerance':math.radians(10)}
 if candidate!='mppi':raise ValueError('This campaign runs MPPI only')
 # Start from the previously verified effective controller and its final relaxed override.
 f=yaml.safe_load((RUN_ROOT/'baseline_effective_controller.yaml').read_text())['/controller_server']['ros__parameters']['FollowPath']
 for key in ['wrapped_plugin','benchmark_timing_file']:f.pop(key,None)
 f['plugin']='nav2_mppi_controller::MPPIController'
 ctrl['FollowPath']=f
 if os.environ.get('MPPI_OVERRIDE'):f.update(yaml.safe_load(Path(os.environ['MPPI_OVERRIDE']).read_text()))
 # Optional identical timing adapter; the delegated controller keeps its name and parameters.
 if os.environ.get('CLEANY_BENCHMARK_TIMING')=='1':
  f=ctrl['FollowPath'];f['wrapped_plugin']=f['plugin'];f['plugin']='cleany_benchmark::TimedController'
  f['benchmark_timing_file']=str(out/'compute_timing.csv')
 # The Gazebo robot overlay is merged last. Write the controller explicitly
 # there too; no common-layer model, sensor, planner or safety values change.
 safety=yaml.safe_load((config/'nav2_safety.yaml').read_text())
 safety.setdefault('controller_server',{}).setdefault('ros__parameters',{})['FollowPath']=ctrl['FollowPath']
 planner_params=out/'planner_params.yaml'
 if planner_params.exists():
  common_planner=yaml.safe_load(planner_params.read_text())['planner_server']['ros__parameters']['GridBased']
  common_root='/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/common/'
  if common_planner.get('lattice_filepath','').startswith(common_root):common_planner['lattice_filepath']=str(SNAPSHOT/common_planner['lattice_filepath'][len(common_root):])
  if 'failure_snapshot_directory' in common_planner:common_planner['failure_snapshot_directory']=str(out/'planner_failures')
  safety.setdefault('planner_server',{}).setdefault('ros__parameters',{})['GridBased']=common_planner
  nav.setdefault('planner_server',{}).setdefault('ros__parameters',{})['GridBased']=common_planner
 (config/'nav2_safety.yaml').write_text(yaml.safe_dump(safety,sort_keys=False))
 nav['controller_server']['ros__parameters']['FollowPath']={} 
 (config/'nav2_amcl.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
 (config/'comparison_motion.yaml').write_text(yaml.safe_dump(motion,sort_keys=False))
 guard=yaml.safe_load((config/'voxel_guard.yaml').read_text());guard['voxel_guard']['ros__parameters'].update(max_speed=math.hypot(.15,.10),max_yaw_rate=.14)
 (config/'voxel_guard.yaml').write_text(yaml.safe_dump(guard,sort_keys=False))
 launch=out/'run.launch.py';text=launch.read_text().replace("'axis_params_file':", "'axis_motion':'false', 'omni_params_file':str(p/'config/comparison_motion.yaml'), 'output_directory':str(p/'runtime'), 'axis_params_file':")
 launch.write_text(text)
 # The legacy pan/CollisionMonitor/voxel chain remains as shadow diagnostics.
 # Only this simulation-only projector has authority over the backend cmd_vel.
 ctrl['failure_tolerance']=-1.0
 ctrl['progress_checker']['movement_time_allowance']=1500.0
 safety=yaml.safe_load((config/'nav2_safety.yaml').read_text())
 safety['controller_server']['ros__parameters'].update(failure_tolerance=-1.0,
    progress_checker=ctrl['progress_checker'],goal_checker=ctrl['goal_checker'],controller_frequency=20.0)
 (config/'nav2_safety.yaml').write_text(yaml.safe_dump(safety,sort_keys=False))
 scenario=facility_world(out,scenario,goal_index,rtf)
 nav=yaml.safe_load((config/'nav2_amcl.yaml').read_text())
 nav['amcl']['ros__parameters'].update(set_initial_pose=True,always_reset_initial_pose=True,initial_pose={'x':HOME[0],'y':HOME[1],'z':0.0,'yaw':HOME[2]})
 (config/'nav2_amcl.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
 tree=ET.parse(out/'world.sdf');world=tree.getroot().find('world')
 robot=world.find("model[@name='cleany_mecanum']")
 def canonical_xml(element):
  clone=ET.fromstring(ET.tostring(element))
  for part in clone.iter():
   part.tail=None
   if part.text:part.text=part.text.strip() or None
  return ET.tostring(clone)
 physical_before=canonical_xml(robot)
 topics=[]
 for link in robot.findall('link'):
  collisions=link.findall('collision')
  if not collisions:continue
  topic='/evaluation/contacts/'+link.get('name');topics.append(topic)
  sensor=ET.SubElement(link,'sensor',name='mppi_contact_probe',type='contact')
  contact=ET.SubElement(sensor,'contact')
  for collision in collisions:ET.SubElement(contact,'collision').text=collision.get('name')
  ET.SubElement(contact,'topic').text=topic
  ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='50'
 physical_copy=ET.fromstring(ET.tostring(robot))
 for link in physical_copy.findall('link'):
  for sensor in link.findall('sensor'):
   if sensor.get('name')=='mppi_contact_probe':link.remove(sensor)
 assert canonical_xml(physical_copy)==physical_before
 ET.SubElement(world,'plugin',filename='ignition-gazebo-contact-system',name='ignition::gazebo::systems::Contact')
 tree.write(out/'world.sdf',encoding='utf-8',xml_declaration=True)
 text=launch.read_text().replace('from launch.actions import IncludeLaunchDescription',
    'from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess')
 text=text.replace('from launch_ros.actions import Node','from launch_ros.actions import Node, SetRemap')
 text=text.replace("        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),",
    "        GroupAction([SetRemap(src='voxel_guard:cmd_vel',dst='/shadow/cmd_vel'), IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),")
 needle="}.items()),\n    ])"
 assert needle in text
 extra="""}.items())]),
        ExecuteProcess(cmd=['/usr/bin/python3',WHEEL_PROJECTOR_SCRIPT],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='mppi_contact_bridge',
             arguments=CONTACT_BRIDGES,parameters=[{'use_sim_time':True}],output='screen'),
    ])""".replace('CONTACT_BRIDGES',repr([topic+'@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts' for topic in topics])).replace('WHEEL_PROJECTOR_SCRIPT',repr(str(RUN_ROOT/'wheel_limit_node.py')))
 launch.write_text(text.replace(needle,extra))
 scenario.update(stage='relaxed_gazebo_only',bypass=['pan alignment/freshness','CollisionMonitor stop/slowdown','serial voxel/freshness'],
   actual_command_chain=['/nav2/cmd_vel','coupled wheel projection','/cmd_vel'],shadow_final_topic='/shadow/cmd_vel',
   contacts_topics=topics,contact_geometry_unchanged_sha256=hashlib.sha256(physical_before).hexdigest(),
   contact_note='Fortress Contact system covers every collision on all six robot links; floor and self contacts are classified separately. Contact publication may be per physics step despite sensor update_rate=50.',
   soft_sim_time_s=240,hard_sim_timeout_s=1200,wall_watchdog_s=1800,
   candidate=candidate,requested_rtf=rtf,comparison_limits=LIMITS)
 scenario['mppi_acceleration_note']='Nav2 1.1.20 does not read ax_max/ay_max/az_max; no native acceleration limit is claimed.'
 scenario['rpp_limit_note']=None
 (out/'scenario.json').write_text(json.dumps(scenario,indent=2))
 return scenario

class Probe(SafetyProbe):
 def __init__(self,out,scenario):
  self.sensor_samples=[];self.diagnostics=[]
  super().__init__(out,scenario);self.vel=[];self.joints=[];self.commands=[];self.scan_stamps=[];self.plans=[];self.contact_count=0;self.contact_file=(out/'contacts.jsonl').open('w',buffering=1024*1024)
  for topic in scenario['contacts_topics']:
   self.create_subscription(Contacts,topic,lambda m,t=topic:self.contact(m,t),qos_profile_sensor_data)
  self.create_subscription(NavPath,'/plan',self.plan,qos_profile_sensor_data)
  self.create_subscription(Odometry,'/ground_truth/odom',self.velocity,qos_profile_sensor_data)
  self.create_subscription(JointState,'/joint_states',self.joint,qos_profile_sensor_data)
  for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/shadow/cmd_vel','/cmd_vel']:
   self.create_subscription(Twist,topic,lambda m,t=topic:self.cmd(m,t),qos_profile_sensor_data)
  for topic in ['/head_pan/state','/body_guard_3d/state','/wheel_limit/state']:
   self.create_subscription(String,topic,lambda m,t=topic:self.diagnostic(m,t),qos_profile_sensor_data)
 def contact(self,m,t):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  for c in m.contacts:
   self.contact_count+=1
   self.contact_file.write(json.dumps({'received_sim_s':self.now(),'stamp_s':stamp,'topic':t,
      'collision1':{'name':c.collision1.name,'id':c.collision1.id},
      'collision2':{'name':c.collision2.name,'id':c.collision2.id},
      'depths':list(c.depths),'positions':[[v.x,v.y,v.z] for v in c.positions]})+'\n')
 def plan(self,m):
  self.plans.append({'sim_s':self.now(),'frame':m.header.frame_id,'poses':[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in m.poses]})
 def receive(self,topic,msg):
  if hasattr(msg,'header') and topic in ('/scan','/camera/head/depth/points','/ground_truth/odom'):
   t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
   self.sensor_samples.append([topic,t,self.now(),time.monotonic()])
  super().receive(topic,msg)
 def diagnostic(self,m,t):
  try:self.diagnostics.append({'topic':t,'received_sim_s':self.now(),**json.loads(m.data)})
  except ValueError:pass
 def velocity(self,m):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;v=m.twist.twist
  self.vel.append([t,v.linear.x,v.linear.y,v.angular.z])
 def joint(self,m):
  if m.velocity:self.joints.append({'t':self.now(),'names':list(m.name),'velocity':list(m.velocity)})
 def cmd(self,m,t):
  self.commands.append([self.now(),time.monotonic(),t,m.linear.x,m.linear.y,m.angular.z])

def resources(root_pid):
 # Read only descendants of the owned launch tree; linux proc CPU counters.
 records={}
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   st=(p/'stat').read_text();tail=st[st.rfind(')')+2:].split();records[int(p.name)]=(int(tail[1]),float(tail[11])+float(tail[12]),int(tail[21])*os.sysconf('SC_PAGE_SIZE'),(p/'cmdline').read_bytes().replace(b'\x00',b' ').decode(errors='replace'))
  except (OSError,ValueError,IndexError):pass
 owned={root_pid}
 for _ in range(8):owned.update(pid for pid,r in records.items() if r[0] in owned)
 return [{'pid':pid,'cpu_s':records[pid][1]/os.sysconf('SC_CLK_TCK'),'rss_bytes':records[pid][2],'cmd':records[pid][3]} for pid in owned if pid in records]

def summarize_velocity(samples,start,end):
 a=np.asarray(samples,float);a=a[(a[:,0]>=start)&(a[:,0]<=end)] if len(a) else a
 if len(a)<12:return {'available':False}
 _,idx=np.unique(a[:,0],return_index=True);a=a[np.sort(idx)];t=np.arange(a[0,0],a[-1,0],.05)
 if len(t)<12:return {'available':False}
 v=np.column_stack([np.interp(t,a[:,0],a[:,i]) for i in [1,2,3]])
 sm=np.column_stack([np.convolve(v[:,i],np.ones(5)/5,mode='same') for i in range(3)])
 accel=np.gradient(sm,.05,axis=0);jerk=np.gradient(accel,.05,axis=0)[5:-5]
 linear=np.linalg.norm(jerk[:,:2],axis=1);angular=np.abs(jerk[:,2]);speed=np.linalg.norm(v[:,:2],axis=1)
 moving=(speed>=.01)|(np.abs(v[:,2])>=.02);transitions=int(np.sum(moving[1:]&~moving[:-1]))
 def reversals(c,dead):
  s=np.sign(c[np.abs(c)>dead]);return int(np.sum(s[1:]!=s[:-1]))
 return {'available':True,'linear_jerk_rms_m_s3':float(np.sqrt(np.mean(linear**2))),'linear_jerk_p95_m_s3':float(np.percentile(linear,95)),'angular_jerk_rms_rad_s3':float(np.sqrt(np.mean(angular**2))),'angular_jerk_p95_rad_s3':float(np.percentile(angular,95)),'stationary_s':float(np.sum(~moving)*.05),'motion_start_count':transitions,'yaw_reversals':reversals(v[:,2],.02),'lateral_reversals':reversals(v[:,1],.01),'speed_mean_m_s':float(np.mean(speed)),'speed_max_m_s':float(np.max(speed))}

def run(out,candidate,goal_index,rtf,domain):
 out.mkdir(parents=True,exist_ok=False);os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',IGN_IP='127.0.0.1',GZ_IP='127.0.0.1',IGN_DISCOVERY_MSG_PORT=str(22651+2*goal_index),IGN_DISCOVERY_SRV_PORT=str(22652+2*goal_index))
 partition='cleany_comparison_'+uuid.uuid4().hex;os.environ.update(IGN_PARTITION=partition,GZ_PARTITION=partition)
 result={'stage':'relaxed_gazebo_only','candidate':candidate,'goal_index':goal_index,'goal_name':GOALS[goal_index][0],'goal_map':GOALS[goal_index][1],'completed':False,'navigation_goal_sent':False,'domain':domain,'partition':partition,'host':os.uname().nodename,'ign_discovery_ports':{'message':22651+2*goal_index,'service':22652+2*goal_index}};proc=node=bag=None;resource_samples=[]
 started=time.monotonic()
 try:
  scenario=setup(out,candidate,rtf,goal_index);result['model_metadata']=scenario['model_metadata']
  proc=subprocess.Popen(['ros2','launch',str(out/'run.launch.py')],stdout=(out/'launch.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
  (out/'session.json').write_text(json.dumps({'launch_pid':proc.pid,'supervisor_pid':os.getpid(),'partition':partition,'domain':domain}))
  rclpy.init();node=Probe(out,scenario);node.initialize();node.wait(2)
  f=yaml.safe_load((out/'config/comparison_motion.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
  keys=['plugin','wrapped_plugin'] if 'wrapped_plugin' in f else ['plugin']
  result['verified_controller']=node.parameters('/controller_server',['FollowPath.'+k for k in keys])
  subprocess.run(['ros2','param','dump','/controller_server','--no-daemon','--spin-time','3','--output-dir',str(out)],check=True)
  loaded=yaml.safe_load((out/'controller_server.yaml').read_text())['/controller_server']['ros__parameters']['FollowPath']
  def flatten(value,prefix=''):
   rows={}
   for key,item in value.items():
    name=prefix+key
    if isinstance(item,dict):rows.update(flatten(item,name+'.'))
    else:rows[name]=item
   return rows
  expected=flatten(f);actual=flatten(loaded)
  mismatches={k:[v,actual.get(k)] for k,v in expected.items() if v!=actual.get(k)}
  if mismatches:raise RuntimeError('Controller parameter mismatch: '+str(mismatches))
  libraries={}
  for process in resources(proc.pid):
   if '/nav2_controller/controller_server' in process['cmd']:
    for line in Path('/proc/'+str(process['pid'])+'/maps').read_text().splitlines():
     path=line.split()[-1]
     if path.endswith('.so') and 'mppi' in path:
      libraries[path]=hashlib.sha256(Path(path).read_bytes()).hexdigest()
  result['contact_publishers']={t:[x.node_name for x in node.get_publishers_info_by_topic(t)] for t in scenario['contacts_topics']}
  assert all(result['contact_publishers'].values())
  result['loaded_mppi_libraries_sha256']=libraries
  result['verified_relaxed_parent_parameters']=node.parameters('/controller_server',['failure_tolerance','progress_checker.movement_time_allowance','controller_frequency','goal_checker.xy_goal_tolerance','goal_checker.yaw_goal_tolerance'])
  assert result['verified_relaxed_parent_parameters']['goal_checker.xy_goal_tolerance']==.30
  assert result['verified_relaxed_parent_parameters']['goal_checker.yaw_goal_tolerance']==math.radians(10)
  assert result['verified_relaxed_parent_parameters']['controller_frequency']==20.0
  assert np.allclose(node.truth(),HOME,atol=.03),node.truth()
  result['initial_truth_world']=node.truth()
  result['verified_costmap_resolution']=node.parameters('/global_costmap/global_costmap',['resolution'])
  if (SNAPSHOT/'READY').exists():assert result['verified_costmap_resolution']['resolution']==.025
  result['scene_validation']= {k:v for k,v in json.loads((out/'scene_validation.json').read_text()).items() if k!='mesh_assets'}
  scene_cmd=['ign','service','-s','/world/'+scenario['world']+'/scene/info','--reqtype','ignition.msgs.Empty','--reptype','ignition.msgs.Scene','--timeout','10000','--req','']
  scene_reply=subprocess.run(scene_cmd,capture_output=True,text=True,timeout=20)
  (out/'server_scene_info.pbtxt').write_text(scene_reply.stdout)
  (out/'server_scene_info.stderr').write_text(scene_reply.stderr)
  import re
  scene_names=set(re.findall(r'^model \{\n(?:[^\n]*\n)*?  name: \"([^\"]+)\"',scene_reply.stdout,re.MULTILINE))
  expected_models={m.get('name') for m in ET.parse(out/'world.sdf').findall('./world/model')}
  result['server_scene_loading']={'command':scene_cmd,'returncode':scene_reply.returncode,'model_names':sorted(scene_names),'missing_models':sorted(expected_models-scene_names)}
  result['server_scene_loading']['null_reason']=None if scene_names else 'Fortress scene/info returned no model names'
  if not scene_names or expected_models-scene_names:raise RuntimeError('Actual server scene incomplete; see server_scene_info.pbtxt')
  mesh_errors=[line for line in (out/'launch.log').read_text(errors='replace').splitlines() if any(x in line.lower() for x in ['unable to find file','unable to find uri','failed to load mesh','could not find resource','unable to load mesh'])]
  result['server_scene_loading']['missing_mesh_errors']=mesh_errors
  if mesh_errors:raise RuntimeError('Mesh loading errors; navigation not started')
  assert result['verified_relaxed_parent_parameters']['failure_tolerance']==-1.0
  assert result['verified_relaxed_parent_parameters']['progress_checker.movement_time_allowance']==1500.0
  result['verified_planner']=node.parameters('/planner_server',['GridBased.plugin','GridBased.holonomic_direct_path','GridBased.require_exact_goal','GridBased.lattice_collision_padding'])
  assert result['verified_planner']=={'GridBased.plugin':'cleany_navigation::StraightFirstPlanner','GridBased.holonomic_direct_path':True,'GridBased.require_exact_goal':True,'GridBased.lattice_collision_padding':.05}
  planner_file=out/'planner_params.yaml'
  if planner_file.exists():
   expected_planner=yaml.safe_load(planner_file.read_text())['planner_server']['ros__parameters']['GridBased']
   common_root='/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/common/'
   if expected_planner.get('lattice_filepath','').startswith(common_root):expected_planner['lattice_filepath']=str(SNAPSHOT/expected_planner['lattice_filepath'][len(common_root):])
   if 'failure_snapshot_directory' in expected_planner:expected_planner['failure_snapshot_directory']=str(out/'planner_failures')
   actual_planner=node.parameters('/planner_server',['GridBased.'+key for key in expected_planner])
   assert actual_planner=={'GridBased.'+key:v for key,v in expected_planner.items()},actual_planner
   result['verified_common_planner_parameters']=actual_planner
  result['rpp_limit_note']=scenario.get('rpp_limit_note')
  if STARTUP_ONLY:
   node.wait(5)
   result.update(preparation_only=True,navigation_goal_sent=False,completed=True,truth=node.truth(),contact_records_observed=node.contact_count)
   return result
  bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),'/clock','/ground_truth/odom','/odom','/wheel/odom','/joint_states','/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel','/scan','/collision_monitor_state','/tf','/tf_static','/head_pan/state','/body_guard_3d/state','/evaluation/depth_points','/amcl_pose','/plan','/local_costmap/costmap','/global_costmap/costmap','/shadow/cmd_vel','/wheel_limit/state',*scenario['contacts_topics']],stdout=(out/'bag.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
  node.wait(1);goal=GOALS[goal_index][1];req=NavigateToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=node.get_clock().now().to_msg();req.pose.pose.position.x=goal[0];req.pose.pose.position.y=goal[1];req.pose.pose.orientation.z=math.sin(goal[2]/2);req.pose.pose.orientation.w=math.cos(goal[2]/2)
  if not node.nav.wait_for_server(timeout_sec=20):raise TimeoutError('No navigate_to_pose action server')
  result['navigation_goal_sent']=True
  handle=node.future(node.nav.send_goal_async(req));node.active_goal=handle
  if not handle.accepted:raise RuntimeError('Goal rejected')
  future=handle.get_result_async();sim_start=node.now();wall_start=time.monotonic();result['navigation_sim_start_s']=sim_start;result['navigation_wall_start_monotonic_s']=wall_start;node.event('comparison_navigation',goal=goal);last_resource=0;soft_exceeded=False;truth_history=[];last_stagnation=-math.inf
  while not future.done():
   node.tick()
   if time.monotonic()-last_resource>=1:
    resource_samples.append({'wall_s':time.monotonic()-wall_start,'sim_s':node.now(),'processes':resources(proc.pid)});last_resource=time.monotonic()
    if len(resource_samples)%10==0:
     (out/'live_status.json').write_text(json.dumps({'sim_s':node.now(),'navigation_sim_s':node.now()-sim_start,'truth':node.truth(),'last_diagnostics':node.diagnostics[-4:]},indent=2))
   if proc.poll() is not None:raise RuntimeError('Launch exited')
   if node.now()-sim_start>240 and not soft_exceeded:
    node.event('soft_240_sim_seconds_exceeded',truth=node.truth());soft_exceeded=True
   if not truth_history or node.now()-truth_history[-1][0]>=.5:truth_history.append((node.now(),node.truth()))
   if node.now()-last_stagnation>30 and len(truth_history)>1:
    old=next((x for x in reversed(truth_history) if x[0]<=node.now()-30),None)
    if old and math.dist(old[1][:2],node.truth()[:2])<.02:
     node.event('stagnation_30s',truth=node.truth(),last_diagnostics=node.diagnostics[-4:]);last_stagnation=node.now()
   if node.now()-sim_start>1200 or time.monotonic()-wall_start>1800:
    node.future(handle.cancel_goal_async());node.future(future,timeout=10);result['timeout']=True;break
  action=future.result();node.active_goal=None;sim_end=node.now();wall_end=time.monotonic();node.wait(2)
  truth=node.truth();err=math.dist(truth[:2],goal[:2]);yawerr=abs(math.atan2(math.sin(truth[2]-goal[2]),math.cos(truth[2]-goal[2])))
  result.update(stage='relaxed_gazebo_only',soft_240s_exceeded=soft_exceeded,completed=True,action_status=action.status,position_error_m=err,yaw_error_deg=math.degrees(yawerr),final_truth=truth,navigation_sim_s=sim_end-sim_start,navigation_wall_s=wall_end-wall_start,achieved_rtf=(sim_end-sim_start)/(wall_end-wall_start),smoothness=summarize_velocity(node.vel,sim_start,sim_end),navigation_sim_start_s=sim_start,navigation_sim_end_s=sim_end)
  active=[c for c in node.commands if sim_start<=c[0]<=sim_end]
  result['max_command_wheel_rad_s']={topic:max((max(map(abs,wheel_rates(c[3:]))) for c in active if c[2]==topic),default=None) for topic in ['/nav2/cmd_vel','/cmd_vel']}
  result['command_sim_interval_s']={}
  for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/shadow/cmd_vel','/cmd_vel']:
   intervals=np.diff([c[0] for c in active if c[2]==topic]);intervals=intervals[intervals>0]
   result['command_sim_interval_s'][topic]={'mean':float(np.mean(intervals)),'p95':float(np.percentile(intervals,95)),'p99':float(np.percentile(intervals,99))} if len(intervals) else None
  result['sensor_freshness']={}
  for topic in ('/scan','/camera/head/depth/points','/ground_truth/odom'):
   a=np.asarray([s[1:] for s in node.sensor_samples if s[0]==topic and sim_start<=s[2]<=sim_end],float)
   if len(a)>1:
    interval=np.diff(a[:,0]);age=a[:,1]-a[:,0]
    result['sensor_freshness'][topic]={'count':len(a),'sim_period_mean_s':float(np.mean(interval)),'sim_period_p95_s':float(np.percentile(interval,95)),'received_age_p95_s':float(np.percentile(age,95)),'received_age_max_s':float(np.max(age))}
  result['final_actual_velocity']=node.vel[-1][1:] if node.vel else None
  result['safety_action_sample_counts']={str(v):sum(r['action']==v for r in node.rows if r['phase']=='comparison_navigation') for v in set(r['action'] for r in node.rows)}
  result['clearance_note']='Existing probe clearance only concerns injected fixture; no fixture injected. Static-world contact/clearance must be evaluated separately; do not infer collision-free from absence of log errors.'
 except BaseException as e:
  result['error']=f'{type(e).__name__}: {e}'
  (out/'error_traceback.txt').write_text(traceback.format_exc())
 finally:
  if node is not None:
   if result.get('navigation_sim_start_s') is not None and result.get('navigation_sim_end_s') is None:
    end=node.now();duration=end-result['navigation_sim_start_s'];wall_duration=time.monotonic()-result['navigation_wall_start_monotonic_s']
    result.update(navigation_sim_end_s=end,navigation_sim_s=duration,navigation_wall_s=wall_duration,achieved_rtf=duration/wall_duration if wall_duration else None,final_truth=node.truth(),position_error_m=math.dist(node.truth()[:2],GOALS[goal_index][1][:2]),yaw_error_deg=math.degrees(abs(math.atan2(math.sin(node.truth()[2]-GOALS[goal_index][1][2]),math.cos(node.truth()[2]-GOALS[goal_index][1][2])))),smoothness=summarize_velocity(node.vel,result['navigation_sim_start_s'],end))
   try:node.finish();node.save()
   except Exception as e:result['cleanup_error']=str(e)
   for name,rows in [('velocity',node.vel),('commands',node.commands)]:
    with (out/(name+'.csv')).open('w') as f:csv.writer(f).writerows(rows)
   (out/'joints.json').write_text(json.dumps(node.joints));node.destroy_node()
   (out/'diagnostics.json').write_text(json.dumps(node.diagnostics))
   (out/'plans.json').write_text(json.dumps(node.plans))
   with (out/'sensor_freshness.csv').open('w') as f:csv.writer(f).writerows(node.sensor_samples)
  if rclpy.ok():rclpy.shutdown()
  stop(bag);stop(proc)
  if node is not None:node.contact_file.close()
  (out/'resources.json').write_text(json.dumps(resource_samples))
  result['elapsed_wall_s']=time.monotonic()-started
  log=(out/'launch.log').read_text(errors='replace') if (out/'launch.log').exists() else ''
  result['controller_deadline_warning_count']=log.count('Control loop missed its desired rate')
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result.get(k) for k in ['goal_index','goal_name','completed','action_status','position_error_m','yaw_error_deg','navigation_sim_s','navigation_wall_s','error','preparation_only']}),flush=True)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=226);p.add_argument('--rtf',type=float,default=1);p.add_argument('--goal-index',type=int);p.add_argument('--prepare-only',action='store_true');p.add_argument('--startup-only',action='store_true');a=p.parse_args()
 STARTUP_ONLY=a.startup_only
 load_common(preliminary=a.prepare_only or a.startup_only)
 a.output.mkdir(parents=True,exist_ok=False);describe(a.output)
 if a.prepare_only:
  out=a.output/'prepared';out.mkdir();setup(out,'mppi',a.rtf,a.goal_index or 0)
 elif a.startup_only:
  run(a.output/'startup_only','mppi',a.goal_index or 0,a.rtf,a.domain)
 else:
  for i in ([a.goal_index] if a.goal_index is not None else range(8)):
   result=run(a.output/(str(i)+'_'+GOALS[i][0]),'mppi',i,a.rtf,a.domain)
   (RUN_ROOT/'STATUS.json').write_text(json.dumps({'phase':'campaign_navigation_completed' if i==7 else 'campaign_running','last_finished_goal_index':i,'last_finished_goal_name':GOALS[i][0],'last_action_status':result.get('action_status'),'last_error':result.get('error'),'campaign_started':True,'output':str(RUN_ROOT)},indent=2))
   if not result.get('navigation_goal_sent'):raise SystemExit('Startup failed before goal; campaign stopped for diagnosis')
