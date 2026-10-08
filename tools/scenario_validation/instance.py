import os,json,time,signal,subprocess,math,uuid
from pathlib import Path
import argparse
from worlds import prepare,restored_tree,chair_name
from stimuli import add_obstacle,move_obstacle,switch_chair
parser=argparse.ArgumentParser()
parser.add_argument('--root',type=Path,required=True)
parser.add_argument('--fixture',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--seat',required=True)
parser.add_argument('--domain',type=int,required=True)
parser.add_argument('--partition',required=True)
parser.add_argument('--gate',type=Path)
parser.add_argument('--scenario',default='normal',choices=['normal','cancel_outbound','cancel_return','safety_stop','sensor_loss','obstacle','path_failure','blocked'])
parser.add_argument('--sequence',help='Comma-separated seats; switch only chairs between missions')
parser.add_argument('--repeats',type=int,default=1)
args=parser.parse_args()
sequence=args.sequence.split(',') if args.sequence else [args.seat]
if sequence[0]!=args.seat:parser.error('sequence must begin with seat')
for item in sequence:chair_name(item)
os.environ.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',IGN_IP='127.0.0.1',GZ_IP='127.0.0.1',IGN_PARTITION=args.partition,GZ_PARTITION=args.partition,__EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
root=args.root.resolve();fixture=args.fixture.resolve();run=args.output.resolve();run.mkdir(parents=True,exist_ok=False)
(run/'observer.json').write_text(json.dumps({'pid':os.getpid()}))
manifest=prepare(fixture/'world.sdf',fixture/'world.study.sdf',args.seat,run/'world.sdf')
txt=(run/'world.sdf').read_text().replace('/home/ehdrms/workspace/cleany/feat-runtime-integration/artifacts/gui-integration-20261003',str(fixture)).replace('/home/ehdrms/workspace/cleany/feat-runtime-integration','/home/donglee/workspace/cleany/runtime-accuracy-review')
(run/'world.sdf').write_text(txt)
if args.sequence:
 import xml.etree.ElementTree as ET
 tree=ET.parse(run/'world.sdf');world=tree.getroot().find('world')
 for model in world.findall('model'):
  if 'chair' in model.get('name',''):
   ET.SubElement(world,'frame',name='validation_'+model.get('name'),attached_to=model.get('name'))
 tree.write(run/'world.sdf',encoding='utf-8',xml_declaration=True)
 reference_models=restored_tree(fixture/'world.sdf',fixture/'world.study.sdf').getroot().find('world')
 manifest['mutable_chair_entities']=True
 transition_world=world.get('name')
if args.scenario=='blocked':
 import yaml,xml.etree.ElementTree as ET
 target=yaml.safe_load((fixture/'targets.yaml').read_text())['seats'][args.seat]
 tree=ET.parse(run/'world.sdf');world=tree.getroot().find('world')
 for i,(dx,dy,sx,sy) in enumerate([(1.,0.,.15,2.15),(-1.,0.,.15,2.15),(0.,1.,2.15,.15),(0.,-1.,2.15,.15)]):
  model=ET.SubElement(world,'model',name=f'validation_blockade_{i}');ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=f'{target[0]+dx} {target[1]+dy} .5 0 0 0'
  link=ET.SubElement(model,'link',name='body')
  for kind in ('collision','visual'):
   shape=ET.SubElement(link,kind,name=kind);box=ET.SubElement(ET.SubElement(shape,'geometry'),'box');ET.SubElement(box,'size').text=f'{sx} {sy} 1'
 tree.write(run/'world.sdf',encoding='utf-8',xml_declaration=True)
stimulus_world=add_obstacle(run/'world.sdf') if args.scenario=='obstacle' else None
import hashlib
manifest['sha256']=hashlib.sha256((run/'world.sdf').read_bytes()).hexdigest()
(run/'world.manifest.json').write_text(json.dumps(manifest,indent=2))
input_paths=[fixture/'world.sdf',fixture/'world.study.sdf',fixture/'facility_integration.yaml',fixture/'facility_integration.pgm',fixture/'targets.yaml',fixture/'config/nav2_calibrated.yaml',fixture/'config/nav2_safety.yaml',fixture/'config/cad_frame.yaml',fixture/'config/sensor_tf.yaml',fixture/'config/wheel_odometry.yaml',root/'ros2_ws/src/cleany_mission_manager/config/runtime.yaml',root/'ros2_ws/src/cleany_navigation/config/amcl_sim_ideal_wheels.yaml']
input_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in input_paths}
tool_hashes={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in Path(__file__).parent.glob('*.py') if not path.name.startswith('test_')}
children=[];result={'base_commit':'076b893','branch':'test/scrum-426-scenarios','scenario':args.scenario,'domain':args.domain,'partition':args.partition,'world_manifest':manifest,'input_hashes':input_hashes,'tool_hashes':tool_hashes,'requested_rtf':4,'max_step_size':float(os.environ.get('CLEANY_PHYSICS_STEP','0.0066667')),'compact_static_scene':True,'localization_profile':str(root/'ros2_ws/src/cleany_navigation/config/amcl_sim_ideal_wheels.yaml'),'observer_sample_period':.2,'runtime_config':'committed default, navigation_timeout 180','missions':[]}; streams=[]
def launch(name,args):
 f=(run/(name+'.log')).open('w');streams.append(f); p=subprocess.Popen(['ros2','launch',*args],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append((name,p));(run/'children.json').write_text(json.dumps([(name,p.pid) for name,p in children]));print('launched',name,p.pid,flush=True)
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from sensor_msgs.msg import LaserScan,Image
from std_msgs.msg import String
from cleany_interfaces.srv import OfferMission,GetRuntimeSnapshot,CancelMission
from nav2_msgs.srv import ManageLifecycleNodes
from cleany_gazebo_sim.navigation_safety_metrics import rectangle_clearance
import ast,yaml
footprint=yaml.safe_load((fixture/'config/nav2_safety.yaml').read_text())['local_costmap']['local_costmap']['ros__parameters']['footprint']
if isinstance(footprint,str):footprint=ast.literal_eval(footprint)
body_low=[min(v[i] for v in footprint) for i in (0,1)];body_high=[max(v[i] for v in footprint) for i in (0,1)]
body_center=[(a+b)/2 for a,b in zip(body_low,body_high)];body_half=[(b-a)/2 for a,b in zip(body_low,body_high)]
from cleany_interfaces.msg import RuntimeDebugSnapshot
rclpy.init();n=rclpy.create_node('desktop_integration_observer');latest={};counts={};tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,n);trace=(run/'trace.jsonl').open('w'); subscriptions=[]
def record(key,data):
 latest[key]=data;counts[key]=counts.get(key,0)+1
subscriptions.append(n.create_subscription(Clock,'/clock',lambda m:record('clock',m.clock.sec+m.clock.nanosec/1e9),qos_profile_sensor_data))
def odom(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation;record('ground_truth',{'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'twist':[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]})
subscriptions.append(n.create_subscription(Odometry,'/ground_truth/odom',odom,qos_profile_sensor_data))
def pose_value(p):
 q=p.orientation;return {'x':p.position.x,'y':p.position.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
def amcl(m):
 record('amcl',{'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'pose':pose_value(m.pose.pose),'covariance':list(m.pose.covariance)})
def wheel(m):
 record('wheel_odom',{'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'pose':pose_value(m.pose.pose),'twist':[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]})
subscriptions.append(n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,10))
subscriptions.append(n.create_subscription(Odometry,'/wheel/odom',wheel,qos_profile_sensor_data))
subscriptions.append(n.create_subscription(Twist,'/cmd_vel',lambda m:record('cmd_vel',[m.linear.x,m.linear.y,m.angular.z]),10))
def scan(m):
 record('scan_stamp',m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 record('scan_frame',m.header.frame_id)
subscriptions.append(n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data))
subscriptions.append(n.create_subscription(Image,'/camera/head/depth/image_raw',lambda m:record('depth_stamp',m.header.stamp.sec+m.header.stamp.nanosec/1e9),qos_profile_sensor_data))
subscriptions.append(n.create_subscription(RuntimeDebugSnapshot,'/mission/debug_snapshot',lambda m:record('debug',json.loads(m.snapshot_json)),10))
subscriptions.append(n.create_subscription(String,'/reactive_guard/state',lambda m:record('guard',json.loads(m.data)),10))
fault=n.create_publisher(String,'/robot/safety_fault',10)
cancel=n.create_client(CancelMission,'/mission/cancel')
snapshot=n.create_client(GetRuntimeSnapshot,'/mission/snapshot');offer=n.create_client(OfferMission,'/mission/offer')
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
def call(client,req,timeout=8):
 f=client.call_async(req);end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 if not f.done():raise TimeoutError('service '+client.srv_name)
 return f.result()
def snap():return json.loads(call(snapshot,GetRuntimeSnapshot.Request()).snapshot_json)
def write_trace(s):
 try:
  t=tf_buffer.lookup_transform('map','base_link',Time());q=t.transform.rotation;p=t.transform.translation
  latest['map_pose']={'stamp':t.header.stamp.sec+t.header.stamp.nanosec/1e9,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
  if 'scan' in latest:
   t=tf_buffer.lookup_transform('base_link',latest['scan']['frame'],Time());p=t.transform.translation;q=t.transform.rotation
   latest['lidar_mount']={'translation':[p.x,p.y,p.z],'rotation':[q.x,q.y,q.z,q.w]}
 except Exception as e: latest['tf_error']=str(e)
 trace.write(json.dumps({'wall':time.monotonic(),'snapshot':s,'latest':latest,'counts':counts})+'\n');trace.flush()
try:
 launch('sim',['cleany_gazebo_sim','gazebo_fortress.launch.py',f'world:={run}/world.sdf','headless:=true','headless_rendering:=true','physics_real_time_factor:=4','physics_max_step_size:='+os.environ.get('CLEANY_PHYSICS_STEP','0.0066667'),'compact_static_scene:=true','sensor_profile:=lidar_rgbd_nav',f'sensor_config:={fixture}/config/sensor_tf.yaml',f'wheel_odometry_config:={fixture}/config/wheel_odometry.yaml'])
 launch('navigation',['cleany_gazebo_sim','amcl_nav2_safety.launch.py','autostart:=false',f'map:={fixture}/facility_integration.yaml','params_file:='+str(fixture/'config/nav2_calibrated.yaml'),'localization_params_file:='+str(root/'ros2_ws/src/cleany_navigation/config/amcl_sim_ideal_wheels.yaml'),f'safety_params_file:={fixture}/config/nav2_safety.yaml',f'body_profile:={fixture}/config/cad_frame.yaml','safety_mode:=reactive','memory_enabled:=true','voxel_enabled:=false',f'output_directory:={run}/navigation'])
 launch('mission',['cleany_mission_manager','mission_runtime.launch.py',f'targets_file:={fixture}/targets.yaml',f'runtime_config:={root}/ros2_ws/src/cleany_mission_manager/config/runtime.yaml','post_mission:=return_home',f'journal_path:={run}/missions.db','enable_bridge:=false'])
 # Start managers explicitly after discovery, rather than simultaneous autostart.
 for manager in ('localization','navigation','safety'):
  client=n.create_client(ManageLifecycleNodes,f'/lifecycle_manager_{manager}/manage_nodes')
  deadline=time.monotonic()+30
  while not client.service_is_ready():
   if time.monotonic()>deadline:raise TimeoutError(f'{manager} lifecycle discovery')
   spin(.2)
  spin(1)
  response=call(client,ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP),timeout=30)
  if not response.success:raise RuntimeError(f'{manager} lifecycle startup failed')
  print('ACTIVE',manager,flush=True)
 deadline=time.monotonic()+150;s={} 
 while time.monotonic()<deadline:
  spin(1)
  if snapshot.service_is_ready():
   s=snap();write_trace(s)
   if s['ready']:break
 else:raise RuntimeError('readiness timeout: '+json.dumps(s))
 result['ready_snapshot']=s;result['cmd_vel_publishers']=[{'node':i.node_name,'type':i.topic_type} for i in n.get_publishers_info_by_topic('/cmd_vel')];print('READY',result['cmd_vel_publishers'],flush=True)
 (run/'ready.json').write_text(json.dumps({'wall':time.monotonic(),'children':[(name,p.pid) for name,p in children]}))
 if args.gate:
  deadline=time.monotonic()+300
  while not args.gate.exists():
   if time.monotonic()>deadline:raise TimeoutError('start barrier')
   spin(.2)
 previous_seat=args.seat
 for seat in sequence*args.repeats:
  if seat!=previous_seat:
   import copy
   model=copy.deepcopy(reference_models.find(f"model[@name='{chair_name(previous_seat)}']"))
   for element in model.iter('uri'):
    if element.text:element.text=element.text.replace('/home/ehdrms/workspace/cleany/feat-runtime-integration/artifacts/gui-integration-20261003',str(fixture))
   transition=switch_chair(transition_world,model,chair_name(seat))
   result.setdefault('chair_transitions',[]).append({'restored':chair_name(previous_seat),'removed':chair_name(seat),'clock':latest['clock'],'requests':transition})
   previous_seat=seat;spin(2)

  req=OfferMission.Request(mission_id=str(uuid.uuid4()),mission_type='clean_desk',target_kind='SEAT',target_id=seat,requested_by='desktop-validation')
  response=call(offer,req);m={'seat':seat,'mission_id':req.mission_id,'accepted':response.accepted,'reason':response.reason};result['missions'].append(m)
  if not response.accepted:break
  stimulus_at=None;stimulus_done=False;m['events']=[]
  start=time.monotonic();clock0=latest['clock'];m['wall_start']=start;m['clock_start']=clock0;deadline=start+600;previous=''
  while time.monotonic()<deadline:
   spin(.2);s=snap();write_trace(s);state=s['state']+' '+s['bt_stage']
   if s['state']=='WORKING' and 'target_ground_truth' not in m:
    m['target_ground_truth']=latest.get('ground_truth');m['target_map_pose']=latest.get('map_pose')
   if not stimulus_done and 'ground_truth' in latest:
    gt=latest['ground_truth'];moving=math.hypot(*gt['twist'][:2])>.05
    trigger=(s['state']=='NAVIGATE_TO_TARGET' and moving and time.monotonic()-start>2)
    if args.scenario=='cancel_return':trigger=s['state']=='RETURN_HOME' and moving
    if args.scenario not in ('normal','blocked') and trigger:
     stimulus_at=time.monotonic();event={'wall':stimulus_at,'clock':latest['clock'],'ground_truth':gt,'scenario':args.scenario,'state':s['state']}
     if args.scenario.startswith('cancel'):
      response=call(cancel,CancelMission.Request(mission_id=req.mission_id));event['accepted']=response.accepted;event['reason']=response.reason
     elif args.scenario=='safety_stop':fault.publish(String(data='E_STOP'))
     elif args.scenario=='obstacle':
      event['obstacle_pose']=[gt['x']+.95*math.cos(gt['yaw']),gt['y']+.95*math.sin(gt['yaw']),gt['yaw']]
      event.update(move_obstacle(stimulus_world,*event['obstacle_pose']))
     elif args.scenario in ('sensor_loss','path_failure'):
      import re
      log=(run/'navigation.log').read_text()
      node='depth_clearance' if args.scenario=='sensor_loss' else 'planner_server'
      found=re.search(r'\['+node+r'-\d+\]: process started with pid \[(\d+)\]',log)
      if not found:raise RuntimeError(node+' PID not recorded')
      pid=int(found.group(1));argv=Path(f'/proc/{pid}/cmdline').read_bytes()
      if node.encode() not in argv:raise RuntimeError(node+' ownership mismatch')
      os.kill(pid,signal.SIGINT);event['stopped_node']={'name':node,'pid':pid}
     m['events'].append(event);stimulus_done=True
   if stimulus_at is not None:
    decision=latest.get('guard',{}).get('decision','UNKNOWN')
    decisions=m.setdefault('guard_decisions_after_stimulus',{});decisions[decision]=decisions.get(decision,0)+1
    if args.scenario=='obstacle':
     gt=latest['ground_truth'];ox,oy,angle=m['events'][0]['obstacle_pose'];c,ss=math.cos(angle),math.sin(angle)
     cx=gt['x']+math.cos(gt['yaw'])*body_center[0]-math.sin(gt['yaw'])*body_center[1]
     cy=gt['y']+math.sin(gt['yaw'])*body_center[0]+math.cos(gt['yaw'])*body_center[1]
     clearance=rectangle_clearance((c*(cx-ox)+ss*(cy-oy),-ss*(cx-ox)+c*(cy-oy),gt['yaw']-angle),body_half,(0.,0.),(.125,.325))
     m['min_fixture_clearance_m']=min(m.get('min_fixture_clearance_m',float('inf')),clearance)
     m['clearance_scope']='configured padded XY body bounds; sampled GT; not contact sensor'
   if stimulus_at is not None and 'stopped_after_stimulus' not in m:
    gt=latest.get('ground_truth',{});velocity=gt.get('twist',[1,1,1]);cmd=latest.get('cmd_vel',[1,1,1])
    if math.hypot(*velocity[:2])<=.03 and abs(velocity[2])<=.05 and all(abs(v)<1e-6 for v in cmd):
     m['stopped_after_stimulus']={'wall_seconds':time.monotonic()-stimulus_at,'ground_truth':gt,'cmd_vel':cmd,'guard':latest.get('guard')}
   if args.scenario=='sensor_loss' and stimulus_at and time.monotonic()-stimulus_at>4 and not m.get('cleanup_cancel'):
    response=call(cancel,CancelMission.Request(mission_id=req.mission_id));m['cleanup_cancel']={'accepted':response.accepted,'wall':time.monotonic()}
   if state!=previous:print(seat,state,s['reason'],flush=True);previous=state
   if s.get('last_result',{} ) and s['last_result'].get('request',{}).get('mission_id')==req.mission_id:break
  m['matched_terminal']=bool(s.get('last_result') and s['last_result'].get('request',{}).get('mission_id')==req.mission_id)
  m['wall_end']=time.monotonic();m['clock_end']=latest['clock']
  m.update(wall_seconds=time.monotonic()-start,sim_seconds=latest['clock']-clock0,final=s,ground_truth=latest.get('ground_truth'),cmd_vel=latest.get('cmd_vel'));m['measured_rtf']=m['sim_seconds']/m['wall_seconds'];m['terminal_scan_age']=latest['clock']-latest.get('scan_stamp',0);m['terminal_depth_age']=latest['clock']-latest.get('depth_stamp',0);print('RESULT',seat,m['final'].get('last_result',{}).get('outcome'),round(m['measured_rtf'],3),flush=True)
  if s['state']=='ERROR':break
 spin(2);write_trace(snap());result['final_latest']=latest;result['counts']=counts
except (Exception, KeyboardInterrupt) as e:result['error']=repr(e);print('ERROR',repr(e),flush=True)
finally:
 (run/'result.json').write_text(json.dumps(result,indent=2));trace.close()
 for name,p in reversed(children):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for name,p in reversed(children):
  try:p.wait(timeout=18)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 (run/'shutdown.json').write_text(json.dumps({name:p.returncode for name,p in children},indent=2));n.destroy_node();rclpy.shutdown()
