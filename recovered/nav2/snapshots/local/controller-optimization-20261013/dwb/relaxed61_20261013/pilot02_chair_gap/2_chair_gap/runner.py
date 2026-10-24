#!/usr/bin/env python3
"""Isolated descriptive controller experiments; no acceptance/pass score."""
import argparse, csv, hashlib, json, math, os, signal, subprocess, sys, time, uuid
from pathlib import Path
import numpy as np
import yaml
import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_ros import TransformException
from rclpy.time import Time
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

ROOT=Path(os.environ.get('CLEANY_COMPARISON_REPO','/home/ehdrms/workspace/cleany/feat-runtime-integration'))
sys.path.insert(0,str(ROOT/'tools/navigation_evaluation'))
from run_safety_evaluation import prepare, stop
from safety_probe_relaxed import SafetyProbe, yaw
from ros_gz_interfaces.msg import Contacts
from prepare_contact_world import instrument

GOALS=[('straight',[2.,0.,0.]),('turn_arrival',[4.,0.,math.pi/2]),('chair_gap',[3.12,-2.465,0.]),('far_heading',[6.,0.,-math.pi/2]),('distant',[9.405,-3.73,0.])]
CONTACT_TOPICS=[]
LIMITS={'nominal_straight_limit_m_s':.3175,'acc_xy':.4,'acc_yaw':.6,'wheel_rad_s':5.,'xy_tolerance':.15,'yaw_tolerance':math.radians(10)}

def wheel_rates(v,r=.0635,k=(.35+.6038)/2):
 x,y,z=v
 return [(x-y-k*z)/r,(x+y+k*z)/r,(x+y-k*z)/r,(x-y+k*z)/r]

def describe(out):
 (out/'manifest.json').write_text(json.dumps({'goals':GOALS,'limits':LIMITS,'protocol':'Gazebo-only relaxed wheel envelope; no independent vx/vy/yaw box','wheel_geometry':{'radius_m':.0635,'wheelbase_m':.35,'separation_m':.6038},'nominal_max_wheel_rad_s':5.,'command_wheel_cap_rad_s':4.99,'command_margin_note':'0.2 percent numeric/feedback margin; hard observed cap remains 5rad/s','soft_sim_timeout_s':240,'hard_sim_timeout_s':1200,'hard_wall_timeout_s':1800,'safety_bypass':'pan/freshness, Collision Monitor, voxel guard remain running on shadow output; direct wheel-cap relay drives Gazebo','controller_internal_collision_checks':'all retained','contact_instrumentation':'physics contacts, one sensor per robot collision, 20Hz nominal','filter':'GT20Hz sim time, centered5point smoothing, two gradients, .25s both-end trim','no_combined_score':True},indent=2))

def setup(out,candidate,rtf):
 fixture=ROOT/'ros2_ws/src/cleany_gazebo_sim/test/fixtures/navigation_goals'
 scenario=prepare(out,fixture/'map.yaml',robot_model='cad_frame',obstacle_memory=True,guard_3d=True,gazebo_profile='fortress',real_time_factor=rtf)
 config=out/'config';nav=yaml.safe_load((config/'nav2_amcl.yaml').read_text())
 motion=yaml.safe_load((Path(get_package_share_directory('cleany_navigation'))/'config/nav2_omni_motion.yaml').read_text())
 ctrl=motion['controller_server']['ros__parameters']
 ctrl['goal_checker']={'plugin':'nav2_controller::SimpleGoalChecker','stateful':True,'xy_goal_tolerance':.15,'yaw_goal_tolerance':math.radians(10)}
 if candidate=='dwb':
  f=ctrl['FollowPath'];f.update(min_vel_x=-.15,max_vel_x=.15,min_vel_y=-.10,max_vel_y=.10,max_vel_theta=.14,max_speed_xy=math.hypot(.15,.10),acc_lim_x=.4,acc_lim_y=.4,acc_lim_theta=.6,decel_lim_x=-.4,decel_lim_y=-.4,decel_lim_theta=-.6,xy_goal_tolerance=.15,debug_trajectory_details=False,timing_trace_file=str(out/'controller_command_dt.json'))
 elif candidate=='mppi':
  f=nav['controller_server']['ros__parameters']['FollowPath']
  f.update(motion_model='Omni',vx_max=.15,vx_min=-.15,vy_max=.10,wz_max=.14,ax_max=.4,ax_min=-.4,ay_max=.4,ay_min=-.4,az_max=.6)
  ctrl['FollowPath']=f
 else: raise ValueError(candidate)
 f=ctrl['FollowPath'];f['wrapped_plugin']=f['plugin'];f['plugin']='cleany_benchmark::TimedController';f['benchmark_timing_file']=str(out/'compute_timing.csv')
 patch=yaml.safe_load(Path(os.environ['DWB_PATCH']).read_text()) if os.environ.get('DWB_PATCH') else {}
 for key,vals in patch.items():
  if key=='amcl':nav['amcl']['ros__parameters'].update(vals)
  elif key=='controller_server':ctrl.update(vals)
  else:ctrl[key].update(vals)
 nav['controller_server']['ros__parameters']['FollowPath']={}
 (config/'nav2_amcl.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
 (config/'comparison_motion.yaml').write_text(yaml.safe_dump(motion,sort_keys=False))
 guard=yaml.safe_load((config/'voxel_guard.yaml').read_text());guard['voxel_guard']['ros__parameters'].update(max_speed=math.hypot(.15,.10),max_yaw_rate=.14)
 (config/'voxel_guard.yaml').write_text(yaml.safe_dump(guard,sort_keys=False))
 launch=out/'run.launch.py';text=launch.read_text().replace("'axis_params_file':", "'axis_motion':'false', 'omni_params_file':str(p/'config/comparison_motion.yaml'), 'output_directory':str(p/'runtime'), 'axis_params_file':")
 backend_share=Path(get_package_share_directory('cleany_gazebo_sim'))
 backend=yaml.safe_load((backend_share/'config/base.yaml').read_text())
 backend['gazebo_command_guard']['ros__parameters'].update(max_linear_x=1.0,max_linear_y=1.0,max_angular_z=2.0,cmd_vel_timeout_sec=5.0)
 (config/'backend_base.yaml').write_text(yaml.safe_dump(backend,sort_keys=False))
 backend_launch=(backend_share/'launch/gazebo_fortress.launch.py').read_text().replace("base_config = package_share / 'config' / 'base.yaml'", "base_config = Path("+repr(str(config/'backend_base.yaml'))+")")
 (out/'backend.launch.py').write_text(backend_launch)
 text=text.replace("str(share/'launch/gazebo_fortress.launch.py')", "str(p/'backend.launch.py')")
 global CONTACT_TOPICS
 CONTACT_TOPICS=instrument(out)
 text=text.replace('from launch.actions import IncludeLaunchDescription','from launch.actions import IncludeLaunchDescription,GroupAction,ExecuteProcess').replace('from launch_ros.actions import Node','from launch_ros.actions import Node,SetRemap')
 text=text.replace("        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),", "        GroupAction([SetRemap(src='voxel_guard:cmd_vel',dst='shadow/cmd_vel'),IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),")
 bridge_args=[t+'@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts' for t in CONTACT_TOPICS]
 closing=".items())]),\n        Node(package='ros_gz_bridge',executable='parameter_bridge',name='contact_bridge',arguments="+repr(bridge_args)+",parameters=[{'use_sim_time':True}],output='screen'),\n        ExecuteProcess(cmd=['python3',"+repr(str(Path(__file__).parent/'wheel_cap_relay.py'))+"],output='screen'),\n    ])"
 text=text.replace('.items()),\n    ])',closing)
 launch.write_text(text)
 scenario['safety_bypass']='all standalone serial guards are observed on shadow/cmd_vel; direct wheel-envelope relay controls cmd_vel'
 scenario['contact_topics']=CONTACT_TOPICS
 scenario.update(candidate=candidate,requested_rtf=rtf,comparison_limits=LIMITS)
 (out/'scenario.json').write_text(json.dumps(scenario,indent=2))
 return scenario

class Probe(SafetyProbe):
 def __init__(self,out,scenario):
  super().__init__(out,scenario);self.vel=[];self.joints=[];self.commands=[];self.scan_stamps=[];self.pose_frames=[];self.diag=[];self.pan_diag=[];self.relay_diag=[];self.contact_records=[];self.contact_counts={t:0 for t in CONTACT_TOPICS};self.contact_logged={t:0 for t in CONTACT_TOPICS};self.last_ground_contact={}
  self.create_subscription(String,'/body_guard_3d/state',self.diagnostics,10)
  for topic in CONTACT_TOPICS:self.create_subscription(Contacts,topic,lambda m,t=topic:self.contact(m,t),qos_profile_sensor_data)
  self.create_subscription(String,'/benchmark/relay_state',lambda m:self.relay_diag.append({'sim_s':self.now(),'wall_s':time.monotonic(),'data':m.data}),10)
  self.create_subscription(String,'/head_pan/state',lambda m:self.pan_diag.append({'sim_s':self.now(),'wall_s':time.monotonic(),'data':m.data}),10)
  for topic in ['/odom','/wheel/odom']:
   self.create_subscription(Odometry,topic,lambda m,t=topic:self.odometry(m,t),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/ground_truth/odom',self.velocity,qos_profile_sensor_data)
  self.create_subscription(JointState,'/joint_states',self.joint,qos_profile_sensor_data)
  for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/shadow/cmd_vel','/cmd_vel','/gazebo_cmd_vel']:
   self.create_subscription(Twist,topic,lambda m,t=topic:self.cmd(m,t),qos_profile_sensor_data)
 def receive(self,topic,msg):
  super().receive(topic,msg)
  if topic=='/amcl_pose':
   p=msg.pose.pose;self.pose_frames.append({'sim_s':self.now(),'source':'amcl','stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'pose':[p.position.x,p.position.y,yaw(p.orientation)]})
 def contact(self,m,topic):
  self.contact_counts[topic]+=1
  if not m.contacts:return
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if all('ground_plane' in c.collision1.name+' '+c.collision2.name for c in m.contacts):
   if stamp-self.last_ground_contact.get(topic,-1.)<.05:return
   self.last_ground_contact[topic]=stamp
  self.contact_logged[topic]+=1
  record={'sim_s':self.now(),'stamp_s':stamp,'topic':topic,'GT':self.truth() if '/ground_truth/odom' in self.latest else None,'contacts':[]}
  for c in m.contacts:
   record['contacts'].append({'collision1':c.collision1.name,'collision2':c.collision2.name,'depths':list(c.depths),'positions':[[v.x,v.y,v.z] for v in c.positions]})
  self.contact_records.append(record)
 def odometry(self,m,topic):
  p=m.pose.pose;self.pose_frames.append({'sim_s':self.now(),'source':topic,'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'pose':[p.position.x,p.position.y,yaw(p.orientation)]})
 def snapshot(self):
  result={'sim_s':self.now(),'GT':self.truth()}
  for source,target in [('map_base_tf','map'),('/odom','odom'),('/wheel/odom','wheel_odom'),('amcl','amcl')]:
   result[target]=next((x for x in reversed(self.pose_frames) if x['source']==source),None)
  return result
 def diagnostics(self,m):
  self.diag.append({'sim_s':self.now(),'wall_s':time.monotonic(),'data':m.data})
 def tick(self):
  super().tick()
  if len(self.rows) and (not hasattr(self,'last_tf_time') or self.now()-self.last_tf_time>.05):
   try:
    self.last_tf_time=self.now()
    tr=self.buffer.lookup_transform('map','base_link',Time()).transform
    self.pose_frames.append({'sim_s':self.now(),'source':'map_base_tf','pose':[tr.translation.x,tr.translation.y,yaw(tr.rotation)]})
   except Exception:pass
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
 accel_trim=accel[5:-5];linear_accel=np.linalg.norm(accel_trim[:,:2],axis=1);angular_accel=np.abs(accel_trim[:,2]);linear=np.linalg.norm(jerk[:,:2],axis=1);angular=np.abs(jerk[:,2]);speed=np.linalg.norm(v[:,:2],axis=1)
 moving=speed>.01;transitions=int(np.sum(moving[1:]&~moving[:-1]))
 def reversals(c,dead):
  s=np.sign(c[np.abs(c)>dead]);return int(np.sum(s[1:]!=s[:-1]))
 return {'available':True,'linear_acceleration_rms_m_s2':float(np.sqrt(np.mean(linear_accel**2))),'linear_acceleration_p95_m_s2':float(np.percentile(linear_accel,95)),'angular_acceleration_rms_rad_s2':float(np.sqrt(np.mean(angular_accel**2))),'angular_acceleration_p95_rad_s2':float(np.percentile(angular_accel,95)),'motion_stop_count':int(np.sum(~moving[1:]&moving[:-1])),'linear_jerk_rms_m_s3':float(np.sqrt(np.mean(linear**2))),'linear_jerk_p95_m_s3':float(np.percentile(linear,95)),'angular_jerk_rms_rad_s3':float(np.sqrt(np.mean(angular**2))),'angular_jerk_p95_rad_s3':float(np.percentile(angular,95)),'stationary_s':float(np.sum(~moving)*.05),'motion_start_count':transitions,'yaw_reversals':reversals(v[:,2],.02),'lateral_reversals':reversals(v[:,1],.01),'speed_mean_m_s':float(np.mean(speed)),'speed_max_m_s':float(np.max(speed))}

def motion_keys():
 return ['plugin','wrapped_plugin','trajectory_generator_name','max_vel_x','max_vel_y','max_vel_theta','min_vel_x','min_vel_y','acc_lim_x','acc_lim_y','decel_lim_x','decel_lim_y','acc_lim_theta','decel_lim_theta','vx_samples','vy_samples','vtheta_samples','sim_time','fine_translation_scale','goal_directed_radius','xy_goal_tolerance','RotateToGoal.slowing_factor','GoalDist.scale','GoalDist.near_goal_distance','ObstacleFootprint.class','ObstacleFootprint.scale','max_wheel_speed_rad_s','mecanum_wheel_radius_m','mecanum_wheelbase_m','mecanum_wheel_separation_m','preserve_translation_direction','prefer_target_translation_direction','use_elapsed_command_time']

def run(out,candidate,goal_index,rtf,domain):
 out.mkdir(parents=True,exist_ok=False)
 (out/'runner.py').write_text(Path(__file__).read_text())
 if os.environ.get('DWB_PATCH'):(out/'parameter_patch.yaml').write_text(Path(os.environ['DWB_PATCH']).read_text())
 provenance={'source_checkout_commit':os.environ['CLEANY_COMPARISON_COMMIT'],'comparison_baseline_commit':'b3ca816ba420452ee7960837bc16ec74148e07b0','architecture':os.uname().machine,'nav2_version':subprocess.check_output(['dpkg-query','-W','-f=${Version}','ros-humble-nav2-controller'],text=True).strip(),'library_sha256':{},'source_sha256':{}}
 for path in (ROOT/'ros2_ws/src/cleany_navigation').rglob('*'):
  if path.is_file() and path.suffix in ['.cpp','.hpp','.yaml','.xml']:provenance['source_sha256'][str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
 for prefix in os.environ.get('AMENT_PREFIX_PATH','').split(':'):
  for path in (Path(prefix)/'lib').glob('libaxis_generator.so'):provenance['library_sha256'][str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
 (out/'provenance.json').write_text(json.dumps(provenance,indent=2));os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',IGN_IP='127.0.0.1',GZ_IP='127.0.0.1')
 partition='cleany_comparison_'+uuid.uuid4().hex;os.environ.update(IGN_PARTITION=partition,GZ_PARTITION=partition)
 result={'candidate':candidate,'goal_index':goal_index,'goal_name':GOALS[goal_index][0],'goal_map':GOALS[goal_index][1],'completed':False,'domain':domain,'partition':partition,'host':os.uname().nodename,'discovery_ports':{'msg':os.environ.get('IGN_DISCOVERY_MSG_PORT'),'srv':os.environ.get('IGN_DISCOVERY_SRV_PORT')}};proc=node=bag=None;resource_samples=[]
 started=time.monotonic()
 try:
  scenario=setup(out,candidate,rtf);result['model_metadata']=scenario['model_metadata']
  proc=subprocess.Popen(['ros2','launch',str(out/'run.launch.py')],stdout=(out/'launch.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
  (out/'session.json').write_text(json.dumps({'launch_pid':proc.pid,'supervisor_pid':os.getpid(),'partition':partition,'domain':domain,'discovery_ports':result['discovery_ports']}))
  rclpy.init();node=Probe(out,scenario);node.initialize();node.wait(2)
  keys=['FollowPath.'+k for k in motion_keys()]+['goal_checker.plugin','goal_checker.xy_goal_tolerance','goal_checker.yaw_goal_tolerance','controller_frequency','failure_tolerance','progress_checker.movement_time_allowance']
  actual=node.parameters('/controller_server',keys)
  (out/'controller_parameters.json').write_text(json.dumps(actual,indent=2))
  assert actual['goal_checker.xy_goal_tolerance']==.15 and abs(actual['goal_checker.yaw_goal_tolerance']-math.radians(10))<1e-10
  assert actual['FollowPath.max_wheel_speed_rad_s']==4.99
  assert actual['FollowPath.mecanum_wheel_radius_m']==.0635 and actual['FollowPath.mecanum_wheelbase_m']==.35 and actual['FollowPath.mecanum_wheel_separation_m']==.6038
  assert actual['failure_tolerance']==-1.0 and actual['progress_checker.movement_time_allowance']>=1200
  loaded={}
  for process in resources(proc.pid):
   if 'controller_server' in process['cmd']:
    loaded[str(process['pid'])]=[line.split()[-1] for line in Path('/proc/'+str(process['pid'])+'/maps').read_text().splitlines() if 'libaxis_generator.so' in line or 'libtimed_controller.so' in line]
  (out/'loaded_controller_libraries.json').write_text(json.dumps(loaded,indent=2))
  raw_publishers=[x.node_name for x in node.get_publishers_info_by_topic('/nav2/cmd_vel')]
  shadow_publishers=[x.node_name for x in node.get_publishers_info_by_topic('/shadow/cmd_vel')]
  assert 'controller_server' in raw_publishers and shadow_publishers==['voxel_guard'],(raw_publishers,shadow_publishers)
  (out/'delivery_publishers.json').write_text(json.dumps({'nav2_cmd_vel':raw_publishers,'shadow_cmd_vel':shadow_publishers,'cmd_vel':[x.node_name for x in node.get_publishers_info_by_topic('/cmd_vel')]},indent=2))
  backend_actual=node.parameters('/gazebo_command_guard',['max_linear_x','max_linear_y','max_angular_z','cmd_vel_timeout_sec'])
  assert backend_actual=={'max_linear_x':1.0,'max_linear_y':1.0,'max_angular_z':2.0,'cmd_vel_timeout_sec':5.0},backend_actual
  (out/'backend_parameters.json').write_text(json.dumps(backend_actual,indent=2))
  result['initial_pose_snapshot']=node.snapshot()
  bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),'/clock','/ground_truth/odom','/odom','/joint_states','/nav2/cmd_vel','/cmd_vel','/scan','/collision_monitor_state','/body_guard_3d/state','/safety_2d/cmd_vel','/gazebo_cmd_vel','/wheel/odom','/amcl_pose','/tf','/tf_static','/navigate_to_pose/_action/status','/follow_path/_action/status','/pan_ready/cmd_vel','/head_pan/state','/shadow/cmd_vel','/benchmark/relay_state',*CONTACT_TOPICS],stdout=(out/'bag.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
  node.wait(1);goal=GOALS[goal_index][1];req=NavigateToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=node.get_clock().now().to_msg();req.pose.pose.position.x=goal[0];req.pose.pose.position.y=goal[1];req.pose.pose.orientation.z=math.sin(goal[2]/2);req.pose.pose.orientation.w=math.cos(goal[2]/2)
  if not node.nav.wait_for_server(timeout_sec=20):raise TimeoutError('No navigate_to_pose action server')
  handle=node.future(node.nav.send_goal_async(req));node.active_goal=handle
  if not handle.accepted:raise RuntimeError('Goal rejected')
  future=handle.get_result_async();sim_start=node.now();wall_start=time.monotonic();node.event('comparison_navigation',goal=goal);last_resource=0;result['navigation_started']=True;result['navigation_sim_start_s']=sim_start;result['navigation_wall_start_monotonic_s']=wall_start
  while not future.done():
   node.tick()
   if time.monotonic()-last_resource>=1:
    resource_samples.append({'wall_s':time.monotonic()-wall_start,'sim_s':node.now(),'processes':resources(proc.pid)});last_resource=time.monotonic()
   if proc.poll() is not None:raise RuntimeError('Launch exited')
   if node.now()-sim_start>240 and 'soft_240_sim_s_crossed_at' not in result:result['soft_240_sim_s_crossed_at']=node.now()
   if node.now()-sim_start>1200 or time.monotonic()-wall_start>1800:
    node.future(handle.cancel_goal_async());node.future(future,timeout=10);result['timeout']=True;break
  action=future.result();node.active_goal=None;sim_end=node.now();wall_end=time.monotonic();result['action_pose_snapshot']=node.snapshot();result['navigation_sim_end_s']=sim_end;node.event('comparison_outcome',status=action.status);node.wait(2);result['settled_pose_snapshot']=node.snapshot()
  truth=node.truth();err=math.dist(truth[:2],goal[:2]);yawerr=abs(math.atan2(math.sin(truth[2]-goal[2]),math.cos(truth[2]-goal[2])))
  result.update(completed=True,action_status=action.status,position_error_m=err,yaw_error_deg=math.degrees(yawerr),final_truth=truth,navigation_sim_s=sim_end-sim_start,navigation_wall_s=wall_end-wall_start,achieved_rtf=(sim_end-sim_start)/(wall_end-wall_start),smoothness=summarize_velocity(node.vel,sim_start,sim_end))
  active=[c for c in node.commands if sim_start<=c[0]<=sim_end]
  raw_times=[c[0] for c in active if c[2]=='/nav2/cmd_vel'];intervals=np.diff(raw_times);intervals=intervals[intervals>0]
  result['controller_command_sim_interval_s']={'mean':float(np.mean(intervals)),'p95':float(np.percentile(intervals,95)),'count':len(intervals)} if len(intervals) else None
  result['max_command_wheel_rad_s']={topic:max((max(map(abs,wheel_rates(c[3:]))) for c in active if c[2]==topic),default=None) for topic in ['/nav2/cmd_vel','/cmd_vel']}
  result['final_actual_velocity']=node.vel[-1][1:] if node.vel else None
  result['safety_action_sample_counts']={str(v):sum(r['action']==v for r in node.rows if r['phase']=='comparison_navigation') for v in set(r['action'] for r in node.rows)}
  result['clearance_note']='Existing probe clearance only concerns injected fixture; no fixture injected. Static-world contact/clearance must be evaluated separately; do not infer collision-free from absence of log errors.'
 except BaseException as e:
  result['error']=f'{type(e).__name__}: {e}'
 finally:
  if node is not None:
   try:node.finish();node.save()
   except Exception as e:result['cleanup_error']=str(e)
   for name,rows in [('velocity',node.vel),('commands',node.commands)]:
    with (out/(name+'.csv')).open('w') as f:csv.writer(f).writerows(rows)
   (out/'joints.json').write_text(json.dumps(node.joints));(out/'poses.json').write_text(json.dumps(node.pose_frames));(out/'guard_diagnostics.json').write_text(json.dumps(node.diag));(out/'pan_diagnostics.json').write_text(json.dumps(node.pan_diag));(out/'relay_diagnostics.json').write_text(json.dumps(node.relay_diag));(out/'contacts.json').write_text(json.dumps(node.contact_records));(out/'contact_topic_counts.json').write_text(json.dumps(node.contact_counts));(out/'contact_logged_counts.json').write_text(json.dumps(node.contact_logged));node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
  stop(bag);stop(proc)
  result['owned_launch_stopped']=proc is None or proc.poll() is not None
  result['owned_bag_stopped']=bag is None or bag.poll() is not None
  (out/'resources.json').write_text(json.dumps(resource_samples))
  result['elapsed_wall_s']=time.monotonic()-started
  log=(out/'launch.log').read_text(errors='replace') if (out/'launch.log').exists() else ''
  result['no_valid_trajectory_warning_count']=log.count('No valid trajectories')
  result['controller_deadline_warning_count']=log.count('Control loop missed its desired rate')
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--candidate',choices=['dwb','mppi'],required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=225);p.add_argument('--rtf',type=float,default=1);p.add_argument('--goal-index',type=int);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);describe(a.output)
 for i in ([a.goal_index] if a.goal_index is not None else range(5)):
  r=run(a.output/(str(i)+'_'+GOALS[i][0]),a.candidate,i,a.rtf,a.domain)
  if not r['completed']:
   if not r.get('navigation_started', False) and 'TimeoutError' in r.get('error',''):
    r=run(a.output/(str(i)+'_'+GOALS[i][0]+'_startup_retry'),a.candidate,i,a.rtf,a.domain)
   if not r['completed'] and 'KeyboardInterrupt' in r.get('error',''):break
