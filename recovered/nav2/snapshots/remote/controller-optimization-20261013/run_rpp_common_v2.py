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
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

ROOT=Path(os.environ.get('CLEANY_COMPARISON_REPO',str(Path(__file__).parent/'source')))
sys.path.insert(0,str(ROOT/'tools/navigation_evaluation'))
from run_safety_evaluation import prepare, stop
from safety_probe import SafetyProbe

GOALS=[('straight',[2.,0.,0.]),('turn_arrival',[4.,0.,math.pi/2]),('chair_gap',[3.12,-2.465,0.]),('far_heading',[6.,0.,-math.pi/2]),('distant',[9.405,-3.73,0.])]
LIMITS={'vx':.15,'vy':.10,'wz':.14,'acc_xy':.4,'acc_yaw':.6,'wheel_rad_s':5.,'xy_tolerance':.15,'yaw_tolerance':math.radians(10)}

def wheel_rates(v,r=.0635,k=(.35+.6038)/2):
 x,y,z=v
 return [(x-y-k*z)/r,(x+y+k*z)/r,(x+y-k*z)/r,(x-y+k*z)/r]

def describe(out):
 (out/'manifest.json').write_text(json.dumps({'goals':GOALS,'limits':LIMITS,'wheel_geometry':{'radius_m':.0635,'wheelbase_m':.35,'separation_m':.6038},'wheel_worst_case_rad_s':max(map(abs,wheel_rates([.15,.1,.14]))),'sim_timeout_s':240,'filter':'GT velocity interpolated at 20Hz sim-time; 0.25s centered moving average before derivatives; jerk RMS and p95; smoothing boundary 0.25s trimmed','no_pass_fail_thresholds':True},indent=2))

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
 elif candidate=='teb':
  geometry=scenario['model_metadata']['navigation_geometry'];padding=geometry['planning_padding']
  padded=[[x+math.copysign(padding,x),y+math.copysign(padding,y)] for x,y in geometry['stop']]
  ctrl['FollowPath']={
   'plugin':'teb_local_planner::TebLocalPlannerROS',
   'max_vel_x':.15,'max_vel_x_backwards':.15,'max_vel_y':.10,'max_vel_theta':.14,
   'acc_lim_x':.4,'acc_lim_y':.4,'acc_lim_theta':.6,
   'min_turning_radius':0.,'cmd_angle_instead_rotvel':False,'free_goal_vel':False,
   'weight_kinematics_nh':0.,'weight_kinematics_forward_drive':0.,
   'global_plan_overwrite_orientation':False,'penalty_epsilon':.01,
   'dt_ref':.3,'dt_hysteresis':.1,'max_global_plan_lookahead_dist':3.,
   'include_costmap_obstacles':True,'costmap_converter_plugin':'',
   'enable_homotopy_class_planning':True,'enable_multithreading':True,
   'footprint_model':{'type':'polygon','vertices':str(padded)},
   'min_obstacle_dist':0.,'inflation_dist':.2,'odom_topic':'odom'
  }
 elif candidate=='rpp':
  ctrl['FollowPath']={
   'plugin':'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
   'desired_linear_vel':.15,'lookahead_dist':.6,'min_lookahead_dist':.3,'max_lookahead_dist':.9,
   'lookahead_time':1.5,'rotate_to_heading_angular_vel':.14,
   'transform_tolerance':.3,'use_velocity_scaled_lookahead_dist':True,
   'min_approach_linear_velocity':.03,'approach_velocity_scaling_dist':.6,
   'use_collision_detection':True,'max_allowed_time_to_collision_up_to_carrot':1.,
   'use_regulated_linear_velocity_scaling':True,'use_cost_regulated_linear_velocity_scaling':True,
   'regulated_linear_scaling_min_radius':.9,'regulated_linear_scaling_min_speed':.03,
   'use_rotate_to_heading':True,'rotate_to_heading_min_angle':.785,
   'allow_reversing':False,'max_angular_accel':.6,
   'cost_scaling_dist':.6,'cost_scaling_gain':1.,'inflation_cost_scaling_factor':3.,
  }
 else: raise ValueError(candidate)
 # Optional identical timing adapter; the delegated controller keeps its name and parameters.
 if os.environ.get('CLEANY_BENCHMARK_TIMING')=='1':
  f=ctrl['FollowPath'];f['wrapped_plugin']=f['plugin'];f['plugin']='cleany_benchmark::TimedController'
  f['benchmark_timing_file']=str(out/'compute_timing.csv')
 nav['controller_server']['ros__parameters']['FollowPath']={}
 (config/'nav2_amcl.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
 (config/'comparison_motion.yaml').write_text(yaml.safe_dump(motion,sort_keys=False))
 guard=yaml.safe_load((config/'voxel_guard.yaml').read_text());guard['voxel_guard']['ros__parameters'].update(max_speed=math.hypot(.15,.10),max_yaw_rate=.14)
 (config/'voxel_guard.yaml').write_text(yaml.safe_dump(guard,sort_keys=False))
 launch=out/'run.launch.py';text=launch.read_text().replace("'axis_params_file':", "'axis_motion':'false', 'omni_params_file':str(p/'config/comparison_motion.yaml'), 'output_directory':str(p/'runtime'), 'axis_params_file':")
 launch.write_text(text)
 scenario.update(candidate=candidate,requested_rtf=rtf,comparison_limits=LIMITS)
 scenario['rpp_limit_note']='Humble RPP has no native translational acceleration or tracking angular-velocity limit. Final common voxel guard caps speed/yaw; controller acceleration settings apply natively only to DWB/MPPI/TEB. No shared velocity smoother is present.' if candidate=='rpp' else None
 (out/'scenario.json').write_text(json.dumps(scenario,indent=2))
 return scenario

class Probe(SafetyProbe):
 def __init__(self,out,scenario):
  self.sensor_samples=[];self.diagnostics=[]
  super().__init__(out,scenario);self.vel=[];self.joints=[];self.commands=[];self.scan_stamps=[]
  self.create_subscription(Odometry,'/ground_truth/odom',self.velocity,qos_profile_sensor_data)
  self.create_subscription(JointState,'/joint_states',self.joint,qos_profile_sensor_data)
  for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel']:
   self.create_subscription(Twist,topic,lambda m,t=topic:self.cmd(m,t),qos_profile_sensor_data)
  for topic in ['/head_pan/state','/pan_motion_gate/state','/body_guard_3d/state']:
   self.create_subscription(String,topic,lambda m,t=topic:self.diagnostic(m,t),qos_profile_sensor_data)
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
 moving=speed>.01;transitions=int(np.sum(moving[1:]&~moving[:-1]))
 def reversals(c,dead):
  s=np.sign(c[np.abs(c)>dead]);return int(np.sum(s[1:]!=s[:-1]))
 return {'available':True,'linear_jerk_rms_m_s3':float(np.sqrt(np.mean(linear**2))),'linear_jerk_p95_m_s3':float(np.percentile(linear,95)),'angular_jerk_rms_rad_s3':float(np.sqrt(np.mean(angular**2))),'angular_jerk_p95_rad_s3':float(np.percentile(angular,95)),'stationary_s':float(np.sum(~moving)*.05),'motion_start_count':transitions,'yaw_reversals':reversals(v[:,2],.02),'lateral_reversals':reversals(v[:,1],.01),'speed_mean_m_s':float(np.mean(speed)),'speed_max_m_s':float(np.max(speed))}

def run(out,candidate,goal_index,rtf,domain):
 out.mkdir(parents=True,exist_ok=False);os.environ.update(ROS_DOMAIN_ID=str(domain),ROS_LOCALHOST_ONLY='1',IGN_IP='127.0.0.1',GZ_IP='127.0.0.1')
 partition='cleany_comparison_'+uuid.uuid4().hex;os.environ.update(IGN_PARTITION=partition,GZ_PARTITION=partition)
 result={'candidate':candidate,'goal_index':goal_index,'goal_name':GOALS[goal_index][0],'goal_map':GOALS[goal_index][1],'completed':False,'domain':domain,'partition':partition,'host':os.uname().nodename};proc=node=bag=None;resource_samples=[]
 started=time.monotonic()
 try:
  scenario=setup(out,candidate,rtf);result['model_metadata']=scenario['model_metadata']
  proc=subprocess.Popen(['ros2','launch',str(out/'run.launch.py')],stdout=(out/'launch.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
  (out/'session.json').write_text(json.dumps({'launch_pid':proc.pid,'supervisor_pid':os.getpid(),'partition':partition,'domain':domain}))
  rclpy.init();node=Probe(out,scenario);node.initialize();node.wait(2)
  f=yaml.safe_load((out/'config/comparison_motion.yaml').read_text())['controller_server']['ros__parameters']['FollowPath']
  keys=['plugin','wrapped_plugin'] if 'wrapped_plugin' in f else ['plugin']
  result['verified_controller']=node.parameters('/controller_server',['FollowPath.'+k for k in keys])
  result['verified_planner']=node.parameters('/planner_server',['GridBased.plugin'])
  result['rpp_limit_note']=scenario.get('rpp_limit_note')
  bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'bag'),'/clock','/ground_truth/odom','/odom','/wheel/odom','/joint_states','/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel','/pan_motion_gate/state','/scan','/collision_monitor_state','/tf','/tf_static','/head_pan/state','/body_guard_3d/state','/evaluation/depth_points','/amcl_pose','/plan','/local_costmap/costmap','/global_costmap/costmap'],stdout=(out/'bag.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
  node.wait(1);goal=GOALS[goal_index][1];req=NavigateToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=node.get_clock().now().to_msg();req.pose.pose.position.x=goal[0];req.pose.pose.position.y=goal[1];req.pose.pose.orientation.z=math.sin(goal[2]/2);req.pose.pose.orientation.w=math.cos(goal[2]/2)
  if not node.nav.wait_for_server(timeout_sec=20):raise TimeoutError('No navigate_to_pose action server')
  handle=node.future(node.nav.send_goal_async(req));node.active_goal=handle
  if not handle.accepted:raise RuntimeError('Goal rejected')
  future=handle.get_result_async();sim_start=node.now();wall_start=time.monotonic();node.event('comparison_navigation',goal=goal);last_resource=0
  while not future.done():
   node.tick()
   if time.monotonic()-last_resource>=1:
    resource_samples.append({'wall_s':time.monotonic()-wall_start,'sim_s':node.now(),'processes':resources(proc.pid)});last_resource=time.monotonic()
   if proc.poll() is not None:raise RuntimeError('Launch exited')
   if node.now()-sim_start>240 or time.monotonic()-wall_start>360:
    node.future(handle.cancel_goal_async());node.future(future,timeout=10);result['timeout']=True;break
  action=future.result();node.active_goal=None;sim_end=node.now();wall_end=time.monotonic();node.wait(2)
  truth=node.truth();err=math.dist(truth[:2],goal[:2]);yawerr=abs(math.atan2(math.sin(truth[2]-goal[2]),math.cos(truth[2]-goal[2])))
  result.update(completed=True,action_status=action.status,position_error_m=err,yaw_error_deg=math.degrees(yawerr),final_truth=truth,navigation_sim_s=sim_end-sim_start,navigation_wall_s=wall_end-wall_start,achieved_rtf=(sim_end-sim_start)/(wall_end-wall_start),smoothness=summarize_velocity(node.vel,sim_start,sim_end),navigation_sim_start_s=sim_start,navigation_sim_end_s=sim_end)
  active=[c for c in node.commands if sim_start<=c[0]<=sim_end]
  result['max_command_wheel_rad_s']={topic:max((max(map(abs,wheel_rates(c[3:]))) for c in active if c[2]==topic),default=None) for topic in ['/nav2/cmd_vel','/cmd_vel']}
  result['command_sim_interval_s']={}
  for topic in ['/nav2/cmd_vel','/pan_ready/cmd_vel','/safety_2d/cmd_vel','/cmd_vel']:
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
 finally:
  if node is not None:
   try:node.finish();node.save()
   except Exception as e:result['cleanup_error']=str(e)
   for name,rows in [('velocity',node.vel),('commands',node.commands)]:
    with (out/(name+'.csv')).open('w') as f:csv.writer(f).writerows(rows)
   (out/'joints.json').write_text(json.dumps(node.joints));node.destroy_node()
   (out/'diagnostics.json').write_text(json.dumps(node.diagnostics))
   with (out/'sensor_freshness.csv').open('w') as f:csv.writer(f).writerows(node.sensor_samples)
  if rclpy.ok():rclpy.shutdown()
  stop(bag);stop(proc)
  (out/'resources.json').write_text(json.dumps(resource_samples))
  result['elapsed_wall_s']=time.monotonic()-started
  log=(out/'launch.log').read_text(errors='replace') if (out/'launch.log').exists() else ''
  result['controller_deadline_warning_count']=log.count('Control loop missed its desired rate')
  (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--candidate',choices=['dwb','mppi','teb','rpp'],required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--domain',type=int,default=221);p.add_argument('--rtf',type=float,default=1);p.add_argument('--goal-index',type=int);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);describe(a.output)
 for i in ([a.goal_index] if a.goal_index is not None else range(5)):
  r=run(a.output/(str(i)+'_'+GOALS[i][0]),a.candidate,i,a.rtf,a.domain)
  if not r['completed'] and 'TimeoutError' in r.get('error','') and not r.get('navigation_sim_start_s'):
   r=run(a.output/(str(i)+'_'+GOALS[i][0]+'_startup_retry'),a.candidate,i,a.rtf,a.domain)
