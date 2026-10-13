from pathlib import Path
import json,time,math,tempfile,os
import numpy as np,yaml,rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,PointCloud2,JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from cleany_navigation.point_cloud import xyz_cloud
from cleany_navigation.reactive_guard_node import ReactiveGuardNode
p=Path(os.environ['CLEANY_REACTIVE_EVIDENCE'])
profile=Path(os.environ['CLEANY_REACTIVE_PROFILE'])
params=dict(use_sim_time=True,profile=str(profile),allow_simulation_limits=True,depth_frame='base_link')
f=p/'fault-params.yaml';f.write_text(yaml.safe_dump({'reactive_guard':{'ros__parameters':params}}))
topics=['clock','tf','tf_static','nav2/cmd_vel','safety_2d/cmd_vel','cmd_vel',
        'odom','scan','camera/head/depth/points','joint_states','reactive_guard/state']
remaps=[arg for topic in topics for arg in ['-r',f'/{topic}:=/reactive_guard_fault_test/{topic}']]
rclpy.init(args=['--ros-args','--params-file',str(f),*remaps])
g=ReactiveGuardNode();n=rclpy.create_node('fault_injector');ex=SingleThreadedExecutor();ex.add_node(g);ex.add_node(n)
pubs={k:n.create_publisher(t,k,10) for k,t in [('clock',Clock),('nav2/cmd_vel',Twist),('safety_2d/cmd_vel',Twist),('odom',Odometry),('scan',LaserScan),('camera/head/depth/points',PointCloud2),('joint_states',JointState)]}
latest={};n.create_subscription(String,'reactive_guard/state',lambda m:latest.update(json.loads(m.data)),10)
tb=TransformBroadcaster(n);st=StaticTransformBroadcaster(n)
a=TransformStamped();a.header.frame_id='map';a.child_frame_id='odom';a.transform.rotation.w=1.;st.sendTransform(a)
begin=time.monotonic();sim=10.;results=[]
def run(name, duration=.85, *, no=(), frozen=False, obstacle=False, monitor_stop=False, raw_stop=False, bad_depth=False, missing_tf=False, reverse_clock=False):
 global sim
 end=time.monotonic()+duration;last_pub=0;states=[]
 while time.monotonic()<end:
  wall=time.monotonic()
  if wall-last_pub>.02:
   last_pub=wall
   if not frozen:sim=10+wall-begin-(5 if reverse_clock else 0)
   sec=int(sim);nano=int((sim-sec)*1e9)
   from builtin_interfaces.msg import Time
   stamp=Time(sec=sec,nanosec=nano)
   pubs['clock'].publish(Clock(clock=stamp))
   tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.header.stamp=stamp;tf.transform.rotation.w=1.;tb.sendTransform(tf)
   cmd=Twist();cmd.linear.x=.0866;cmd.linear.y=.05
   if 'raw' not in no:pubs['nav2/cmd_vel'].publish(Twist() if raw_stop else cmd)
   if 'monitor' not in no:pubs['safety_2d/cmd_vel'].publish(Twist() if monitor_stop else cmd)
   od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.orientation.w=1.
   if 'odom' not in no:pubs['odom'].publish(od)
   joint=JointState();joint.header.stamp=stamp;joint.name=['head_pan_joint'];joint.position=[-2.];joint.velocity=[1.]
   if 'joint' not in no:pubs['joint_states'].publish(joint)
   scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='missing_frame' if missing_tf else 'base_link';scan.angle_min=-math.pi;scan.angle_increment=.1;scan.range_min=.1;scan.range_max=10.;scan.ranges=[2.]*64
   if 'scan' not in no:pubs['scan'].publish(scan)
   point=[float('nan'),0.,0.] if bad_depth else ([float(g.guard.high[0]+.025),0.,0.] if obstacle else [2.,0.,0.])
   cloud=xyz_cloud(np.array([point]),'base_link',stamp)
   if 'depth' not in no:pubs['camera/head/depth/points'].publish(cloud)
  ex.spin_once(timeout_sec=.002)
  if wall>end-.15 and latest:states.append(dict(latest))
 assert states,name+' missing state'
 stopped=all(max(map(abs,s['output']))<1e-9 for s in states)
 moving=all(math.hypot(*s['output'][:2])>.05 for s in states)
 expect_stop=name!='clear_with_misaligned_pan' and not name.startswith('recover')
 assert stopped if expect_stop else moving,(name,states[-1])
 results.append(dict(case=name,pass_=True,last=states[-1],samples=len(states)))
 print(name,states[-1]['decision'],flush=True)
try:
 run('clear_with_misaligned_pan',1.5)
 for name,kw in [('monitor_stop',{'monitor_stop':True}),('raw_stop',{'raw_stop':True}),('obstacle',{'obstacle':True}),('missing_depth',{'no':('depth',)}),('missing_lidar',{'no':('scan',)}),('missing_odom',{'no':('odom',)}),('missing_joint',{'no':('joint',)}),('missing_monitor',{'no':('monitor',)}),('missing_raw',{'no':('raw',)}),('invalid_depth',{'bad_depth':True}),('missing_tf',{'missing_tf':True}),('frozen_clock',{'frozen':True})]:
  run(name,**kw);run('recover_'+name)
 run('clock_reset',reverse_clock=True)
 run('clock_reset_latched')
 (p/'fault-result.json').write_text(json.dumps(results,indent=2))
finally:
 ex.remove_node(g);ex.remove_node(n);g.destroy_node();n.destroy_node();rclpy.shutdown()
