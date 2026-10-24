import json,math,time,sys,pathlib
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from nav2_msgs.action import NavigateToPose
p=pathlib.Path(sys.argv[1]);rclpy.init();n=Node('gui_chair_gap_operator',parameter_overrides=[])
n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
seen={}
rows=[]
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def gt_sample(m):
 seen['gt']=m
 pos=m.pose.pose
 rows.append({'sim_s':n.get_clock().now().nanoseconds/1e9,'x':pos.position.x,'y':pos.position.y,'yaw':yaw(pos.orientation)})
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:seen.update(amcl=m),10)
n.create_subscription(Odometry,'/ground_truth/odom',gt_sample,qos_profile_sensor_data)
n.create_subscription(LaserScan,'/scan',lambda m:seen.update(scan=m),qos_profile_sensor_data)
init=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
def wait(f,seconds=120):
 end=time.monotonic()+seconds
 while not f():
  rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()>end:raise TimeoutError('readiness timeout')
def call(typ,path,request,seconds=15):
 cli=n.create_client(typ,path)
 if not cli.wait_for_service(timeout_sec=seconds):raise TimeoutError(path)
 f=cli.call_async(request);wait(f.done,seconds);ans=f.result();n.destroy_client(cli);return ans
wait(lambda:all(k in seen for k in ['gt','scan']))
pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=n.get_clock().now().to_msg();pose.pose.pose.position.x=-1.865;pose.pose.pose.position.y=-4.705;pose.pose.pose.orientation.z=math.sin(1.5708/2);pose.pose.pose.orientation.w=math.cos(1.5708/2)
pose.pose.covariance[0]=.0625;pose.pose.covariance[7]=.0625;pose.pose.covariance[35]=math.radians(15)**2
init.publish(pose)
for _ in range(5):
 call(Empty,'/request_nomotion_update',Empty.Request());end=time.monotonic()+1;wait(lambda:time.monotonic()>=end)
names=['map_server','amcl','controller_server','planner_server','behavior_server','bt_navigator']
for name in names:
 wait(lambda:call(GetState,'/'+name+'/get_state',GetState.Request()).current_state.label=='active')
(p/'READY').write_text('Nav2 active; ground truth and scan fresh; initial pose set\n');print('READY',p,flush=True)
wait(lambda:(p.parent/'START').exists(),86400)
goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg();goal.pose.pose.position.x=.575;goal.pose.pose.position.y=-1.625;goal.pose.pose.orientation.z=math.sin(1.570796/2);goal.pose.pose.orientation.w=math.cos(1.570796/2)
start_wall=time.monotonic();start_sim=n.get_clock().now().nanoseconds/1e9
client=ActionClient(n,NavigateToPose,'/navigate_to_pose')
if not client.wait_for_server(timeout_sec=15):raise TimeoutError('navigate_to_pose')
f=client.send_goal_async(goal);wait(f.done);handle=f.result();(p/'goal_accepted.json').write_text(json.dumps({'accepted':handle.accepted,'goal':[.575,-1.625,1.570796]}));print('GOAL ACCEPTED',handle.accepted,flush=True)
if not handle.accepted:raise RuntimeError('goal rejected')
f=handle.get_result_async();wait(f.done,1800);answer=f.result();gt=seen['gt'].pose.pose;xy=[gt.position.x,gt.position.y]
(p/'live_result.json').write_text(json.dumps({'action_status':answer.status,'last_gt_xy':xy,'gt_yaw':yaw(gt.orientation),'goal_world':[.575,-1.625,1.570796],'gt_xy_error_m':math.hypot(xy[0]-.575,xy[1]+1.625),'gt_yaw_error_deg':math.degrees(abs(math.atan2(math.sin(yaw(gt.orientation)-1.570796),math.cos(yaw(gt.orientation)-1.570796)))),'wall_s':time.monotonic()-start_wall,'sim_s':n.get_clock().now().nanoseconds/1e9-start_sim,'amcl_pose':([seen['amcl'].pose.pose.position.x,seen['amcl'].pose.pose.position.y,yaw(seen['amcl'].pose.pose.orientation)] if 'amcl' in seen else None),'note':'Laptop facility world seat-29 chair removed; one visual comparison run'},indent=2));print('RESULT',answer.status,xy,flush=True)
(p/'trajectory.json').write_text(json.dumps(rows))
while rclpy.ok():rclpy.spin_once(n,timeout_sec=.2)
