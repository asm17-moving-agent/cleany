import json,math,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.serialization import serialize_message
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters
out=Path(sys.argv[1]);out.mkdir(exist_ok=True)
rclpy.init();node=Node('dwb_readonly_diagnostic');latest={}
for topic,kind in [('/ground_truth/odom',Odometry),('/odom',Odometry),('/local_costmap/costmap',OccupancyGrid),('/global_costmap/costmap',OccupancyGrid),('/plan',RosPath),('/amcl_pose',PoseWithCovarianceStamped)]:
 node.create_subscription(kind,topic,lambda m,t=topic:latest.__setitem__(t,m),qos_profile_sensor_data if kind is not OccupancyGrid else QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
start=time.monotonic()
while time.monotonic()-start<8:rclpy.spin_once(node,timeout_sec=.1)
report={'captured_wall_epoch_s':time.time(),'topics':{}}
for topic,m in latest.items():
 (out/(topic.replace('/','_')+'.cdr')).write_bytes(serialize_message(m));stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 d={'stamp_sim_s':stamp,'frame':m.header.frame_id}
 if hasattr(m,'pose'):
  p=m.pose.pose;q=p.orientation;d['pose']=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
 if hasattr(m,'info'):d.update(resolution=m.info.resolution,size=[m.info.width,m.info.height],origin=[m.info.origin.position.x,m.info.origin.position.y],lethal_cells=sum(v==100 for v in m.data),unknown_cells=sum(v<0 for v in m.data))
 report['topics'][topic]=d
(out/'snapshot.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));node.destroy_node();rclpy.shutdown()
