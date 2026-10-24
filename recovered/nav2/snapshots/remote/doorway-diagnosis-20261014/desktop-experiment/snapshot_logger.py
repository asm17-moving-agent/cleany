"""Read-only exact costmap/TF snapshots; no commands or localization injection."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as NavPath,Odometry
from tf2_ros import Buffer,TransformListener

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose(p):return [p.position.x,p.position.y,yaw(p.orientation)]
class Logger(Node):
 def __init__(self,out):
  super().__init__('doorway_snapshot',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.out=out;out.mkdir(exist_ok=True,parents=True);self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
  self.plan=[];self.gt=None;self.pending={};self.last=-10
  self.map_clients={k:self.create_client(GetCostmap,f'/{k}_costmap/get_costmap') for k in ['local','global']}
  self.create_subscription(NavPath,'/plan',self.receive_plan,qos_profile_sensor_data)
  self.create_subscription(Odometry,'/ground_truth/odom',lambda m:setattr(self,'gt',pose(m.pose.pose)),qos_profile_sensor_data)
  self.create_timer(.2,self.tick)
 def receive_plan(self,m):self.plan=[pose(p.pose) for p in m.poses]
 def tick(self):
  now=self.get_clock().now().nanoseconds*1e-9
  if self.pending:
   if not all(f.done() for f in self.pending.values()):return
   d={'sim_s':now,'gt':self.gt,'plan_map':self.plan};arrays={}
   for k,f in self.pending.items():
    m=f.result().map;info=m.metadata
    arrays[k]=np.asarray(m.data,dtype=np.uint8).reshape(info.size_y,info.size_x)
    d[k]={'frame':m.header.frame_id,'resolution':info.resolution,'origin':pose(info.origin),'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9}
   for frame in ['map','odom']:
    try:
     tf=self.buffer.lookup_transform(frame,'base_link',Time()).transform
     d['robot_'+frame]=[tf.translation.x,tf.translation.y,yaw(tf.rotation)]
    except Exception:pass
   tag=f'{now:09.3f}';np.savez_compressed(self.out/(tag+'.npz'),**arrays);(self.out/(tag+'.json')).write_text(json.dumps(d))
   self.pending={}
  if now-self.last>=5 and all(c.service_is_ready() for c in self.map_clients.values()):
   self.last=now;self.pending={k:c.call_async(GetCostmap.Request()) for k,c in self.map_clients.items()}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args();rclpy.init();n=Logger(a.output)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
