from pathlib import Path
import json,time,math
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters,GetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from nav_msgs.msg import Odometry
from nav2_msgs.srv import GetCostmap
O=Path(__file__).parent/'stall-diagnosis';rclpy.init();n=Node('door_layer_diagnosis');samples=[];phase='baseline'
def gt(m):
 p=m.pose.pose;q=p.orientation;samples.append({'wall':time.monotonic(),'phase':phase,'x':p.position.x,'y':p.position.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'sim_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9})
n.create_subscription(Odometry,'/ground_truth/odom',gt,10)
def service(name,typ,req):
 c=n.create_client(typ,name);assert c.wait_for_service(timeout_sec=5);f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=6);return f.result()
def snapshot(label):
 reply=service('/local_costmap/get_costmap',GetCostmap,GetCostmap.Request()).map;m=reply.metadata;d={'frame':reply.header.frame_id,'stamp_s':reply.header.stamp.sec+reply.header.stamp.nanosec/1e9,'origin':[m.origin.position.x,m.origin.position.y],'resolution':m.resolution,'shape':[m.size_y,m.size_x],'data':list(reply.data)};(O/(label+'-costmap.json')).write_text(json.dumps(d))
def toggle(key,value):
 req=SetParameters.Request();req.parameters=[Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value))];res=service('/local_costmap/local_costmap/set_parameters',SetParameters,req);assert res.results[0].successful;print(key,value,flush=True)
def stage(label,sec):
 global phase;phase=label;i=len(samples);start=time.monotonic()
 while time.monotonic()-start<sec:rclpy.spin_once(n,timeout_sec=.1)
 a=samples[i:];d={'phase':label,'wall_s':time.monotonic()-start,'sim_s':a[-1]['sim_s']-a[0]['sim_s'],'start':a[0],'end':a[-1],'displacement_m':math.hypot(a[-1]['x']-a[0]['x'],a[-1]['y']-a[0]['y'])};(O/(label+'.json')).write_text(json.dumps(d,indent=2));print(json.dumps(d),flush=True);snapshot(label)
try:
 stage('depth_baseline',5);toggle('depth_layer.enabled',False);stage('depth_disabled',35);toggle('depth_layer.enabled',True);stage('depth_restored',8)
finally:
 toggle('depth_layer.enabled',True);(O/'depth-samples.json').write_text(json.dumps(samples));n.destroy_node();rclpy.shutdown()
