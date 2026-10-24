from pathlib import Path
import json,time,math
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters,GetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from nav_msgs.msg import Odometry,Path as NavPath
from geometry_msgs.msg import Twist
O=Path(__file__).parent/'stall-diagnosis';O.mkdir(exist_ok=True);rclpy.init();n=Node('mppi_stall_diagnosis');samples=[];state={'phase':'baseline'};plans=[]
def gt(m):
 p=m.pose.pose;q=p.orientation;samples.append({'wall':time.monotonic(),'phase':state['phase'],'x':p.position.x,'y':p.position.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'sim_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9})
def plan(m):
 plans.append({'wall':time.monotonic(),'frame':m.header.frame_id,'poses':[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in m.poses]});(O/'live-plans.json').write_text(json.dumps(plans))
n.create_subscription(Odometry,'/ground_truth/odom',gt,10);n.create_subscription(NavPath,'/plan',plan,10)
c=n.create_client(SetParameters,'/controller_server/set_parameters');assert c.wait_for_service(timeout_sec=5)
def setparams(values):
 req=SetParameters.Request();req.parameters=[Parameter(name='FollowPath.'+k,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(v))) for k,v in values.items()];f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);reply=[{'successful':x.successful,'reason':x.reason} for x in f.result().results];assert all(x['successful'] for x in reply);print('set',values,reply,flush=True)
def stage(name,seconds,values=None):
 state['phase']=name
 if values:setparams(values)
 start=time.monotonic();i=len(samples)
 while time.monotonic()-start<seconds:rclpy.spin_once(n,timeout_sec=.1)
 rows=samples[i:];summary={'phase':name,'values':values,'wall_s':time.monotonic()-start,'sim_s':rows[-1]['sim_s']-rows[0]['sim_s'],'start':rows[0],'end':rows[-1],'displacement_m':math.hypot(rows[-1]['x']-rows[0]['x'],rows[-1]['y']-rows[0]['y'])};(O/(name+'.json')).write_text(json.dumps(summary,indent=2));(O/'twirl-samples.json').write_text(json.dumps(samples));print(json.dumps(summary),flush=True)
try:
 stage('twirl_baseline',5)
 stage('lower_rotation_penalty',35,{'TwirlingCritic.cost_weight':0.0})
 stage('restored_rotation_penalty',10,{'TwirlingCritic.cost_weight':10.0})
finally:
 setparams({'TwirlingCritic.cost_weight':10.0});(O/'twirl-samples.json').write_text(json.dumps(samples));n.destroy_node();rclpy.shutdown()
