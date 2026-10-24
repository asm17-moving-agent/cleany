import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
out=Path(__file__).parent/'amcl-continuous-restored';out.mkdir(exist_ok=True)
rclpy.init();n=Node('amcl_continuous_correction');state={'sim_s':None};poses=[]
def pose_cb(m):
 row={'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'qz':m.pose.pose.orientation.z,'qw':m.pose.pose.orientation.w,'covariance':list(m.pose.covariance)};poses.append(row);(out/'latest-pose.json').write_text(json.dumps(row,indent=2))
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',pose_cb,10);n.create_subscription(Clock,'/clock',lambda m:state.update(sim_s=m.clock.sec+m.clock.nanosec/1e9),10)
c=n.create_client(SetParameters,'/amcl/set_parameters');assert c.wait_for_service(timeout_sec=5)
req=SetParameters.Request();req.parameters=[Parameter(name=k,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=.01)) for k in ['update_min_d','update_min_a']]
f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);reply=[{'successful':x.successful,'reason':x.reason} for x in f.result().results];print('AMCL thresholds',reply,flush=True);assert all(x['successful'] for x in reply)
c=n.create_client(Empty,'/request_nomotion_update');assert c.wait_for_service(timeout_sec=5)
last=-1;pending=None;count=0;start=time.monotonic()
try:
 while time.monotonic()-start<1200:
  rclpy.spin_once(n,timeout_sec=.1)
  if state['sim_s'] is not None and state['sim_s']-last>=1 and (pending is None or pending.done()):
   pending=c.call_async(Empty.Request());last=state['sim_s'];count+=1
   (out/'status.json').write_text(json.dumps({'sim_s':last,'requests':count,'pose_updates':len(poses),'latest_pose':poses[-1] if poses else None},indent=2))
except KeyboardInterrupt:pass
finally:
 (out/'poses.json').write_text(json.dumps(poses));n.destroy_node();rclpy.shutdown()
