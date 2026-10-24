from pathlib import Path
import rclpy,json,time,math
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import MarkerArray
rclpy.init();n=Node('capture_mppi_actual_path');O=Path(__file__).parent/'stall-diagnosis';rows={}
def path(m):rows['transformed_path']={'frame':m.header.frame_id,'path':[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in m.poses]}
def markers(m):
 rows['optimal']=[[x.pose.position.x,x.pose.position.y] for x in m.markers if x.ns=='Optimal Trajectory'];rows['candidate_marker_count']=sum(x.ns=='Candidate Trajectories' for x in m.markers)
n.create_subscription(NavPath,'/transformed_global_plan',path,1);n.create_subscription(MarkerArray,'/trajectories',markers,1);c=n.create_client(SetParameters,'/controller_server/set_parameters');assert c.wait_for_service(timeout_sec=5)
def toggle(v):
 q=SetParameters.Request();q.parameters=[Parameter(name='FollowPath.visualize',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=v))];f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5);assert f.result().results[0].successful
try:
 toggle(True);start=time.monotonic()
 while time.monotonic()-start<8:rclpy.spin_once(n,timeout_sec=.1)
 (O/'actual-mppi-plan.json').write_text(json.dumps(rows,indent=2));print({k:len(v['path']) if isinstance(v,dict) else len(v) if isinstance(v,list) else v for k,v in rows.items()})
finally:toggle(False);n.destroy_node();rclpy.shutdown()
