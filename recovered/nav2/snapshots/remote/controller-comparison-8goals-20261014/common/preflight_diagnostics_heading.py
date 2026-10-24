from pathlib import Path
import os,json,math,time,subprocess,signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import LoadMap,ClearEntireCostmap
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from prepare_common import ROOT,FOOT,PADDING

def pose(p):
 x=PoseStamped();x.header.frame_id='map';x.pose.position.x=p[0];x.pose.position.y=p[1];x.pose.orientation.z=math.sin(p[2]/2);x.pose.orientation.w=math.cos(p[2]/2);return x
class Probe(Node):
 def __init__(self):super().__init__('common_preflight');self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
 def future(self,f,secs=60):
  limit=time.monotonic()+secs
  while not f.done() and time.monotonic()<limit:rclpy.spin_once(self,timeout_sec=.05)
  if not f.done():raise TimeoutError('future')
  return f.result()
 def call(self,t,name,req):
  c=self.create_client(t,name)
  if not c.wait_for_service(timeout_sec=30):raise TimeoutError(name)
  out=self.future(c.call_async(req));self.destroy_client(c);return out
 def wait(self,s):
  limit=time.monotonic()+s
  while time.monotonic()<limit:rclpy.spin_once(self,timeout_sec=.05)
 def params(self,node,names):
  req=GetParameters.Request();req.names=names;return dict(zip(names,[parameter_value_to_python(p) for p in self.call(GetParameters,node+'/get_parameters',req).values]))
def main():
 os.environ.update(ROS_DOMAIN_ID='225',ROS_LOCALHOST_ONLY='1');log=(ROOT/'preflight_diagnostics_heading.log').open('w');proc=subprocess.Popen(['ros2','launch',str(ROOT/'preflight.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);(ROOT/'preflight_diagnostics_heading_session.json').write_text(json.dumps({'launch_pid':proc.pid,'supervisor_pid':os.getpid(),'domain':225}));rclpy.init();n=Probe();static=json.loads((ROOT/'preflight.static.json').read_text());proposed=json.loads((ROOT/'plan.proposed.json').read_text());results=[]
 try:
  for name in ['map_server','planner_server']:
   for tr in [1,3]:
    req=ChangeState.Request();req.transition.id=tr;assert n.call(ChangeState,'/'+name+'/change_state',req).success
  assert n.planner.wait_for_server(timeout_sec=30);n.wait(2)
  live=n.params('/global_costmap/global_costmap',['footprint','footprint_padding','inflation_layer.inflation_radius','inflation_layer.cost_scaling_factor']);live['planner']=n.params('/planner_server',['GridBased.plugin','GridBased.holonomic_direct_path','GridBased.require_exact_goal','GridBased.lattice_collision_padding'])
  for g in [{**static['goals'][-1],'index':7,'id':'diag_M1_heading0','pose_map_world':[-16.775,1.775,0.]}]:
   req=LoadMap.Request();req.map_url=str(ROOT/g['map']);resp=n.call(LoadMap,'/map_server/load_map',req);assert resp.result==0;n.call(ClearEntireCostmap,'/global_costmap/clear_entirely_global_costmap',ClearEntireCostmap.Request());n.wait(1)
   request=ComputePathToPose.Goal();request.goal=pose(g['pose_map_world']);request.start=pose(proposed['home_world_map']);request.use_start=True;request.planner_id='GridBased';begin=time.monotonic();handle=n.future(n.planner.send_goal_async(request));assert handle.accepted;action=n.future(handle.get_result_async(),90);path=action.result.path;points=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in path.poses];length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]));yawchange=sum(abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))) for a,b in zip(points,points[1:]));(ROOT/'diagnostics'/f"{g['index']}_{g['id']}.path.json").write_text(json.dumps(points,indent=2));result={'index':g['index'],'id':g['id'],'action_status':action.status,'pose_count':len(points),'wall_s':time.monotonic()-begin,'path_length_m':length,'total_path_yaw_change_rad':yawchange,'start':points[:1],'end':points[-1:],'preflight_layers':'static LiDAR-plane map+same inflation+full footprint; dynamic depth/memory not asserted','live':live};results.append(result);(ROOT/'preflight.diagnostics_heading.json').write_text(json.dumps(results,indent=2));print(json.dumps(result),flush=True)
 finally:
  n.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
if __name__=='__main__':main()
