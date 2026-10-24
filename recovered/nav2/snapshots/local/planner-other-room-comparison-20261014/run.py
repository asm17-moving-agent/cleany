from pathlib import Path
import os,json,math,time,subprocess,signal,hashlib
import yaml,rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
O=Path(__file__).resolve().parent
C=O.parent/'controller-comparison-8goals-20261014/common'
S=O.parent/'standard-planner-20261014/preflight'
META=json.loads((C/'goals.locked.json').read_text());START=META['home_world_map'];GOAL=META['goals'][7]['pose_map_world']
MAP=S/'goals/7_other_room/map.yaml'
primitive='/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json'
common={'tolerance':.05,'allow_unknown':False}
plugins={
'StraightFirst':dict(common,plugin='cleany_navigation::StraightFirstPlanner',holonomic_direct_path=True,require_exact_goal=True,smooth_path=False,allow_reverse_expansion=True,max_planning_time=5.,cache_obstacle_heuristic=False,lattice_filepath=primitive,analytic_expansion_max_length=0.,cost_penalty=5.,lattice_collision_padding=.05,body_heading_grid_bins=32),
'NavFn':dict(common,plugin='nav2_navfn_planner/NavfnPlanner',use_astar=False),
'Smac2D':dict(common,plugin='nav2_smac_planner/SmacPlanner2D',max_planning_time=5.,cost_travel_multiplier=2.,use_final_approach_orientation=False),
'ThetaStar':dict(common,plugin='nav2_theta_star_planner/ThetaStarPlanner'),
'SmacLattice':dict(common,plugin='nav2_smac_planner/SmacPlannerLattice',smooth_path=False,allow_reverse_expansion=True,max_planning_time=5.,cache_obstacle_heuristic=False,lattice_filepath=primitive,cost_penalty=2.),
'SmacHybrid':dict(common,plugin='nav2_smac_planner/SmacPlannerHybrid',motion_model_for_search='REEDS_SHEPP',minimum_turning_radius=.5,angle_quantization_bins=72,max_planning_time=5.,smooth_path=False,cache_obstacle_heuristic=False)
}
config=yaml.safe_load((S/'preflight_planner.yaml').read_text());config['map_server']['ros__parameters']['yaml_filename']=str(MAP);cfg=config['planner_server']['ros__parameters'];cfg.pop('GridBased',None);cfg['planner_plugins']=list(plugins);cfg.update(plugins)
(O/'params.yaml').write_text(yaml.safe_dump(config))
launch=(C/'preflight.launch.py').read_text().replace('preflight_planner.yaml','params.yaml');(O/'launch.py').write_text(launch)
def pose(values):
 p=PoseStamped();p.header.frame_id='map';p.pose.position.x=values[0];p.pose.position.y=values[1];p.pose.orientation.z=math.sin(values[2]/2);p.pose.orientation.w=math.cos(values[2]/2);return p
class Probe(Node):
 def __init__(self):super().__init__('planner_six_probe');self.client=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
 def wait(self,f,seconds=40):
  end=time.monotonic()+seconds
  while not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)
  if not f.done():raise TimeoutError('future')
  return f.result()
 def service(self,name,typ,request):
  c=self.create_client(typ,name)
  if not c.wait_for_service(timeout_sec=30):raise TimeoutError(name)
  result=self.wait(c.call_async(request));self.destroy_client(c);return result
os.environ.update(ROS_DOMAIN_ID='230',ROS_LOCALHOST_ONLY='1')
proc=subprocess.Popen(['ros2','launch',str(O/'launch.py')],stdout=(O/'planner.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True);rclpy.init();node=Probe();rows=[]
try:
 for name in ['map_server','planner_server']:
  for transition in [1,3]:
   r=ChangeState.Request();r.transition.id=transition;assert node.service('/'+name+'/change_state',ChangeState,r).success
 assert node.client.wait_for_server(timeout_sec=20)
 time.sleep(1)
 for name,settings in plugins.items():
  g=ComputePathToPose.Goal();g.start=pose(START);g.goal=pose(GOAL);g.use_start=True;g.planner_id=name
  req=GetParameters.Request();req.names=[name+'.'+k for k in settings];params=node.service('/planner_server/get_parameters',GetParameters,req);actual=dict(zip(req.names,[parameter_value_to_python(p) for p in params.values]))
  begin=time.monotonic();handle=node.wait(node.client.send_goal_async(g));assert handle.accepted;reply=node.wait(handle.get_result_async(),40);elapsed=time.monotonic()-begin;points=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in reply.result.path.poses]
  row={'planner':name,'status':reply.status,'wall_s':elapsed,'path_length_m':sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])) if points else None,'path':points,'actual_parameters':actual,'goal_endpoint_error_m':math.dist(points[-1][:2],GOAL[:2]) if points else None};rows.append(row);(O/'results.json').write_text(json.dumps({'start':START,'goal':GOAL,'map':str(MAP),'map_sha256':hashlib.sha256((MAP.parent/'map.pgm').read_bytes()).hexdigest(),'scope':'static map planning only, no driving;5cm same map/inflation/polygon; 2D planners do not use orientation-aware polygon checks;5s budgets for custom/Smac; NavFn/Theta native termination','results':rows},indent=2));print(json.dumps({k:v for k,v in row.items() if k not in ['path','actual_parameters']}),flush=True)
finally:
 node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
