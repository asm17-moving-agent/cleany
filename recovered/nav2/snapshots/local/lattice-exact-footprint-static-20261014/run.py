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
plugins={'SmacLattice':dict(common,plugin='nav2_smac_planner/SmacPlannerLattice',smooth_path=False,allow_reverse_expansion=True,max_planning_time=80.,max_iterations=-1,cache_obstacle_heuristic=False,lattice_filepath=primitive,cost_penalty=2.)}

config=yaml.safe_load((S/'preflight_planner.yaml').read_text());config['map_server']['ros__parameters']['yaml_filename']=str(MAP);config['global_costmap']['global_costmap']['ros__parameters'].update(footprint='[[-0.238525,-0.32737300000024483],[-0.238525,0.32742700000024483],[0.238475,0.32742700000024483],[0.238475,-0.32737300000024483]]',footprint_padding=0.0);cfg=config['planner_server']['ros__parameters'];cfg.pop('GridBased',None);cfg['planner_plugins']=list(plugins);cfg.update(plugins)
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
os.environ.update(ROS_DOMAIN_ID='229',ROS_LOCALHOST_ONLY='1')
rows=[]
for name,settings in plugins.items():
 config['planner_server']['ros__parameters']['planner_plugins']=[name]
 for k in plugins:config['planner_server']['ros__parameters'].pop(k,None)
 config['planner_server']['ros__parameters'][name]=settings
 params_file=O/(name+'.yaml');params_file.write_text(yaml.safe_dump(config))
 launch_file=O/(name+'.launch.py');launch_file.write_text(launch.replace('params.yaml',params_file.name))
 proc=subprocess.Popen(['ros2','launch',str(launch_file)],stdout=(O/(name+'.log')).open('w'),stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();node=Probe()
 try:
  for server in ['map_server','planner_server']:
   for transition in [1,3]:
    req=ChangeState.Request();req.transition.id=transition;assert node.service('/'+server+'/change_state',ChangeState,req).success
  assert node.client.wait_for_server(timeout_sec=20)
  end=time.monotonic()+1
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
  req=GetParameters.Request();req.names=[name+'.plugin'];actual=node.service('/planner_server/get_parameters',GetParameters,req);actual={req.names[0]:parameter_value_to_python(actual.values[0])}
  g=ComputePathToPose.Goal();g.start=pose(START);g.goal=pose(GOAL);g.use_start=True;g.planner_id=name
  begin=time.monotonic();handle=node.wait(node.client.send_goal_async(g));assert handle.accepted;reply=node.wait(handle.get_result_async(),95);elapsed=time.monotonic()-begin;points=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in reply.result.path.poses]
  row={'planner':name,'status':reply.status,'wall_s':elapsed,'path_length_m':sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])) if points else None,'path':points,'actual_plugin':actual,'configured_parameters':settings,'goal_endpoint_error_m':math.dist(points[-1][:2],GOAL[:2]) if points else None,'error':None}
 except Exception as e:row={'planner':name,'status':None,'wall_s':None,'path_length_m':None,'path':[],'error':repr(e),'configured_parameters':settings}
 finally:
  node.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
 rows.append(row);(O/'results.json').write_text(json.dumps({'start':START,'goal':GOAL,'map':str(MAP),'map_sha256':hashlib.sha256((MAP.parent/'map.pgm').read_bytes()).hexdigest(),'scope':'static map only; exact physical collision XY bounds; padding0; stock5cm omni primitives; max_iterations=-1; max_planning_time80s; same home/other-room goal with destination chair removed','results':rows},indent=2));print(json.dumps({k:v for k,v in row.items() if k not in ['path','actual_plugin','configured_parameters']}),flush=True)
