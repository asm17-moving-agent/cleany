from pathlib import Path
import os,json,math,time,subprocess,signal,csv,argparse,hashlib
from collections import Counter
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import LoadMap,ClearEntireCostmap,GetCostmap
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
R=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--zones',nargs='+',default=['D-hub']);args=parser.parse_args()
def pose(p):
 m=PoseStamped();m.header.frame_id='map';m.pose.position.x=p[0];m.pose.position.y=p[1];m.pose.orientation.z=math.sin(p[2]/2);m.pose.orientation.w=math.cos(p[2]/2);return m
class Probe(Node):
 def __init__(self):super().__init__('facility_seat_static_probe');self.planner=ActionClient(self,ComputePathToPose,'compute_path_to_pose')
 def future(self,f,sec=30):
  end=time.monotonic()+sec
  while not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.04)
  if not f.done():raise TimeoutError('future')
  return f.result()
 def call(self,t,name,req):
  c=self.create_client(t,name)
  try:
   if not c.wait_for_service(timeout_sec=20):raise TimeoutError(name)
   return self.future(c.call_async(req))
  finally:self.destroy_client(c)
 def wait(self,sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.04)
 def params(self,node,names):
  req=GetParameters.Request();req.names=names;return dict(zip(names,[parameter_value_to_python(x) for x in self.call(GetParameters,node+'/get_parameters',req).values]))
 def costmap(self):return self.call(GetCostmap,'/global_costmap/get_costmap',GetCostmap.Request()).map

def save(records):
 (R/'results.json').write_text(json.dumps(records,indent=2))
 keys=['seat','zone','action_status','path_generated','pose_count','path_length_m','planned_yaw_variation_deg','planning_s','wall_s','goal_xy_error_m','goal_yaw_error_deg','removed_chair','error']
 with (R/'results.csv').open('w') as f:w=csv.DictWriter(f,keys,extrasaction='ignore');w.writeheader();w.writerows(records)
 (R/'progress.json').write_text(json.dumps({'completed':len(records),'by_zone':dict(Counter(x['zone'] for x in records)),'generated':sum(x.get('path_generated',False) for x in records),'last':records[-1]['seat'] if records else None},indent=2))
def main():
 os.environ.update(ROS_DOMAIN_ID='193',ROS_LOCALHOST_ONLY='1');suffix='-'.join(args.zones);logpath=R/f'planner-{suffix}.log';log=logpath.open('w');proc=subprocess.Popen(['ros2','launch',str(R/'planner.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (R/'active-session.json').write_text(json.dumps({'pid':proc.pid,'domain':193,'zones':args.zones}));rclpy.init();n=Probe();locked=json.loads((R/'goals.json').read_text());records=json.loads((R/'results.json').read_text()) if (R/'results.json').exists() else [];done={x['seat'] for x in records};lastmap=None
 try:
  for name in ['map_server','planner_server']:
   for t in [1,3]:
    req=ChangeState.Request();req.transition.id=t;assert n.call(ChangeState,'/'+name+'/change_state',req).success
  assert n.planner.wait_for_server(timeout_sec=30)
  live={'planner':n.params('/planner_server',['GridBased.plugin', 'GridBased.allow_unknown', 'GridBased.w_traversal_cost', 'GridBased.how_many_corners']),'costmap':n.params('/global_costmap/global_costmap',['footprint','footprint_padding','resolution','inflation_layer.inflation_radius','inflation_layer.cost_scaling_factor'])};(R/'verified-runtime.json').write_text(json.dumps(live,indent=2));assert live['planner']['GridBased.plugin']=='nav2_theta_star_planner/ThetaStarPlanner';assert live['costmap']['footprint_padding']==0.
  for goal in locked['goals']:
   if goal['zone'] not in args.zones or goal['seat'] in done:continue
   folder=R/'paths'/goal['seat'];folder.mkdir(parents=True,exist_ok=True)
   if goal['map']!=lastmap:
    req=LoadMap.Request();req.map_url=goal['map'];assert n.call(LoadMap,'/map_server/load_map',req).result==0
    n.call(ClearEntireCostmap,'/global_costmap/clear_entirely_global_costmap',ClearEntireCostmap.Request());target=np.array(Image.open(Path(goal['map']).with_name('map.pgm')))[::-1]==0
    deadline=time.monotonic()+15;good=0
    while time.monotonic()<deadline:
     n.wait(.25);cm=n.costmap();a=np.array(cm.data,dtype=np.uint8).reshape(cm.metadata.size_y,cm.metadata.size_x)
     good=good+1 if a.shape==target.shape and np.array_equal(a==254,target) else 0
     if good>=2:break
    assert good>=2,'Native static costmap did not match target-chair map'
    lastmap=goal['map'];a.tofile(folder/'costmap.bin');(folder/'costmap-metadata.json').write_text(json.dumps({'size':[int(cm.metadata.size_x),int(cm.metadata.size_y)],'resolution':cm.metadata.resolution,'origin':[cm.metadata.origin.position.x,cm.metadata.origin.position.y],'costmap_lethal_mask_verified':True,'map_sha256':goal['map_sha256']},indent=2))
   record={**goal,'start':locked['home'],'action_status':None,'path_generated':False,'pose_count':0,'path_length_m':None,'planned_yaw_variation_deg':None,'planning_s':None,'wall_s':None,'goal_xy_error_m':None,'goal_yaw_error_deg':None,'error':None}
   req=ComputePathToPose.Goal();req.start=pose(locked['home']);req.goal=pose(goal['goal']);req.use_start=True;req.planner_id='GridBased';begin=time.monotonic();logoffset=logpath.stat().st_size
   try:
    handle=n.future(n.planner.send_goal_async(req));assert handle.accepted
    result=n.future(handle.get_result_async(),95);record['action_status']=result.status;p=[[z.pose.position.x,z.pose.position.y,2*math.atan2(z.pose.orientation.z,z.pose.orientation.w)] for z in result.result.path.poses];record['pose_count']=len(p);record['path_generated']=result.status==4 and len(p)>0;record['planning_s']=result.result.planning_time.sec+result.result.planning_time.nanosec*1e-9
    if p:
     record.update(path_length_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(p,p[1:])),planned_yaw_variation_deg=sum(abs(math.degrees(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))) for a,b in zip(p,p[1:])),goal_xy_error_m=math.dist(p[-1][:2],goal['goal'][:2]),goal_yaw_error_deg=abs(math.degrees(math.atan2(math.sin(p[-1][2]-goal['goal'][2]),math.cos(p[-1][2]-goal['goal'][2])))))
    (folder/'path.json').write_text(json.dumps(p,indent=2))
   except Exception as e:record['error']=repr(e)
   record['wall_s']=time.monotonic()-begin
   with logpath.open() as f:f.seek(logoffset);excerpt=f.read()
   (folder/'planner.log').write_text(excerpt);record['planner_diagnostics']=[line for line in excerpt.splitlines() if any(k in line.lower() for k in ['fail','occupied','timeout','no valid','exceed','error'])][-12:]
   (folder/'result.json').write_text(json.dumps(record,indent=2));records.append(record);save(records);print(json.dumps({k:record[k] for k in ['seat','zone','action_status','path_generated','planning_s','wall_s','error']}),flush=True)
 finally:
  n.destroy_node();rclpy.shutdown();os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=12)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
  (R/f'completed-{suffix}.json').write_text(json.dumps({'pid':proc.pid,'returncode':proc.returncode,'zones':args.zones,'completed':len(records),'simulator_started':False}))
if __name__=='__main__':main()
