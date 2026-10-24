"""Configuration replay only: no Gazebo, activation or navigation goal."""
import json,os,signal,subprocess,time,hashlib
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import GetParameters
ROOT=Path(__file__).resolve().parent;out=ROOT/'parameter_replay';out.mkdir(exist_ok=True);original=ROOT/'final8/0_straight/config/relaxed_parameters.yaml';params=yaml.safe_load(original.read_text());params['controller_server']['ros__parameters']['FollowPath']['benchmark_timing_file']=str(out/'unused_compute.csv');(out/'parameters.yaml').write_text(yaml.safe_dump(params,sort_keys=False));os.environ.update(ROS_DOMAIN_ID='228',ROS_LOCALHOST_ONLY='1')
p=subprocess.Popen(['/opt/ros/humble/lib/nav2_controller/controller_server','--ros-args','--params-file',str(out/'parameters.yaml')],stdout=(out/'controller.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True);rclpy.init();node=Node('teb_configuration_replay');(out/'session.json').write_text(json.dumps({'launch_pid':p.pid,'domain':228,'purpose':'No simulation/activation/navigation, parameter replay only'}))
def call(kind,name,request):
 c=node.create_client(kind,name);assert c.wait_for_service(timeout_sec=20),name;f=c.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=30);assert f.done(),name;r=f.result();node.destroy_client(c);return r
try:
 req=ChangeState.Request();req.transition.id=1;assert call(ChangeState,'/controller_server/change_state',req).success
 names=json.loads((ROOT/'final8/0_straight/result.json').read_text())['verified_goal_checker'].keys();names=list(names)+['FollowPath.'+k for k in ['plugin','wrapped_plugin','footprint_model.vertices','min_obstacle_dist','max_vel_x','max_vel_x_backwards','max_vel_y','max_vel_theta','acc_lim_x','acc_lim_y','acc_lim_theta','dt_ref','dt_hysteresis','no_inner_iterations','no_outer_iterations','enable_homotopy_class_planning','enable_multithreading','max_global_plan_lookahead_dist','free_goal_vel','feasibility_check_no_poses','feasibility_check_lookahead_distance','min_resolution_collision_check_angular','include_costmap_obstacles']]
 values={}
 for name in names:
  req=GetParameters.Request(names=[name]);r=call(GetParameters,'/controller_server/get_parameters',req);values[name]=parameter_value_to_python(r.values[0]) if r.values else None
 native={}
 for line in Path('/proc',str(p.pid),'maps').read_text().splitlines():
  if 'libteb_local_planner.so' in line:
   f=Path(line.split()[-1]);native[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
 info={'parameters':values,'scope':'Same loaded native controller binary and provided YAML, configure only. Not a retroactive per-goal runtime query; no simulation, activation, command computation or navigation goal.','original_yaml_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),'original_run_binary_maps':json.loads((ROOT/'final8/0_straight/controller_binary_maps.json').read_text()),'binary_hashes':native,'original_runtime_query_failure':'Batch GetParameters contained undeclared xy_goal_tolerance/yaw_goal_tolerance; ROS service returned an empty batch. Original raw result remains unchanged.'};(out/'evidence.json').write_text(json.dumps(info,indent=2));print(json.dumps(info))
finally:
 node.destroy_node();rclpy.shutdown()
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=10)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
