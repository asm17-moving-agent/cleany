from pathlib import Path
from tempfile import TemporaryDirectory
import hashlib,json,yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from cleany_navigation import launch_runtime
W=Path('/home/ehdrms/workspace/cleany/nav2-tuning-recovered')
share=Path(get_package_share_directory('cleany_navigation'));primitive=share/'config/lattice/omni-r10cm.json'
sha=hashlib.sha256(primitive.read_bytes()).hexdigest();assert sha=='01165b98a42f01c0676f9ee6e691a9ed27b72dc7413c18a3a92b830ca3f2756e'
results=[]
for label,path in [('navigation',share/'config/nav2_amcl.yaml'),('gazebo',W/'ros2_ws/src/cleany_gazebo_sim/config/nav2_amcl.yaml')]:
 with TemporaryDirectory() as tmp:
  out=Path(tmp);mapfile=out/'map.yaml';mapfile.write_text('image: unused.pgm\n');launch_runtime.mkdtemp=lambda **kwargs: str(out)
  ctx=LaunchContext();ctx.launch_configurations.update(map=str(mapfile),params_file=str(path),robot_params_file='',localization_params_file='',use_sim_time='true',autostart='false')
  nodes=launch_runtime._launch(ctx,safety=False);params=yaml.safe_load((out/'parameters.yaml').read_text());p=params['planner_server']['ros__parameters']['GridBased'];f=params['controller_server']['ros__parameters']['FollowPath']
  assert p['plugin']=='nav2_smac_planner/SmacPlannerLattice' and p['lattice_filepath']==str(primitive.resolve())
  assert f['plugin']=='nav2_mppi_controller::MPPIController' and f['motion_model']=='Omni' and f['vy_max']>0
  results.append({'profile':label,'planner':p,'controller':f['plugin'],'motion_model':f['motion_model'],'nodes_constructed':len(nodes)})
j={'launch_assembly_passed':True,'primitive_sha256':sha,'profiles':results,'ros_nodes_started':False,'gazebo_started':False}
(W/'recovered/nav2/provenance/default-lattice-launch-check.json').write_text(json.dumps(j,indent=2));print(json.dumps(j,indent=2))
