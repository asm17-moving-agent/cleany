from pathlib import Path
import yaml,json,numpy as np
from PIL import Image
P=Path(__file__).resolve().parent
C=P.parents[1]/'controller-comparison-8goals-20261014/common'
d=yaml.safe_load((C/'preflight_planner.yaml').read_text())
d['planner_server']['ros__parameters']['GridBased']={'plugin':'nav2_smac_planner/SmacPlannerLattice','tolerance':.05,'allow_unknown':False,'smooth_path':False,'allow_reverse_expansion':True,'max_planning_time':5.,'cache_obstacle_heuristic':False,'lattice_filepath':'/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json','cost_penalty':2.}
d['global_costmap']['global_costmap']['ros__parameters']['resolution']=.05
for g in json.loads((C/'goals.locked.json').read_text())['goals']:
 p=P/g['map'];p.parent.mkdir(parents=True,exist_ok=True);image=Image.open((C/g['map']).parent/'map.pgm');arr=np.asarray(image);Image.fromarray(arr.reshape(image.height//2,2,image.width//2,2).min(axis=(1,3))).save(p.parent/'map.pgm');m=yaml.safe_load((C/g['map']).read_text());m['resolution']=.05;m['image']='map.pgm';p.write_text(yaml.safe_dump(m))
d['map_server']['ros__parameters']['yaml_filename']=str(P/'goals/0_straight/map.yaml');(P/'preflight_planner.yaml').write_text(yaml.safe_dump(d))
