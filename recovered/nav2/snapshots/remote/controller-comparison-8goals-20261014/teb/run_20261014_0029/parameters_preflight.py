from pathlib import Path
import json,yaml,numpy as np
import runner
from materialize import COMMON
runner.LOCKED=json.loads((COMMON/'goals.locked.json').read_text())
out=Path(__file__).resolve().parent/'parameters_preflight'/'0_straight';out.mkdir(parents=True)
scenario=runner.setup(out,'teb',1.)
p=yaml.safe_load((out/'config/relaxed_parameters.yaml').read_text())
assert p['global_costmap']['global_costmap']['ros__parameters']['resolution']==.025
assert p['planner_server']['ros__parameters']['GridBased']['body_heading_grid_bins']==32
assert p['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance']==.30
v=[[float(t),0.,0.,.1] for t in np.arange(0,3,.05)]
s=runner.C.summarize_velocity(v,0,3)
assert s['stationary_s']==0.,s
assert runner.C.summarize_velocity([[r[0],0.,0.,0.] for r in v],0,3)['stationary_s']>2.
print(json.dumps({'parameter_preflight':True,'yaw_only_is_moving':True,'scene_models':len(scenario['expected_scene_models'])}))
