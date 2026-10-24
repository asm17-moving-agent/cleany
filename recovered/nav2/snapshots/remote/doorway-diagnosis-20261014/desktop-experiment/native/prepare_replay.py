from pathlib import Path
import json,yaml,numpy as np
r=Path(__file__).resolve().parents[1];e=r.parent/'laptop-evidence';o=r/'native/replay';o.mkdir(exist_ok=True)
d=json.loads((e/'actual-path-snapshot/snapshot.json').read_text());a=np.load(e/'actual-path-snapshot/local-costmap.npy');a.tofile(o/'map.bin')
p=json.loads((e/'actual-path-snapshot/actual-path.json').read_text())['path']
(o/'poses.txt').write_text('\n'.join(' '.join(map(str,v)) for v in [d['robot_pose']]+p))
(o/'map.txt').write_text(' '.join(map(str,[a.shape[1],a.shape[0],d['resolution'],*d['origin']])))
f=yaml.safe_load((r/'laptop-runtime/effective_parameters.yaml').read_text())['controller_server']['ros__parameters']
f['FollowPath']['regenerate_noises']=False
params={'controller_server':{'ros__parameters':f},'local_costmap':{'local_costmap':{'ros__parameters':{'global_frame':'odom','robot_base_frame':'base_link','plugins':['inflation_layer'],'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.8,'cost_scaling_factor':6.},'footprint':str([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]),'footprint_padding':0.,'resolution':.05,'use_sim_time':False}}}}
f['use_sim_time']=False
for mode in ['baseline','pathseed']:
 f['FollowPath']['path_seed_count']=64 if mode=='pathseed' else 0;f['FollowPath']['path_seed_max_speed']=.25
 (o/(mode+'.yaml')).write_text(yaml.safe_dump(params))
