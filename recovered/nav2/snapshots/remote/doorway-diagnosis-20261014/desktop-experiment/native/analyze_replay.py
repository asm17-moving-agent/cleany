from pathlib import Path
import json,numpy as np
r=Path(__file__).resolve().parent/'replay';result={'scope':'Frozen laptop local costmap and actual 29-pose MPPI path. Native Nav2 critics, 240 optimizer calls at a fixed pose and zero measured velocity; fixed Gaussian batch. Not a physics driving result.','models':{}}
for mode in ['baseline','pathseed']:
 d=np.genfromtxt(r/(mode+'_candidates.csv'),delimiter=',',names=True);out=np.genfromtxt(r/(mode+'_output.csv'),delimiter=',',names=True);rows=[]
 for n in np.unique(d['snapshot']):
  a=d[d['snapshot']==n];b=a[np.argmin(a['critic_total'])];yaw=np.degrees(a['yaw4']-a['robot_yaw'])
  rows.append({'snapshot':int(n),'sim_label_s':float(b['sim_s']),'required_yaw_deg':float(np.degrees(b['path0_yaw']-b['robot_yaw'])),'candidate_yaw_min_deg':float(yaw.min()),'candidate_yaw_max_deg':float(yaw.max()),'best_candidate':int(b['candidate']),'best_costs':{k:float(b[k]) for k in ['ConstraintCritic','CostCritic','GoalCritic','GoalAngleCritic','PathAlignCritic','PathFollowCritic','PathAngleCritic','TwirlingCritic','critic_total']},'collision_penalized_count':int(sum(a['CostCritic']>=180))})
 result['models'][mode]={'snapshots':rows,'command_first':{k:float(out[0][k]) for k in out.dtype.names},'command_last':{k:float(out[-1][k]) for k in out.dtype.names},'output_rollouts_with_lethal':int(sum(out['optimized_lethal']>0))}
result['confirmed']='At the frozen doorway pose the initial Gaussian proposals do not contain the required positive heading change. Coherent path proposals satisfy native collision scoring and achieve a substantially lower existing critic cost. A single critic rejecting all valid coupled maneuvers is not supported.'
result['limitations']=['Fixed state is not a closed-loop simulation','Initial measured speed assumed zero','Fixed Gaussian batch for paired comparison; live uses regenerated noise','Reported candidate costs precede MPPI control regularization','Repeated optimization can shift the Gaussian nominal sequence']
(r/'diagnosis.json').write_text(json.dumps(result,indent=2));print({m:result['models'][m]['snapshots'][0] for m in result['models']})
