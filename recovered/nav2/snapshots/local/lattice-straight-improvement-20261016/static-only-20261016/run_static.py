"""Fixed-pose native Lattice comparison; no simulator or controller is started."""
from pathlib import Path
import copy,json,math,subprocess,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent
OLD=PARENT.parent/'planner-lattice-theta-20261016'
STOCK=Path('/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json')
raw=json.loads((PARENT/'grid5cm.json').read_text());filtered=copy.deepcopy(raw)
filtered['primitives']=[p for p in filtered['primitives'] if p['arc_length']<.001]
for n,p in enumerate(filtered['primitives']):p['trajectory_id']=n
filtered['lattice_metadata']['number_of_trajectories']=len(filtered['primitives'])
filtered_path=ROOT/'no-curves.json';filtered_path.write_text(json.dumps(filtered,indent=2))
current={'primitive_file':str(PARENT/'grid5cm.json'),'lookup_dim':1.0,'rotation_penalty':.5,'non_straight_penalty':2.0,'reverse_penalty':1.0,'change_penalty':0.0,'retrospective_penalty':0.0,'tolerance_cells':0.0}
variants={'original_profile':{'primitive_file':str(STOCK),'tolerance_cells':1.0},'original_exact':{'primitive_file':str(STOCK),'tolerance_cells':0.0},'current':current,'current_cost0':{**current,'cost_penalty':0.0},'no_curves':{**current,'primitive_file':str(filtered_path)},'no_curves_cost0':{**current,'primitive_file':str(filtered_path),'cost_penalty':0.0}}
summary=[];provenance=[]
for i,name in [(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')]:
 folder=OLD/'lattice'/f'{i}_{name}';r=json.loads((folder/'result.json').read_text())
 start=r['initial_truth_world'];assert math.dist(start[:2],[-1.865,-4.705])<1e-6
 snaps=[s for s in (folder/'snapshots').glob('*.json') if float(s.stem)<=r['navigation_sim_start_s']]
 snap=max(snaps,key=lambda s:float(s.stem));meta=json.loads(snap.read_text());a=np.load(snap.with_suffix('.npz'))['global'];binary=ROOT/f'{i}-{name}.bin';a.tofile(binary)
 provenance.append({'route':name,'snapshot':str(snap),'stamp':meta['global']['stamp_s'],'binary_sha256':hashlib.sha256(binary.read_bytes()).hexdigest(),'start':start,'goal':r['goal_map']})
 for v,changes in variants.items():
  stem=ROOT/f'{i}-{name}-{v}';j={'nx':a.shape[1],'ny':a.shape[0],'resolution':meta['global']['resolution'],'origin':meta['global']['origin'],'binary':str(binary),'start':start,'goal':r['goal_map'],'footprint':[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]],'mode':'snapshot',**changes}
  inp=stem.with_suffix('.input.json');out=stem.with_suffix('.output.json');inp.write_text(json.dumps(j))
  with stem.with_suffix('.log').open('w') as log:code=subprocess.run([str(PARENT/'native_probe'),str(inp),str(out)],stdout=log,stderr=subprocess.STDOUT).returncode
  q=json.loads(out.read_text()) if out.exists() else {};p=q.get('poses',[]);row={'route_index':i+1,'route':name,'variant':v,'exit':code,**{k:q.get(k) for k in ['ok','iterations','search_ms']}}
  if p:
   a0,b=p[0],p[-1];dx,dy=b[0]-a0[0],b[1]-a0[1];length=math.hypot(dx,dy)
   row.update(max_cross_track_cm=max(abs(dx*(z[1]-a0[1])-dy*(z[0]-a0[0]))/length for z in p)*100,length_m=sum(math.dist(x[:2],y[:2]) for x,y in zip(p,p[1:])),straight_distance_m=length,yaw_variation_deg=sum(abs(math.degrees(math.atan2(math.sin(y[2]-x[2]),math.cos(y[2]-x[2])))) for x,y in zip(p,p[1:])),goal_yaw_error_deg=abs(math.degrees(math.atan2(math.sin(b[2]-r['goal_map'][2]),math.cos(b[2]-r['goal_map'][2])))),pose_count=len(p),start_pose=a0,end_pose=b)
  summary.append(row);print(json.dumps(row),flush=True);(ROOT/'summary.json').write_text(json.dumps(summary,indent=2))
(ROOT/'provenance.json').write_text(json.dumps({'type':'Static native AStarAlgorithm<NodeLattice> and GridCollisionChecker only; no MPPI/Gazebo, localization or replanning','source':'Same pre-navigation frozen costmap per route; all variants use identical poses and map','start':'Actual home world/map pose held constant across all four routes; native 5cm grid quantization retained','footprint_padding_m':0,'heading_count':16,'primitive_counts':{'stock':144,'current':272,'no_curves':len(filtered['primitives'])},'profile_notes':{'original_profile':'Original planner parameters incl 5cm tolerance','original_exact':'Original parameters but exact goal cell and heading required for common endpoint comparison','current':'Final refined candidate from previous experiment','current_cost0':'Disable soft obstacle cost penalty for diagnosis; collision checker remains enabled','no_curves':'Current profile with only curved translation primitives removed; existing translations and in-place rotations kept','no_curves_cost0':'Combine removal of curves and disabling soft cost penalty for diagnosis'},'snapshots':provenance},indent=2))
