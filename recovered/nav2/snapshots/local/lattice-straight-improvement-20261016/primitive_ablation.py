from pathlib import Path
import json,subprocess,math
import numpy as np
ROOT=Path(__file__).resolve().parent
OLD=ROOT.parent/'planner-lattice-theta-20261016'
OUT=ROOT/'primitive-ablation';OUT.mkdir(exist_ok=True)
FP=[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]
PRIM='/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json'
variants={'length_only':{'primitive_file':str(ROOT/'length_only.json')},'grid5cm':{'primitive_file':str(ROOT/'grid5cm.json')},'grid5cm_low_rotation':{'primitive_file':str(ROOT/'grid5cm.json'),'rotation_penalty':.5,'non_straight_penalty':1.2,'reverse_penalty':1.0,'change_penalty':0.0,'retrospective_penalty':0.0}}
summary=[]
for name in ['0_straight','1_turn_arrival','3_far_heading','5_lateral']:
 old=OLD/'lattice'/name;r=json.loads((old/'result.json').read_text());p=json.loads((old/'plans.json').read_text())[0]['poses']
 # Last recorded global snapshot before navigation starts, matching the recorded initial map.
 snaps=sorted((old/'snapshots').glob('*.json'));snap=max((s for s in snaps if float(s.stem)<=r['navigation_sim_start_s']),key=lambda s:float(s.stem));meta=json.loads(snap.read_text());a=np.load(snap.with_suffix('.npz'))['global'];binary=OUT/(name+'.bin');a.tofile(binary)
 for v,changes in variants.items():
  stem=OUT/(name+'-'+v);j={'nx':a.shape[1],'ny':a.shape[0],'resolution':meta['global']['resolution'],'origin':meta['global']['origin'],'binary':str(binary),'start':p[0],'goal':r['goal_map'],'primitive_file':PRIM,'footprint':FP,'mode':'snapshot',**changes}
  inp=stem.with_suffix('.input.json');out=stem.with_suffix('.output.json');inp.write_text(json.dumps(j));code=subprocess.run([str(ROOT/'native_probe'),str(inp),str(out)],stdout=stem.with_suffix('.log').open('w'),stderr=subprocess.STDOUT).returncode
  q=json.loads(out.read_text()) if out.exists() else {};poses=q.get('poses',[]);rec={'route':name,'variant':v,'exit':code,**{k:q.get(k) for k in ['ok','iterations','search_ms']}}
  if poses:
   a0,b=poses[0],poses[-1];dx,dy=b[0]-a0[0],b[1]-a0[1];den=math.hypot(dx,dy);rec['max_cross_track_m']=max(abs(dx*(x[1]-a0[1])-dy*(x[0]-a0[0]))/den for x in poses);rec['length_m']=sum(math.dist(x[:2],y[:2]) for x,y in zip(poses,poses[1:]));rec['pose_count']=len(poses)
  summary.append(rec);print(json.dumps(rec),flush=True);(OUT/'summary.json').write_text(json.dumps(summary,indent=2))
