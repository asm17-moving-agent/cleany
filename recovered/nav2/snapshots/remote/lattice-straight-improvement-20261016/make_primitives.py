from pathlib import Path
import copy,json,math,hashlib
ROOT=Path(__file__).resolve().parent
source=Path('/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json')
raw=json.loads(source.read_text());report={'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'variants':{}}
for mode in ['length_only','grid5cm']:
 d=copy.deepcopy(raw); corrected=[]
 if mode=='length_only':
  for p in d['primitives']:
   if p['arc_length']>=.001 or p['trajectory_length']<1e-4:continue
   length=sum(math.dist(a[:2],b[:2]) for a,b in zip([[0,0,0]]+p['poses'],p['poses']))
   if abs(length-p['trajectory_length'])>.001:
    corrected.append({'id':p['trajectory_id'],'old_length':p['trajectory_length'],'actual_length':length});p['trajectory_length']=length;p['straight_length']=length
 else:
  headings=d['lattice_metadata']['heading_angles'];resolution=d['lattice_metadata']['grid_resolution']
  for i,yaw in enumerate(headings):
   for dx,dy in [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]:
    x,y=round(dx*resolution,9),round(dy*resolution,9);length=math.hypot(x,y)
    d['primitives'].append({'trajectory_id':0,'start_angle_index':i,'end_angle_index':i,'left_turn':True,'trajectory_radius':0.0,'trajectory_length':length,'arc_length':0.0,'straight_length':length,'poses':[[x/2,y/2,yaw],[x,y,yaw]]})
  # Native loader groups contiguous primitives by start heading.
  d['primitives'].sort(key=lambda p:p['start_angle_index'])
  for i,p in enumerate(d['primitives']):p['trajectory_id']=i
 d['lattice_metadata']['number_of_trajectories']=len(d['primitives'])
 out=ROOT/(mode+'.json');out.write_text(json.dumps(d,indent=2));report['variants'][mode]={'count':len(d['primitives']),'corrected':corrected,'sha256':hashlib.sha256(out.read_bytes()).hexdigest()}
(ROOT/'primitive-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
