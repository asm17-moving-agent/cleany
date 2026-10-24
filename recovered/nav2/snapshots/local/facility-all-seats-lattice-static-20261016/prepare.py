from pathlib import Path
import sys,copy,json,hashlib,xml.etree.ElementTree as ET
from collections import Counter
import numpy as np,yaml
from PIL import Image
R=Path(__file__).resolve().parent
C=Path('/home/donglee/workspace/cleany/controller-comparison-8goals-20261014/common')
sys.path.insert(0,str(C));import prepare_common as pc
pc.FIX=C/'fixture'
T=ET.parse(C/'world.full.template.sdf');W=T.getroot().find('world');models={m.get('name'):m for m in W.findall('model')}
targets=yaml.safe_load((C/'fixture/targets.yaml').read_text());home=targets['home'];FP=[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]
assert len(targets['seats'])==82
chairnames=[f'office_chair_{i:02d}' for i in range(1,49)];assert all(n in models for n in chairnames)
L=R/'layers';L.mkdir(exist_ok=True);M=R/'maps';M.mkdir(exist_ok=True)
# Raster the shared world and each D-hub chair independently. Occupancy union preserves all overlapping obstacles.
base=copy.deepcopy(T);bw=base.getroot().find('world')
for m in list(bw.findall('model')):
 if m.get('name') in chairnames:bw.remove(m)
basedir=L/'base';basedir.mkdir(exist_ok=True);base.write(basedir/'world.sdf',encoding='unicode');baseocc=np.array(pc.raster(basedir/'world.sdf',basedir))==0
count=np.zeros(baseocc.shape,dtype=np.uint16);masks={}
for n in chairnames:
 p=L/n;p.mkdir(exist_ok=True);tr=ET.ElementTree(ET.Element('sdf',version='1.8'));w=ET.SubElement(tr.getroot(),'world',name='chair_slice');w.append(copy.deepcopy(models[n]));tr.write(p/'world.sdf',encoding='unicode');a=np.array(pc.raster(p/'world.sdf',p))==0;count+=a;masks[n]=a
full=baseocc|(count>0)
def save_map(name,occ):
 folder=M/name;folder.mkdir(exist_ok=True)
 arr=np.where(occ,0,254).astype(np.uint8);arr=arr.reshape(arr.shape[0]//2,2,arr.shape[1]//2,2).min(axis=(1,3));Image.fromarray(arr).save(folder/'map.pgm')
 (folder/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-24.,-15.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
 return folder/'map.yaml',hashlib.sha256((folder/'map.pgm').read_bytes()).hexdigest()
fullmap,fullsha=save_map('all-chairs-present',full)
records=[]
for seat,goal in targets['seats'].items():
 isdh=seat[5:].isdigit();n='office_chair_'+seat[5:] if isdh else None
 if isdh:
  occ=baseocc|((count-masks[n])>0);mp,sha=save_map(seat,occ);changed=int(np.count_nonzero(full!=occ));assert not np.any(occ&~full)
 else:mp,sha=fullmap,fullsha;changed=0
 zone='D-hub' if isdh else seat.split('-')[1].upper()
 records.append(dict(seat=seat,zone=zone,goal=goal,map=str(mp),map_sha256=sha,removed_chair=n,removed_only_target_chair=True,changed_2p5cm_cells=changed))
# D-hub first; the room subset is selected before planner execution.
records.sort(key=lambda x:(x['zone']!='D-hub',x['zone'],x['seat']))
(R/'goals.json').write_text(json.dumps(dict(home=home,footprint=FP,goals=records),indent=2))
provenance={'world':str(C/'world.full.template.sdf'),'world_sha256':hashlib.sha256((C/'world.full.template.sdf').read_bytes()).hexdigest(),'targets':str(C/'fixture/targets.yaml'),'targets_sha256':hashlib.sha256((C/'fixture/targets.yaml').read_bytes()).hexdigest(),'seat_counts':dict(Counter(x['zone'] for x in records)),'map':'Same canonical triangle/primitive collision slice at z=0.30m, raster0.025m then conservative2x2 downsample to0.05m; no simulator started. D-hub maps omit exactly the destination chair, other objects retained; room maps retain every chair. Collision-slice maps are not full3D geometry.','goal_source':'Existing fixed approximate desk-front approach poses; no automatic goal relocation. Duplicate approach poses are preserved and reported.','duplicate_goal_groups':[[x['seat'] for x in records if x['goal']==g] for g in {tuple(x['goal']) for x in records} if sum(tuple(x['goal'])==g for x in records)>1]}
# Compare original full-world slice to stored canonical no-removal map when available.
expected=C/'goals/0_straight/map.pgm'
if expected.exists():provenance['full_map_matches_prior_canonical_2p5cm']=np.array_equal(np.where(full,0,254).astype(np.uint8),np.array(Image.open(expected)))
(R/'provenance.json').write_text(json.dumps(provenance,indent=2));print(json.dumps(provenance),flush=True)
