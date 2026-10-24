"""Check dense v2 transit including intermediate checker XY/yaw envelopes."""
from pathlib import Path
import json,ctypes as c,math
import numpy as np
r=Path(__file__).resolve().parent;run=r.parent/'regression8/7_other_room';snapshot=sorted((run/'snapshots').glob('*.json'))[-1];d=json.loads(snapshot.read_text());arrays=np.load(snapshot.with_suffix('.npz'))
lib=c.CDLL(str(r/'libfootprint.so'));f=lib.check_poses;f.argtypes=[c.POINTER(c.c_ubyte),c.c_uint,c.c_uint,c.c_double,c.c_double,c.c_double,c.POINTER(c.c_double),c.c_int,c.POINTER(c.c_double)]
def check(poses,k):
 a=arrays[k].copy();m=d[k];b=np.asarray(poses,np.float64).copy()
 if m['frame']=='odom':
  gm=d['robot_map'];om=d['robot_odom'];theta=om[2]-gm[2];dx=b[:,0]-gm[0];dy=b[:,1]-gm[1]
  b[:,0]=om[0]+np.cos(theta)*dx-np.sin(theta)*dy;b[:,1]=om[1]+np.sin(theta)*dx+np.cos(theta)*dy;b[:,2]+=theta
 b=np.ascontiguousarray(b);o=np.zeros(len(b));f(a.ctypes.data_as(c.POINTER(c.c_ubyte)),a.shape[1],a.shape[0],m['resolution'],*m['origin'][:2],b.ctypes.data_as(c.POINTER(c.c_double)),len(b),o.ctypes.data_as(c.POINTER(c.c_double)));return o
poses=[[-17.125+dx,y,math.pi+dyaw] for dx in [-.02,0,.02] for dyaw in [-math.radians(2),0,math.radians(2)] for y in np.arange(.005,2.046,.01)]
rows={}
for k in ['local','global']:
 costs=check(poses,k);bad=np.flatnonzero((costs<0)|(costs>=254));rows[k]={'samples':len(costs),'collisions':len(bad),'max_cost':float(max(costs)),'first_invalid_poses':[poses[i] for i in bad[:10]]}
out={'snapshot':str(snapshot),'description':'Dense transit x=-17.125±.02, yaw=pi±2deg, y=.005..2.045. Static snapshot test; not a guarantee against localization changes.','maps':rows}
(r/'portal-clearance-v2.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
