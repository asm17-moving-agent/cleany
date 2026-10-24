"""Check configured doorway corridors with Nav2's native footprint rasterizer."""
from pathlib import Path
import ctypes as c,json,math,numpy as np
r=Path(__file__).resolve().parent;root=r.parent
snapshot=sorted((root/'pathseed_m1_v3/snapshots').glob('*.json'))[-1]
d=json.loads(snapshot.read_text());arrays=np.load(snapshot.with_suffix('.npz'))
lib=c.CDLL(str(r/'libfootprint.so'));f=lib.check_poses
f.argtypes=[c.POINTER(c.c_ubyte),c.c_uint,c.c_uint,c.c_double,c.c_double,c.c_double,c.POINTER(c.c_double),c.c_int,c.POINTER(c.c_double)]
def check(poses,kind='global'):
 a=arrays[kind].copy();m=d[kind];b=np.asarray(poses,np.float64).copy()
 if m['frame']=='odom':
  gm=d['robot_map'];om=d['robot_odom'];theta=om[2]-gm[2]
  dx=b[:,0]-gm[0];dy=b[:,1]-gm[1]
  b[:,0]=om[0]+np.cos(theta)*dx-np.sin(theta)*dy
  b[:,1]=om[1]+np.sin(theta)*dx+np.cos(theta)*dy;b[:,2]+=theta
 b=np.ascontiguousarray(b);o=np.zeros(len(b))
 f(a.ctypes.data_as(c.POINTER(c.c_ubyte)),a.shape[1],a.shape[0],m['resolution'],*m['origin'][:2],b.ctypes.data_as(c.POINTER(c.c_double)),len(b),o.ctypes.data_as(c.POINTER(c.c_double)));return o
def stats(poses):
 return {k:{'collisions':int(sum((v>=254)|(v<0))),'max_cost':float(max(v)),'count':len(v)} for k in ['local','global'] for v in [check(poses,k)]}
rows=[]
for x in np.arange(-17.35,-16.949,.025):
 poses=[[x,y,math.pi] for y in np.arange(.10,1.61,.01)];rows.append({'x':float(x),**stats(poses)})
rotations=[]
for y in [-.3,0.,.1,1.5,1.7,1.8]:
 rotations.append({'pose':[-17.125,y],**stats([[-17.125,y,yaw] for yaw in np.arange(math.pi/2,math.pi+.001,math.pi/180)])})
result={'snapshot':str(snapshot),'scan':rows,'rotations_90_to_180':rotations}
(r/'portal-clearance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
