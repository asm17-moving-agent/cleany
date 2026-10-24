"""Native footprint costs of saved robot/goal poses in both saved costmaps."""
import argparse,ctypes as c,json,math
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();r=args.run
lib=c.CDLL(str(Path(__file__).resolve().parent/'libfootprint.so'));f=lib.check_poses;f.argtypes=[c.POINTER(c.c_ubyte),c.c_uint,c.c_uint,c.c_double,c.c_double,c.c_double,c.POINTER(c.c_double),c.c_int,c.POINTER(c.c_double)]
goal=json.loads((r/'result.json').read_text())['goal_map'];rows=[]
for path in sorted((r/'snapshots').glob('*.json')):
 d=json.loads(path.read_text());arrays=np.load(path.with_suffix('.npz'));record={'sim_s':d['sim_s'],'gt':d['gt'],'robot_map':d.get('robot_map')}
 if 'robot_map' not in d or 'robot_odom' not in d:continue
 for k in ['global','local']:
  a=arrays[k].copy();m=d[k];b=np.array([d['robot_map'],goal],dtype=np.float64)
  if m['frame']=='odom':
   gm=d['robot_map'];om=d['robot_odom'];theta=om[2]-gm[2];dx=b[:,0]-gm[0];dy=b[:,1]-gm[1]
   b[:,0]=om[0]+np.cos(theta)*dx-np.sin(theta)*dy;b[:,1]=om[1]+np.sin(theta)*dx+np.cos(theta)*dy;b[:,2]+=theta
  b=np.ascontiguousarray(b);o=np.zeros(len(b));f(a.ctypes.data_as(c.POINTER(c.c_ubyte)),a.shape[1],a.shape[0],m['resolution'],*m['origin'][:2],b.ctypes.data_as(c.POINTER(c.c_double)),len(b),o.ctypes.data_as(c.POINTER(c.c_double)))
  record[k]={'robot_cost':float(o[0]),'goal_cost':float(o[1])}
 rows.append(record)
(r/'native_pose_costs.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows[-3:],indent=2))
