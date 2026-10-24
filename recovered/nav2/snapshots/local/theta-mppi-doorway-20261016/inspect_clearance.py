from pathlib import Path
import ctypes as c,json,math
import numpy as np
R=Path(__file__).resolve().parent;old=R/'clearance'
lib=c.CDLL(str(R.parent/'doorway-diagnosis-20261014/desktop-experiment/native/libfootprint.so'));f=lib.check_poses;f.argtypes=[c.POINTER(c.c_ubyte),c.c_uint,c.c_uint,c.c_double,c.c_double,c.c_double,c.POINTER(c.c_double),c.c_int,c.POINTER(c.c_double)]
records=[]
for p in sorted((old/'snapshots').glob('*.json'))[-3:]:
 d=json.loads(p.read_text());a=np.load(p.with_suffix('.npz'));gm=d['robot_map'];om=d['robot_odom'];path=np.asarray(d['plan_map']);nearest=np.argmin(np.linalg.norm(path[:,:2]-np.asarray(gm[:2]),axis=1));selected=path[nearest:];selected=selected[np.linalg.norm(selected[:,:2]-np.asarray(gm[:2]),axis=1)<1.2]
 row={'snapshot':p.name,'gt':d['gt'],'robot_map':gm,'path_yaws_deg':list(sorted(set(round(math.degrees(v),1) for v in selected[:,2]))),'orientations':{}}
 for yaw in [0,math.pi/2]:
  poses=selected.copy();poses[:,2]=yaw;costs={}
  for k in ['local','global']:
   data=np.ascontiguousarray(a[k]);m=d[k];q=poses.copy()
   if m['frame']=='odom':
    theta=om[2]-gm[2];v=q[:,:2]-np.asarray(gm[:2]);q[:,0]=om[0]+math.cos(theta)*v[:,0]-math.sin(theta)*v[:,1];q[:,1]=om[1]+math.sin(theta)*v[:,0]+math.cos(theta)*v[:,1];q[:,2]+=theta
   q=np.ascontiguousarray(q);out=np.zeros(len(q));f(data.ctypes.data_as(c.POINTER(c.c_ubyte)),data.shape[1],data.shape[0],m['resolution'],*m['origin'][:2],q.ctypes.data_as(c.POINTER(c.c_double)),len(q),out.ctypes.data_as(c.POINTER(c.c_double)))
   costs[k]={'poses':len(q),'lethal':int(sum(out>=254)),'off_map':int(sum(out<0)),'max_cost':float(max(out,default=0))}
  row['orientations'][str(round(math.degrees(yaw)))]=costs
 records.append(row)
(R/'clearance-native-inspection.json').write_text(json.dumps(records,indent=2));print(json.dumps(records,indent=2))
