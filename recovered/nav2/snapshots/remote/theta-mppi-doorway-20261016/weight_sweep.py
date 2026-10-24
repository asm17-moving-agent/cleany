from pathlib import Path
import ctypes as c,json,math,subprocess
import numpy as np
R=Path(__file__).resolve().parent;old=R.parent/'planner-lattice-theta-20261016/theta/7_other_room';O=R/'weight-sweep';O.mkdir(exist_ok=True)
lib=c.CDLL(str(R.parent/'doorway-diagnosis-20261014/desktop-experiment/native/libfootprint.so'));f=lib.check_poses;f.argtypes=[c.POINTER(c.c_ubyte),c.c_uint,c.c_uint,c.c_double,c.c_double,c.c_double,c.POINTER(c.c_double),c.c_int,c.POINTER(c.c_double)]
summary=[]
for tag in ['00005','00030','00041']:
 paths=sorted((old/'snapshots').glob(tag+'*.json'))
 if not paths:continue
 p=paths[-1];d=json.loads(p.read_text());a=np.ascontiguousarray(np.load(p.with_suffix('.npz'))['global']);m=d['global'];binary=O/(tag+'.bin');a.tofile(binary)
 for weight in [2.,5.,10.,20.]:
  j={'nx':a.shape[1],'ny':a.shape[0],'resolution':m['resolution'],'origin':m['origin'],'mode':'snapshot','binary':str(binary),'start':d['robot_map'],'goal':[-16.775,1.775,1.570796],'footprint':[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]],'w_traversal_cost':weight}
  stem=O/f'{tag}-{int(weight)}';inp=stem.with_suffix('.input.json');out=stem.with_suffix('.output.json');inp.write_text(json.dumps(j));subprocess.run([str(R/'theta_probe'),str(inp),str(out)],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL);result=json.loads(out.read_text());path=result['poses'];dense=[]
  for u,v in zip(path,path[1:]):
   n=max(1,math.ceil(math.dist(u[:2],v[:2])/.025));dense.extend([[u[0]+(v[0]-u[0])*i/n,u[1]+(v[1]-u[1])*i/n,0] for i in range(n)])
  dense.append(path[-1]);rec={'snapshot':p.name,'weight':weight,'length_m':sum(math.dist(u[:2],v[:2]) for u,v in zip(path,path[1:])),'footprint_lethal':{},'path_pose_count':len(path)}
  for angle in [0,math.pi/2]:
   q=np.ascontiguousarray(dense,dtype=np.float64);q[:,2]=angle;cost=np.zeros(len(q));f(a.ctypes.data_as(c.POINTER(c.c_ubyte)),a.shape[1],a.shape[0],m['resolution'],*m['origin'][:2],q.ctypes.data_as(c.POINTER(c.c_double)),len(q),cost.ctypes.data_as(c.POINTER(c.c_double)));rec['footprint_lethal'][str(round(math.degrees(angle)))]=int(sum(cost>=254))
  summary.append(rec);print(rec,flush=True);(O/'summary.json').write_text(json.dumps(summary,indent=2))
