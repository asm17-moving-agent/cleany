from pathlib import Path
import json,math,numpy as np
O=Path(__file__).parent;d=json.loads((O/'actual-path-snapshot/snapshot.json').read_text());a=np.load(O/'actual-path-snapshot/local-costmap.npy');p=json.loads((O/'actual-path-snapshot/actual-path.json').read_text())['path'];dense=[]
for b,e in zip([d['robot_pose']]+p, p):
 delta=math.atan2(math.sin(e[2]-b[2]),math.cos(e[2]-b[2]));N=max(1,math.ceil(math.hypot(e[0]-b[0],e[1]-b[1])/.01),math.ceil(abs(delta)/math.radians(1)))
 dense += [[b[0]+(e[0]-b[0])*t/N,b[1]+(e[1]-b[1])*t/N,b[2]+delta*t/N] for t in range(N+1)]
p=dense
theta=0.0;c,s=math.cos(theta),math.sin(theta);R=np.array([[c,-s],[s,c]]);offset=np.zeros(2);res=d['resolution'];origin=np.array(d['origin']);body=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]);rows=[]
for x,y,yaw in p:
 point=R@np.array([x,y])+offset;odomyaw=yaw+theta;c,s=math.cos(odomyaw),math.sin(odomyaw);poly=body@np.array([[c,s],[-s,c]])+point;cells=set()
 for b,e in zip(poly,np.roll(poly,-1,axis=0)):
  x0,y0=np.floor((b-origin)/res).astype(int);x1,y1=np.floor((e-origin)/res).astype(int);dx=abs(x1-x0);dy=abs(y1-y0);sx=1 if x0<x1 else -1;sy=1 if y0<y1 else -1;err=dx-dy
  while True:
   if 0<=x0<a.shape[1] and 0<=y0<a.shape[0] and a[y0,x0]==254:cells.add((int(x0),int(y0)))
   if x0==x1 and y0==y1:break
   e2=2*err
   if e2>-dy:err-=dy;x0+=sx
   if e2<dx:err+=dx;y0+=sy

 rows.append({'map_pose':[x,y,yaw],'odom_pose':[*point.tolist(),odomyaw],'lethal_boundary_cells':len(cells),'cells':[[int(v) for v in c] for c in cells]})
result={'note':'Diagnostic global plan compared with earlier local snapshot through saved map/odom transform; asynchronous comparison, not exact live MPPI evaluation','colliding_pose_count':sum(r['lethal_boundary_cells']>0 for r in rows),'pose_count':len(rows),'rows':rows};(O/'stall-diagnosis/actual-sync-swept-vs-local.json').write_text(json.dumps(result,indent=2));print('global collision poses',result['colliding_pose_count'],'/',len(rows));print([r for r in rows if r['lethal_boundary_cells']][:4])
