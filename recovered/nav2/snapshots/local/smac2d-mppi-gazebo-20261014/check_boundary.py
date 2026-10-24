from pathlib import Path
import json,numpy as np,math
p=Path(__file__).parent/'costmap-stall';d=json.loads((p/'snapshot.json').read_text());a=np.load(p/'local-costmap.npy');r=d['resolution'];ox,oy=d['origin'];poly=d['published_footprint'];hits=[]
for i in range(len(poly)):
 p1,p2=poly[i],poly[(i+1)%len(poly)];x0,y0=[math.floor((v-o)/r) for v,o in zip(p1,[ox,oy])];x1,y1=[math.floor((v-o)/r) for v,o in zip(p2,[ox,oy])]
 dx=abs(x1-x0);dy=abs(y1-y0);sx=1 if x0<x1 else -1;sy=1 if y0<y1 else -1;err=dx-dy
 while True:
  if 0<=y0<a.shape[0] and 0<=x0<a.shape[1] and a[y0,x0]==254:hits.append({'edge':i,'cell':[x0,y0],'center':[ox+(x0+.5)*r,oy+(y0+.5)*r]})
  if x0==x1 and y0==y1:break
  e2=2*err
  if e2>-dy:err-=dy;x0+=sx
  if e2<dx:err+=dx;y0+=sy
print(hits);d['rasterized_boundary_lethal_hits']=hits;(p/'snapshot.json').write_text(json.dumps(d,indent=2))
