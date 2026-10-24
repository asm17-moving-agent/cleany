from pathlib import Path
import json,sys,math
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
r=Path(sys.argv[1]);snap=sorted((r/'snapshots').glob('*.json'))[-1];d=json.loads(snap.read_text());a=np.load(snap.with_suffix('.npz'))
fig,axs=plt.subplots(1,2,figsize=(14,6))
for k,ax in zip(['global','local'],axs):
 info=d[k];frame=info['frame'];o=info['origin'];res=info['resolution'];im=a[k];ax.imshow(im,origin='lower',extent=[o[0],o[0]+im.shape[1]*res,o[1],o[1]+im.shape[0]*res],vmin=0,vmax=255,cmap='magma_r')
 path=np.asarray(d['plan_map']);p=np.asarray(d['robot_'+frame]);path0=path.copy()
 if frame=='odom':
  m=d['robot_map'];od=d['robot_odom'];angle=od[2]-m[2];rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);path[:,:2]=(path[:,:2]-m[:2])@rot.T+od[:2];path[:,2]+=angle
 ax.plot(path[:,0],path[:,1],color='cyan',lw=1);ax.quiver(path[::8,0],path[::8,1],np.cos(path[::8,2]),np.sin(path[::8,2]),color='green',scale=20)
 fp=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]);c,s=math.cos(p[2]),math.sin(p[2]);fp=fp@np.array([[c,s],[-s,c]])+p[:2];ax.add_patch(Polygon(fp,fill=False,color='blue',lw=2));ax.scatter(*p[:2],color='blue')
 ax.set_xlim(p[0]-2,p[0]+2);ax.set_ylim(p[1]-2,p[1]+2);ax.set_title(f'{k}: {d["sim_s"]:.2f}s, robot yaw {math.degrees(p[2]):.1f}deg');ax.set_aspect('equal')
fig.tight_layout();fig.savefig(r/'failure_snapshot.png',dpi=130);print('snapshot',snap,'GT',d['gt'],'map',d.get('robot_map'))
