from pathlib import Path
import json,math,yaml
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon,Patch
O=Path(__file__).parent;d=json.loads((O/'results.json').read_text());r=d['results'][0];m=yaml.safe_load(Path(d['map']).read_text());im=np.asarray(Image.open(Path(d['map']).parent/m['image']));res=m['resolution'];ox,oy,_=m['origin'];path=np.array(r['path']);body=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
poses=[]
for a,b in zip(path,path[1:]):
 dyaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));count=max(1,math.ceil(math.dist(a[:2],b[:2])/.01),math.ceil(abs(dyaw)/math.radians(1)))
 for t in np.arange(count)/count:poses.append([*(a[:2]+t*(b[:2]-a[:2])),a[2]+t*dyaw])
poses.append(path[-1].tolist());polys=[]
for x,y,yaw in poses:
 c,s=math.cos(yaw),math.sin(yaw);polys.append(body@np.array([[c,s],[-s,c]])+np.array([x,y]))
fig=plt.figure(figsize=(17,10));gs=fig.add_gridspec(2,3,width_ratios=[2,2,2],hspace=.25,wspace=.23);ax=fig.add_subplot(gs[:,0:2]);axes=[ax,fig.add_subplot(gs[0,2]),fig.add_subplot(gs[1,2])]
for a in axes:
 a.imshow(im,cmap='gray',vmin=0,vmax=255,origin='upper',extent=(ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res));a.add_collection(PolyCollection(polys,facecolors='#74b9ef',edgecolors='none',alpha=.09));a.plot(path[:,0],path[:,1],c='#1464bc',lw=1.7)
 for p in polys[::max(1,len(polys)//65)]:a.add_patch(Polygon(p,fill=False,edgecolor='#177d62',lw=.7,alpha=.65))
 a.scatter(*d['start'][:2],c='#159649',s=65,zorder=5);a.scatter(*d['goal'][:2],c='#df3535',s=130,marker='*',zorder=5);a.set_aspect('equal');a.set_xlabel('Map X (m)');a.set_ylabel('Map Y (m)')
ax.set_xlim(path[:,0].min()-1,path[:,0].max()+1);ax.set_ylim(path[:,1].min()-1,path[:,1].max()+1);ax.set_title('Full path and footprint sweep');ax.legend(handles=[Patch(facecolor='#74b9ef',label='Footprint swept area'),Patch(facecolor='none',edgecolor='#177d62',label='Body pose outlines')],loc='upper right')
axes[1].set_xlim(-6.8,-1.1);axes[1].set_ylim(-7.2,-3.8);axes[1].set_title('Departure / corridor turns');axes[2].set_xlim(-18.2,-14.4);axes[2].set_ylim(-.7,2.8);axes[2].set_title('Other-room entrance / arrival')
fig.suptitle('Smac Lattice — path including physical robot footprint',fontsize=20,y=.98);fig.text(.07,.935,'Body 47.7 × 65.5 cm | extra margin 0 | padding 0 | 20.64 m path | destination chair removed',fontsize=12);fig.text(.07,.028,'Blue area: all path poses + interpolated footprint samples (≤1 cm translation / ≤1° rotation). Static map planning; driving not yet validated.',fontsize=10);fig.subplots_adjust(top=.88,bottom=.085,left=.06,right=.98);dest=O/'lattice-footprint-movement-path.png';fig.savefig(dest,dpi=180);(O/'footprint-path-render.json').write_text(json.dumps({'source':'results.json','body_polygon_m':body.tolist(),'path_poses':len(path),'rendered_poses':len(poses),'interpolation_translation_m':.01,'interpolation_angle_deg':1,'note':'sampled sweep visualization, not continuous collision proof'},indent=2));print(dest)
