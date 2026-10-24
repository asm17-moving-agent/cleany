from pathlib import Path
import json,math,yaml
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
O=Path(__file__).parent;d=json.loads((O/'results.json').read_text());m=yaml.safe_load(Path(d['map']).read_text());im=np.asarray(Image.open(Path(d['map']).parent/m['image']));res=m['resolution'];ox,oy,_=m['origin'];body=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
fig,axes=plt.subplots(2,3,figsize=(21,12));labels={'StraightFirst':'StraightFirst (reference only)','NavFn':'NavFn / Dijkstra','Smac2D':'Smac 2D / A*','ThetaStar':'Theta*','SmacLattice':'Smac Lattice / Omni','SmacHybrid':'Smac Hybrid / Reeds-Shepp'}
for ax,r in zip(axes.flat,d['results']):
 ax.imshow(im,cmap='gray',vmin=0,vmax=255,origin='upper',extent=(ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res));p=np.array(r['path']);polys=[]
 if len(p):
  samples=[]
  for a,b in zip(p,p[1:]):
   dyaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]));n=max(1,math.ceil(math.dist(a[:2],b[:2])/.02),math.ceil(abs(dyaw)/math.radians(2)))
   for t in np.arange(n)/n:samples.append([*(a[:2]+t*(b[:2]-a[:2])),a[2]+t*dyaw])
  samples.append(p[-1])
  for x,y,yaw in samples:
   c,s=math.cos(yaw),math.sin(yaw);polys.append(body@np.array([[c,s],[-s,c]])+np.array([x,y]))
  ax.add_collection(PolyCollection(polys,facecolors='#82c1ef',edgecolors='none',alpha=.13));ax.plot(p[:,0],p[:,1],c='#1464bc',lw=1.5)
  for poly in polys[::max(1,len(polys)//28)]:ax.add_patch(Polygon(poly,fill=False,edgecolor='#177d62',lw=.7,alpha=.75))
 else:ax.text(.5,.45,'No returned path',transform=ax.transAxes,ha='center',fontsize=18,color='#bf3535',bbox=dict(facecolor='white',alpha=.9))
 ax.scatter(*d['start'][:2],c='#159649',s=45,zorder=5);ax.scatter(*d['goal'][:2],c='#df3535',s=90,marker='*',zorder=5);ax.set_xlim(-18.5,-.8);ax.set_ylim(-7.5,3);ax.set_aspect('equal');ax.tick_params(labelsize=8);ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)')
 status=f'{r["wall_s"]:.3f} s | {r["path_length_m"]:.2f} m' if len(p) else f'No valid path / {r["wall_s"]:.3f} s'
 ax.set_title(labels[r['planner']]+'\n'+status,fontsize=13)
fig.suptitle('OTHER ROOM / 6 PLANNERS — EXACT PHYSICAL FOOTPRINT',fontsize=22,y=.97);fig.text(.055,.925,'47.7 × 65.5 cm body | extra margin 0 | padding 0 | same 5 cm static map | destination chair removed',fontsize=13);fig.text(.055,.056,'Blue band: sampled footprint sweep using returned path yaw; green outlines: body poses. 2D paths do not guarantee oriented-body feasibility.',fontsize=11);fig.text(.055,.03,'Custom/Smac time budget 80 s; Lattice/Hybrid iteration limit disabled. NavFn/Theta native termination. Static planning only; CPU timing includes concurrent Gazebo load.',fontsize=10);fig.subplots_adjust(left=.055,right=.985,top=.86,bottom=.12,wspace=.2,hspace=.36);fig.savefig(O/'six-planners-footprint-2x3.png',dpi=160);print(O/'six-planners-footprint-2x3.png')
