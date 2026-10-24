from pathlib import Path
import json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
R=Path(__file__).resolve().parent;O=R/'static-comparison';OLD=R.parent/'lattice-straight-improvement-20261016/static-only-20261016/recorded-start'
rows=json.loads((O/'comparison.json').read_text());routes=[(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')];variants=['previous272','official144_r10'];colors={'previous272':'#2563eb','official144_r10':'#ea580c'}
DATA={}
for i,name in routes:
 for v in variants:
  s=O/f'{i}-{name}-{v}';j=json.loads(s.with_suffix('.input.json').read_text());p=np.array(json.loads(s.with_suffix('.output.json').read_text())['poses']);rec=next(x for x in rows if x['route_index']==i+1 and x['variant']==v);DATA[i,v]=(j,p,rec)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
cmap=[]
for c in range(256):cmap.append('#fafafa' if c==0 else '#cbd5e1' if c==255 else '#292929' if c==254 else '#999999' if c==253 else (1, .98-.25*c/252,.98-.25*c/252))
cmap=ListedColormap(cmap)
def background(ax,i,name):
 j=DATA[i,variants[0]][0];p=np.concatenate([DATA[i,v][1] for v in variants]);origin=np.array(j['start'][:2]);a=np.fromfile(OLD/f'{i}-{name}.bin',dtype=np.uint8).reshape(j['ny'],j['nx']);ox,oy=j['origin'][:2];res=j['resolution'];ax.imshow(a,origin='lower',extent=[ox-origin[0],ox+res*j['nx']-origin[0],oy-origin[1],oy+res*j['ny']-origin[1]],cmap=cmap,vmin=0,vmax=255,interpolation='nearest');ax.set_xlim(p[:,0].min()-origin[0]-.55,p[:,0].max()-origin[0]+.55);ax.set_ylim(p[:,1].min()-origin[1]-.55,p[:,1].max()-origin[1]+.55);ax.set_aspect('equal');ax.set_xlabel('x from requested start (m)');ax.set_ylabel('y from requested start (m)');ax.grid(alpha=.12);return origin

def path(ax,i,v,origin,footprints=False):
 j,p,rec=DATA[i,v];c=colors[v];xy=p[:,:2]-origin;ax.plot(xy[:,0],xy[:,1],color=c,lw=2.1,zorder=5)
 if footprints:
  prev=None
  for pose in p:
   if prev is not None and math.dist(pose[:2],prev[:2])<.18 and abs(math.atan2(math.sin(pose[2]-prev[2]),math.cos(pose[2]-prev[2])))<.25:continue
   fp=np.array(j['footprint']);t=pose[2];rot=np.array([[math.cos(t),-math.sin(t)],[math.sin(t),math.cos(t)]]);corners=fp@rot.T+pose[:2]-origin;ax.add_patch(Polygon(corners,facecolor='none',edgecolor=c,alpha=.25,lw=.65));prev=pose
 for pose in [p[0],p[-1]]:
  x,y=pose[:2]-origin;ax.plot(x,y,'o',color=c,ms=3);ax.arrow(x,y,.16*math.cos(pose[2]),.16*math.sin(pose[2]),width=.008,color=c,length_includes_head=True,zorder=6)
 return rec
for footprint in [False,True]:
 f,axs=plt.subplots(2,4,figsize=(19,10),dpi=160)
 f.subplots_adjust(top=.84,bottom=.16,wspace=.35,hspace=.4)
 for col,(i,name) in enumerate(routes):
  for row,v in enumerate(variants):
   ax=axs[row,col];origin=background(ax,i,name);r=path(ax,i,v,origin,footprint)
   ax.set_title(f'{i+1}. {name} | '+('previous 272' if row==0 else 'official 144, R=10 cm')+f'\nDeviation {r["max_cross_track_cm"]:.2f} cm | length {r["length_m"]:.3f} m\nTotal planned yaw change {r["yaw_variation_deg"]:.1f} deg',loc='left',fontsize=10,color=colors[v])
 f.suptitle('Static Lattice plans | identical recorded start, goal, costmap and planner settings',fontsize=17,fontweight='bold',y=.97)
 f.text(.5,.91,'TOP: previous tuned 272 primitives     BOTTOM: official regenerated 144 primitives',ha='center',fontsize=12)
 f.text(.035,.035,'Native Nav2 1.1.20 A* + polygon collision checker; 5 cm map, padding 0. Arrows show start/end planned yaw.\n'+('Outlines use actual planned yaw; sampled outlines do not prove continuous swept-volume clearance. ' if footprint else '')+'Gray: obstacles. Pink: inflated costs. No MPPI/Gazebo driving.',fontsize=10)
 f.savefig(O/('static-before-after-footprints.png' if footprint else 'static-before-after-centerlines.png'),facecolor='white');plt.close(f)
f,axs=plt.subplots(1,4,figsize=(19,7),dpi=160);f.subplots_adjust(top=.8,bottom=.13,wspace=.35)
for ax,(i,name) in zip(axs,routes):
 origin=background(ax,i,name)
 for v in variants:path(ax,i,v,origin)
 ax.set_title(f'{i+1}. {name}',loc='left',fontweight='bold')
f.suptitle('Static paths overlaid | blue: previous 272, orange: regenerated 144',fontsize=17,fontweight='bold');f.text(.035,.04,'Centerlines and start/end yaw only. Same inputs; no hand-drawn replacement or smoothing. No driving.',fontsize=11);f.savefig(O/'static-overlay-centerlines.png',facecolor='white')
# Confirm control variables and endpoint equality per route.
checks=[]
for i,name in routes:
 a=DATA[i,variants[0]][0];b=DATA[i,variants[1]][0];assert {k:v for k,v in a.items() if k!='primitive_file'}=={k:v for k,v in b.items() if k!='primitive_file'};assert np.allclose(DATA[i,variants[0]][1][[0,-1]],DATA[i,variants[1]][1][[0,-1]])
 checks.append({'route':i+1,'identical_inputs_except_primitive':True,'identical_quantized_endpoints':True})
(O/'validation.json').write_text(json.dumps(checks,indent=2));print(O)
