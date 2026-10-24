from pathlib import Path
import json,math
import numpy as np,yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
O=Path(__file__).resolve().parent;data=json.loads((O/'results.json').read_text());assert len(data['results'])==6
m=yaml.safe_load(Path(data['map']).read_text());im=np.asarray(Image.open(Path(data['map']).parent/m['image']));res=m['resolution'];ox,oy,_=m['origin'];extent=(ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res)
start=data['start'];goal=data['goal'];points=[start[:2],goal[:2]]+[p[:2] for r in data['results'] for p in r['path']];xmin=min(p[0] for p in points)-1.3;xmax=max(p[0] for p in points)+1.3;ymin=min(p[1] for p in points)-1.;ymax=max(p[1] for p in points)+1.
labels={'StraightFirst':'StraightFirstPlanner (previous custom)','NavFn':'NavFn / Dijkstra','Smac2D':'Smac 2D / A*','ThetaStar':'Theta*','SmacLattice':'Smac Lattice / Omni','SmacHybrid':'Smac Hybrid-A* / Reeds-Shepp'}
fig,axes=plt.subplots(2,3,figsize=(19.8,10.4));fig.patch.set_facecolor('#f6f8fb')
for ax,r in zip(axes.flat,data['results']):
 ax.set_facecolor('#f6f8fb');ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=extent,origin='upper',interpolation='nearest');ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax);ax.set_aspect('equal');ax.tick_params(labelsize=8)
 success=r['status']==4 and bool(r['path']);color='#136fda' if success else '#c44437'
 if r['path']:
  path=np.array(r['path']);ax.plot(path[:,0],path[:,1],color=color,lw=2.2,zorder=3)
  if r['planner'] in ['StraightFirst','SmacLattice','SmacHybrid']:
   for p in path[::max(1,len(path)//20)]:ax.arrow(p[0],p[1],.22*math.cos(p[2]),.22*math.sin(p[2]),width=.02,head_width=.13,color=color,alpha=.65,zorder=4)
 ax.scatter([start[0]],[start[1]],s=55,c='#159b69',edgecolor='white',zorder=6,label='Start');ax.scatter([goal[0]],[goal[1]],s=100,c='#d24148',marker='*',edgecolor='white',zorder=6,label='Goal')
 ax.annotate('START',(start[0],start[1]),xytext=(5,-15),textcoords='offset points',fontsize=8,color='#066741',fontweight='bold');ax.annotate('GOAL / M1',(goal[0],goal[1]),xytext=(5,8),textcoords='offset points',fontsize=8,color='#a62a34',fontweight='bold')
 ax.set_title(labels[r['planner']],fontsize=13,fontweight='bold',loc='left',pad=33,color='#1c2b42')
 if success:detail=f"PATH FOUND  |  {r['path_length_m']:.2f} m  |  {r['wall_s']*1000:.1f} ms"
 elif r['status']==6:detail=f"NO PATH  |  {r['wall_s']:.2f} s  |  "+('iteration cap 1M' if r['planner'] in ['SmacLattice','SmacHybrid'] else '5s budget')
 else:detail='INFRASTRUCTURE ERROR: '+str(r.get('error'))
 ax.text(0,1.035,detail,transform=ax.transAxes,color=('#0d7951' if success else '#b33931'),fontsize=10,fontweight='bold')
 model='2D point/circular approximation; body orientation not checked' if r['planner'] in ['NavFn','Smac2D','ThetaStar'] else ('SE2 polygon + Omni primitives' if r['planner']=='SmacLattice' else ('SE2 polygon; car-like motion model, no lateral moves' if r['planner']=='SmacHybrid' else 'Custom straight/heading searches + footprint checks'))
 ax.text(0,-.15,model,transform=ax.transAxes,fontsize=8.6,color='#506078')
 if not success:ax.text(.5,.45,'No returned path',ha='center',va='center',transform=ax.transAxes,fontsize=15,color='#b33931',bbox={'facecolor':'white','edgecolor':'#e4b6af','alpha':.95,'boxstyle':'round,pad=.5'})
 ax.set_xlabel('World X (m)',fontsize=9);ax.set_ylabel('World Y (m)',fontsize=9)
fig.suptitle('OTHER ROOM / M1 — 6 PLANNERS, SAME STATIC MAP',x=.04,ha='left',y=.982,fontsize=21,fontweight='bold',color='#14263e')
fig.text(.04,.934,'Start (-1.865, -4.705, 90°) → goal (-16.775, 1.775, 90°) | destination chair removed | ROS 2 Humble / Nav2 1.1.20',fontsize=11,color='#52617a')
fig.text(.04,.034,'5 cm map | same footprint + 2 cm padding, inflation 0.8 m | fresh process per planner | 5 s search budget for Smac/custom',fontsize=10,color='#52617a')
fig.text(.04,.012,'Planning only. A returned 2D path does not establish rectangular-body feasibility or successful driving. No path within budget does not prove impossibility.',fontsize=10,color='#52617a')
fig.subplots_adjust(left=.04,right=.985,top=.875,bottom=.115,hspace=.55,wspace=.22)
fig.savefig(O/'other-room-planners-2rows-3cols.png',dpi=150,facecolor=fig.get_facecolor());print(O/'other-room-planners-2rows-3cols.png')
