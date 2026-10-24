from pathlib import Path
import json, math, hashlib, shutil
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
R=Path(__file__).resolve().parent
stock=Path('/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json')
shutil.copy2(stock,R/'existing-stock-144.json')
shutil.copy2(R.parent/'lattice-straight-improvement-20261016/grid5cm.json',R/'existing-tuned-272.json')
paths={'stock':R/'existing-stock-144.json','tuned':R/'existing-tuned-272.json','new':R/'omni-r10cm/output.json'}
data={k:json.loads(p.read_text()) for k,p in paths.items()}
def category(p):
 if p['arc_length']>=.001:return 'curve'
 if math.hypot(*p['poses'][-1][:2])<1e-7:return 'in_place_rotation'
 return 'translation'
def summary(d):
 a=[p for p in d['primitives'] if category(p)=='curve']
 trans=[p for p in d['primitives'] if category(p)=='translation']
 m=d['lattice_metadata'];assert m['number_of_trajectories']==len(d['primitives'])
 assert len({p['trajectory_id'] for p in d['primitives']})==len(d['primitives'])
 grid=m['grid_resolution']
 for p in d['primitives']:
  assert all(abs(v/grid-round(v/grid))<.002 for v in p['poses'][-1][:2])
  assert p['start_angle_index']<m['num_of_headings'] and p['end_angle_index']<m['num_of_headings']
 return {'count':len(d['primitives']),'categories':dict(Counter(category(p) for p in d['primitives'])),'metadata':m,'curve_total_length_cm':[min(p['trajectory_length'] for p in a)*100,max(p['trajectory_length'] for p in a)*100],'curve_radius_cm':[min(p['trajectory_radius'] for p in a)*100,max(p['trajectory_radius'] for p in a)*100],'translation_endpoint_distance_cm':sorted(set(round(math.hypot(*p['poses'][-1][:2])*100,3) for p in trans))}
summaries={k:summary(d) for k,d in data.items()}
oldcolor='#2563eb';newcolor='#ea580c'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(15,13),dpi=180)
fig.subplots_adjust(top=.85,bottom=.13,wspace=.23,hspace=.28)
def draw(ax,d,color,heading=None,old=False):
 for p in d['primitives']:
  if heading is not None and p['start_angle_index']!=heading:continue
  if category(p)=='in_place_rotation':continue
  xy=np.array([[0,0]]+[v[:2] for v in p['poses']])*100
  ax.plot(xy[:,0],xy[:,1],color=color,alpha=.68 if heading is not None else .30,lw=1.9 if heading is not None else 1.05,linestyle='--' if old else '-')
  if heading is not None:ax.scatter(*xy[-1],s=15,color=color,zorder=4)
for row,key in enumerate(['stock','tuned']):
 for col in range(2):
  ax=axs[row,col];heading=0 if col==0 else None
  draw(ax,data[key],oldcolor,heading,True);draw(ax,data['new'],newcolor,heading)
  ax.set_aspect('equal');ax.set_xlim(-18,78) if col==0 else ax.set_xlim(-78,78);ax.set_ylim(-78,78)
  ax.set_xticks(np.arange(-10,81,10) if col==0 else np.arange(-75,76,25));ax.set_yticks(np.arange(-75,76,25));ax.grid(alpha=.18)
  ax.axhline(0,color='#94a3b8',lw=.7);ax.axvline(0,color='#94a3b8',lw=.7);ax.plot(0,0,'ko',ms=4)
  ax.set_xlabel('x displacement (cm)');ax.set_ylabel('y displacement (cm)')
  title=f"{'Stock 144' if key=='stock' else 'Previously tuned 272'} vs regenerated {summaries['new']['count']}"
  ax.set_title(title+'\n'+('Start heading 0 deg (facing +x)' if col==0 else 'All 16 start headings overlaid'),loc='left',fontweight='bold')
  if col==0:
   ins=ax.inset_axes([.53,.04,.44,.30]);draw(ins,data[key],oldcolor,0,True);draw(ins,data['new'],newcolor,0)
   ins.set_xlim(-7,26);ins.set_ylim(-17,17);ins.set_aspect('equal');ins.grid(alpha=.18);ins.tick_params(labelsize=7);ins.set_title('Origin detail',fontsize=9)
legend=[Line2D([0],[0],color=oldcolor,lw=2,ls='--',label='Existing: radius setting 50 cm'),Line2D([0],[0],color=newcolor,lw=2,label='Official regeneration: radius setting 10 cm')]
fig.legend(handles=legend,loc='upper center',bbox_to_anchor=(.5,.935),ncol=2,frameon=False,fontsize=12)
fig.suptitle('Lattice motion primitives | same origin, same scale',fontsize=21,fontweight='bold',y=.98)
n=summaries['new'];c=n['categories'];txt=(f"NEW: {n['count']} primitives = {c.get('curve',0)} curves + {c.get('translation',0)} translations + {c.get('in_place_rotation',0)} in-place rotations\n"
 f"Official Nav2 1.1.20 generator (unmodified) | Omni | 5 cm grid | 16 headings | stopping threshold 5\n"
 "10 cm is an experimental curve-radius setting, not a measured chassis minimum. No primitives manually added or removed.\n"
 "Lines show robot-center motion. In-place rotations share the origin and have no XY trace. No footprint/collision/drive validation here.")
fig.text(.06,.025,txt,fontsize=10,linespacing=1.6,color='#334155')
fig.savefig(R/'primitive-overlay-before-after.png',facecolor='white')
# A separate uncluttered close-up at a single heading.
f,ax=plt.subplots(figsize=(9,8),dpi=180)
for k,color,style in [('tuned',oldcolor,'--'),('new',newcolor,'-')]:draw(ax,data[k],color,0,k!='new')
ax.set_xlim(-10,75);ax.set_ylim(-45,45);ax.set_aspect('equal');ax.set_xticks(np.arange(-10,76,5));ax.set_yticks(np.arange(-45,46,5));ax.grid(alpha=.2);ax.set_xlabel('x displacement (cm)');ax.set_ylabel('y displacement (cm)');ax.set_title('Start heading 0 deg: tuned 272 vs official regeneration',fontweight='bold');ax.legend(handles=legend,loc='upper left',fontsize=9);f.tight_layout();f.savefig(R/'primitive-overlay-heading0.png',facecolor='white')
report={'source':'https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_smac_planner/lattice_primitives','generator_modified':False,'summaries':summaries,'sha256':{k:hashlib.sha256(p.read_bytes()).hexdigest() for k,p in paths.items()},'generator_source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (R/'official-generator').iterdir() if p.is_file()},'scope':'Static primitive regeneration and visualization only; no planner-default changes, no collision/kinematic-speed or Gazebo validation','radius_rationale':'Omni chassis supports lateral/in-place motions; 0.1 m is an exploratory positive radius at twice 0.05 m grid, not a physical minimum inferred from chassis dimensions.'}
(R/'comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(summaries,indent=2))
