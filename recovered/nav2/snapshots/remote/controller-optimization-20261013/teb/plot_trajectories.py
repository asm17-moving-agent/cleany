"""Standalone trajectory figure; distinct experimental conditions share axes."""
import csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
fig,axes=plt.subplots(1,2,figsize=(12,5),sharex=True,sharey=True)
colors=plt.cm.tab10.colors
for ax,stage,title in zip(axes,['resume_20261013_01','relaxed_20261013_01'],['Conservative: shared safety gates','Gazebo relaxed: coupled wheel limiter']):
 results=json.loads((ROOT/stage/'results.json').read_text())
 for r in sorted(results,key=lambda r:r['goal_index']):
  run=next(p.parent for p in (ROOT/stage/'final').glob('*/result.json') if json.loads(p.read_text()).get('goal_index')==r['goal_index'] and json.loads(p.read_text()).get('completed'))
  rows=list(csv.DictReader((run/'samples.csv').open()))
  rows=[x for x in rows if x['phase']=='comparison_navigation']
  c=colors[r['goal_index']]
  ax.plot([float(x['gt_x']) for x in rows],[float(x['gt_y']) for x in rows],color=c,linewidth=1.4,label=f"{r['goal_name']}: {r['terminal_status']}")
  gx,gy,_=r['goal_map'];ax.scatter(gx,gy,color=c,marker='*',s=120,zorder=3)
  x,y,_=r['final_truth'];ax.scatter(x,y,color=c,marker='o',s=24,zorder=3)
 ax.scatter(0,0,color='black',marker='+',s=80)
 ax.set_title(title);ax.set_aspect('equal');ax.grid(alpha=.2)
 ax.set_xlabel('Initial map frame x (m)');ax.legend(fontsize=7,loc='lower left')
axes[0].set_ylabel('Initial map frame y (m)')
fig.suptitle('TEB final five goals, once each — stars: goals; circles: terminal GT poses')
fig.tight_layout()
fig.savefig(ROOT/'final_trajectories.png',dpi=180)
fig.savefig(ROOT/'final_trajectories.pdf')
