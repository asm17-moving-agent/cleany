import json,csv,math
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
rows=json.loads((ROOT/'results.json').read_text());assert len(rows)==8
fig,axes=plt.subplots(2,4,figsize=(20,10),layout='constrained')
for row,ax in zip(sorted(rows,key=lambda r:r['goal_index']),axes.flat):
 run=Path('/home/donglee/workspace/cleany')/row['run_directory'];meta=yaml.safe_load((run/'map.yaml').read_text());image=plt.imread(run/'map.pgm');o=meta['origin'];res=meta['resolution'];height,width=image.shape[:2]
 ax.imshow(image,origin='upper',extent=[o[0],o[0]+width*res,o[1],o[1]+height*res],cmap='gray',vmin=0,vmax=255)
 data=list(csv.DictReader((run/'samples.csv').open()));a=np.array([[float(r['gt_x']),float(r['gt_y'])] for r in data if row.get('navigation_sim_start_s') is not None and row['navigation_sim_start_s']<=float(r['sim_s'])<=row['navigation_sim_end_s']])
 scenario=json.loads((run/'scenario.json').read_text());start=scenario['initial_pose'];goal=scenario['common_goal']['pose_map_world']
 if len(a):ax.plot(a[:,0],a[:,1],color='#007c91',lw=2,label='GT trajectory')
 ax.scatter([start[0]],[start[1]],marker='o',c='#3169bb',s=30,label='Start');ax.scatter([goal[0]],[goal[1]],marker='*',c='#ce582a',s=90,label='Goal')
 ax.arrow(goal[0],goal[1],.35*math.cos(goal[2]),.35*math.sin(goal[2]),color='#ce582a',width=.02,length_includes_head=True)
 extent=np.vstack((a,np.array([start[:2],goal[:2]]))) if len(a) else np.array([start[:2],goal[:2]]);lo=extent.min(axis=0)-1.2;hi=extent.max(axis=0)+1.2;ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');error_label=(f"GT end error {row['position_error_m']*100:.1f}cm" if row['arrived'] else f"GT remaining {row['position_error_m']:.2f}m");ax.set_title(f"{row['goal_index']}: {row['goal_name']} — {row['terminal_status']}\n{error_label} / {row['yaw_error_deg']:.2f}°",fontsize=11)
axes.flat[0].legend(fontsize=8,loc='best');fig.suptitle('TEB: locked 18F facility goals, one navigation attempt each',fontsize=16);fig.savefig(ROOT/'trajectories.png',dpi=170);fig.savefig(ROOT/'trajectories.pdf');plt.close(fig)
print(ROOT/'trajectories.png')
