from pathlib import Path
import json,yaml,numpy as np
from PIL import Image
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
r=Path(__file__).resolve().parent
fig,axes=plt.subplots(2,4,figsize=(16,10),constrained_layout=True)
fp=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
for ax,p in zip(axes.flat,sorted((r/'diverse8').glob('*/metrics.json'))):
 d=p.parent;m=json.loads(p.read_text());res=json.loads((d/'result.json').read_text());cfg=yaml.safe_load((d/'map.yaml').read_text());im=np.asarray(Image.open(d/'map.pgm'));ox,oy,_=cfg['origin'];scale=cfg['resolution']
 samples=np.genfromtxt(d/'samples.csv',delimiter=',',names=True);samples=samples[(samples['sim_s']>=res['navigation_sim_start_s'])&(samples['sim_s']<=res['navigation_sim_end_s'])]
 x,y=samples['gt_x'],samples['gt_y'];goal=res['goal_map'];ax.imshow(im,cmap='gray',origin='upper',extent=[ox,ox+im.shape[1]*scale,oy,oy+im.shape[0]*scale],vmin=0,vmax=255)
 ax.plot(x,y,c='royalblue',lw=1.4);ax.scatter(x[0],y[0],c='orange',s=22,zorder=4);ax.scatter(goal[0],goal[1],marker='*',c='green',s=90,zorder=5)
 yaw=samples['gt_yaw'][-1];c,s=np.cos(yaw),np.sin(yaw);ax.add_patch(Polygon(fp@np.array([[c,s],[-s,c]])+[x[-1],y[-1]],fill=False,edgecolor='darkorange'))
 ax.set_xlim(min(x.min(),goal[0])-.8,max(x.max(),goal[0])+.8);ax.set_ylim(min(y.min(),goal[1])-.8,max(y.max(),goal[1])+.8);ax.set_aspect('equal');ax.grid(alpha=.15)
 ax.set_title(f'{m["goal_index"]}: {m["goal_name"]} — {"PASS" if m["success"] else "FAIL"}\nGT {m["position_error_m"]*100:.1f} cm / {m["yaw_error_deg"]:.2f} deg',fontsize=11);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)')
fig.suptitle('Eight original goals, one common MPPI configuration, no intermediate goals\nBlue: recorded Gazebo GT | Orange: start / final footprint | Green: original goal',fontsize=15)
fig.savefig(r/'eight-routes.png',dpi=140);plt.close(fig)
