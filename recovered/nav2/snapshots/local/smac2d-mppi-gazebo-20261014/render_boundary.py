from pathlib import Path
import json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
out=Path(__file__).parent/'costmap-stall';d=json.loads((out/'snapshot.json').read_text());a=np.load(out/'local-costmap.npy');res=d['resolution'];ox,oy=d['origin'];poly=np.array(d['published_footprint']);pose=d['robot_pose'];path=np.array(d['plan_in_costmap_frame']);hits=np.array(list({tuple(h['center']) for h in d['rasterized_boundary_lethal_hits']}))
fig,axes=plt.subplots(1,2,figsize=(13,6));extent=[ox,ox+a.shape[1]*res,oy,oy+a.shape[0]*res]
for ax in axes:
 ax.imshow(np.where(a==255,np.nan,a),origin='lower',extent=extent,cmap='YlOrRd',vmin=0,vmax=254,interpolation='nearest');ly,lx=np.where(a==254);ax.scatter(ox+(lx+.5)*res,oy+(ly+.5)*res,c='black',s=12,marker='s',label='Lethal obstacle cells (254)')
 ax.add_patch(Polygon(poly,fill=False,edgecolor='#00a8e8',linewidth=2.5,label='Padded robot footprint'))
 if len(path):ax.plot(path[:,0],path[:,1],c='#247a32',lw=2,label='Smac 2D path')
 if len(hits):ax.scatter(hits[:,0],hits[:,1],s=160,facecolors='none',edgecolors='#ff00df',lw=2.5,label='Boundary collision cell')
 ax.arrow(pose[0],pose[1],.3*math.cos(pose[2]),.3*math.sin(pose[2]),head_width=.09,color='#0088cc');ax.set_aspect('equal');ax.set_xlabel('odom X (m)');ax.set_ylabel('odom Y (m)')
axes[0].set_xlim(ox,extent[1]);axes[0].set_ylim(oy,extent[3]);axes[0].set_title('Live local costmap');axes[0].legend(fontsize=8,loc='upper left');axes[1].set_xlim(pose[0]-1,pose[0]+1);axes[1].set_ylim(pose[1]-1,pose[1]+1);axes[1].set_title('Footprint corner intersects 1 lethal grid cell')
fig.suptitle('Smac 2D + MPPI / stalled robot: footprint boundary collision',fontsize=15);fig.tight_layout();fig.savefig(out/'costmap-stall.png',dpi=160)
