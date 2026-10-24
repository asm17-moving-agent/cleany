"""GIF from recorded Gazebo ground truth, not a predicted trajectory or GUI capture."""
from pathlib import Path
import argparse,json,math
import numpy as np
from PIL import Image
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation,PillowWriter
import yaml
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();r=a.run
result=json.loads((r/'result.json').read_text());rows=np.genfromtxt(r/'samples.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
start=result.get('navigation_sim_start_s',0);end=result.get('navigation_sim_end_s',rows['sim_s'][-1]);rows=rows[(rows['sim_s']>=start)&(rows['sim_s']<=end)]
idx=np.unique(np.linspace(0,len(rows)-1,min(150,len(rows))).astype(int));mapcfg=yaml.safe_load((r/'map.yaml').read_text());im=np.asarray(Image.open(r/'map.pgm'));res=mapcfg['resolution'];ox,oy,_=mapcfg['origin'];extent=[ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res]
goal=result['goal_map'];fp=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
stages=json.loads((r/'scenario.json').read_text()).get('approach_stages',[])
fig,axs=plt.subplots(1,2,figsize=(11,5),constrained_layout=True);lines=[];patches=[];dots=[]
for ax in axs:
 ax.imshow(im,cmap='gray',origin='upper',extent=extent,vmin=0,vmax=255)
 ax.scatter(goal[0],goal[1],marker='*',c='green',s=110,label='Goal')
 if stages:ax.scatter([s['pose'][0] for s in stages],[s['pose'][1] for s in stages],marker='x',c='purple',s=18,label='Intermediate goals')
 ln,=ax.plot([],[],c='royalblue',lw=1.5,label='Gazebo ground truth');lines.append(ln)
 patch=Polygon(fp,fill=False,edgecolor='orange',lw=2);ax.add_patch(patch);patches.append(patch)
 ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('map/world x (m)');ax.set_ylabel('y (m)')
axs[0].set_xlim(min(min(rows['gt_x']),goal[0])-1.2,max(max(rows['gt_x']),goal[0])+1.2);axs[0].set_ylim(min(min(rows['gt_y']),goal[1])-1.2,max(max(rows['gt_y']),goal[1])+1.2);axs[0].legend(loc='lower right',fontsize=7)
text=fig.suptitle('')
def update(j):
 i=idx[j];row=rows[i];x,y,yaw=row['gt_x'],row['gt_y'],row['gt_yaw'];c,s=np.cos(yaw),np.sin(yaw);body=fp@np.array([[c,s],[-s,c]])+[x,y]
 for ln,patch in zip(lines,patches):ln.set_data(rows['gt_x'][:i+1],rows['gt_y'][:i+1]);patch.set_xy(body)
 axs[1].set_xlim(x-1.5,x+1.5);axs[1].set_ylim(y-1.5,y+1.5)
 text.set_text(f'Gazebo recorded trajectory | {result["goal_name"]} | t={row["sim_s"]-start:.1f} sim s | final Nav2 status={result.get("action_status")}')
 return lines+patches+[text]
movie=FuncAnimation(fig,update,frames=len(idx),interval=80,blit=False);movie.save(r/'drive.gif',writer=PillowWriter(fps=12));update(len(idx)-1);fig.savefig(r/'drive.png',dpi=150);plt.close(fig)
(r/'drive-media.json').write_text(json.dumps({'kind':'Gazebo GT data replay over actual navigation map','gui_capture':False,'frames':len(idx),'sim_start':float(start),'sim_end':float(end),'playback_fps':12},indent=2))
