"""Render recorded GT only: individual replays plus a common-time 8-panel GIF."""
from pathlib import Path
import concurrent.futures,subprocess,sys,json
import numpy as np,yaml
from PIL import Image
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation,PillowWriter
root=Path(__file__).resolve().parent
runs=sorted((root/'diverse8').glob('*/metrics.json'))
def individual(p):
 d=p.parent
 if not (d/'drive.gif').exists():subprocess.run([sys.executable,str(root.parent/'render_drive.py'),str(d)],check=True)
 return d.name
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
 for name in pool.map(individual,runs):print('individual',name,flush=True)
fig,axs=plt.subplots(2,4,figsize=(16,10),constrained_layout=True)
fp=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
data=[]
for ax,p in zip(axs.flat,runs):
 d=p.parent;m=json.loads(p.read_text());result=json.loads((d/'result.json').read_text());cfg=yaml.safe_load((d/'map.yaml').read_text());im=np.asarray(Image.open(d/'map.pgm'));ox,oy,_=cfg['origin'];res=cfg['resolution']
 rows=np.genfromtxt(d/'samples.csv',delimiter=',',names=True);start=result['navigation_sim_start_s'];end=result['navigation_sim_end_s'];rows=rows[(rows['sim_s']>=start)&(rows['sim_s']<=end)]
 ts=rows['sim_s']-start;goal=result['goal_map'];x,y=rows['gt_x'],rows['gt_y']
 ax.imshow(im,cmap='gray',origin='upper',extent=[ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res],vmin=0,vmax=255)
 ax.scatter(goal[0],goal[1],marker='*',c='green',s=80,zorder=4);ax.scatter(x[0],y[0],c='darkorange',s=16)
 line,=ax.plot([],[],c='royalblue',lw=1.5);body=Polygon(fp,fill=False,edgecolor='darkorange',lw=1.5);ax.add_patch(body)
 ax.set_xlim(min(x.min(),goal[0])-.8,max(x.max(),goal[0])+.8);ax.set_ylim(min(y.min(),goal[1])-.8,max(y.max(),goal[1])+.8);ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.tick_params(labelsize=8)
 title=ax.set_title('',fontsize=10);data.append((rows,ts,line,body,title,m,end-start))
max_t=max(d[-1] for d in data);times=np.r_[np.linspace(0,max_t,240),np.full(24,max_t)]
suptitle=fig.suptitle('',fontsize=14)
def update(frame):
 t=times[frame];suptitle.set_text(f'Recorded Gazebo GT | Common elapsed simulation time: {t:.1f} s\nSame MPPI settings, no intermediate goals | Blue: GT trail, orange: footprint, green: goal')
 for rows,ts,line,body,title,m,duration in data:
  i=min(np.searchsorted(ts,t,side='right')-1,len(rows)-1);i=max(0,i);row=rows[i];yaw=row['gt_yaw'];c,s=np.cos(yaw),np.sin(yaw)
  line.set_data(rows['gt_x'][:i+1],rows['gt_y'][:i+1]);body.set_xy(fp@np.array([[c,s],[-s,c]])+[row['gt_x'],row['gt_y']])
  status='PASS' if t>=duration else 'RUNNING'
  title.set_text(f'{m["goal_index"]}: {m["goal_name"]} | {status}\n'+(f'Final GT {m["position_error_m"]*100:.1f} cm / {m["yaw_error_deg"]:.2f} deg' if t>=duration else f't={min(t,duration):.1f} sim s'))
ani=FuncAnimation(fig,update,frames=len(times),interval=1000/12,blit=False)
ani.save(root/'eight-routes.gif',writer=PillowWriter(fps=12),dpi=85)
for frame,name in [(0,'start'),(120,'middle'),(len(times)-1,'final')]:
 update(frame);fig.savefig(root/f'gif-preview-{name}.png',dpi=85)
plt.close(fig)
meta={'kind':'Recorded Gazebo GT replay, not GUI screen capture','shared_elapsed_time':True,'frames':len(times),'fps':12,'simulation_duration_s':max_t,'motion_playback_speed_sim_s_per_wall_s':max_t/(239/12),'final_hold_s':2,'individual_gifs':[str(p.parent.relative_to(root)/'drive.gif') for p in runs]}
(root/'gifs.json').write_text(json.dumps(meta,indent=2));print('combined complete',flush=True)
