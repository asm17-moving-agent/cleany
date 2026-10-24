from pathlib import Path
import json, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image
from io import BytesIO
R=Path(__file__).resolve().parent
src=Path('/opt/ros/humble/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json')
d=json.loads(src.read_text());(R/'primitives.json').write_text(src.read_text())
ps=[p for p in d['primitives'] if p['start_angle_index']==0]
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False})
colors=plt.cm.tab10(np.arange(9)); labels=['Right wide turn','Right shallow turn','Rotate right','Forward','Strafe left','Strafe right','Left shallow turn','Rotate left','Left wide turn']
fp=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
def path(p):return np.vstack(([0,0,d['lattice_metadata']['heading_angles'][p['start_angle_index']]],p['poses']))
def setup(ax):
 ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.set_xticks(np.arange(-.5,1.51,.05),minor=True);ax.set_yticks(np.arange(-1,1.51,.05),minor=True);ax.grid(which='minor',alpha=.12);ax.grid(alpha=.25)
def arrow(ax,x,y,yaw,c,length=.065):ax.arrow(x,y,length*np.cos(yaw),length*np.sin(yaw),width=.003,head_width=.022,color=c,length_includes_head=True,zorder=5)
fig,axs=plt.subplots(2,2,figsize=(13,10),layout='constrained');a,b,c,e=axs.flat
for i,p in enumerate(ps):
 q=path(p)
 if np.ptp(q[:,:2],axis=0).max()>0:
  a.plot(q[:,0],q[:,1],color=colors[i],lw=2,label=f'{i}: {labels[i]}');a.scatter(*q[-1,:2],color=colors[i],s=20);arrow(a,*q[-1],colors[i]);a.text(q[-1,0]+.02,q[-1,1]+.015,str(i),color=colors[i],weight='bold')
a.scatter(0,0,color='black',s=35);a.set_title('1. Stored translation / turn primitives | heading 0 deg');setup(a);a.set_xlim(-.17,.7);a.set_ylim(-.48,.48);a.legend(fontsize=8,loc='upper left')
for i in [2,7]:
 q=path(ps[i]);theta=q[:,2];b.plot(.22*np.cos(theta),.22*np.sin(theta),c=colors[i],lw=3,label=f'{i}: {labels[i]}');arrow(b,0,0,theta[-1],colors[i],.32)
b.add_patch(Polygon(fp,fill=False,edgecolor='gray',ls='--'));arrow(b,0,0,0,'black',.32);b.scatter(0,0,c='black');setup(b);b.set_xlim(-.4,.45);b.set_ylim(-.4,.4);b.set_title('2. In-place rotations | same (x, y), different yaw');b.legend(fontsize=9,loc='lower left')
for j,h in enumerate(d['lattice_metadata']['heading_angles']):
 c.plot([0,np.cos(h)],[0,np.sin(h)],color='#407cad',alpha=.8);c.text(1.13*np.cos(h),1.13*np.sin(h),f'{np.degrees(h):.1f}',ha='center',va='center',fontsize=8)
c.set_aspect('equal');c.set_xlim(-1.4,1.4);c.set_ylim(-1.4,1.4);c.axis('off');c.set_title('3. 16 heading bins (not equally spaced)');c.text(0,-1.4,'144 stored primitives total; 9 per heading',ha='center',fontsize=10)
for p in ps:
 q=path(p);e.plot(q[:,0],q[:,1],c='#1768ac',lw=2)
 for n in d['primitives']:
  if n['start_angle_index']==p['end_angle_index']:
   z=path(n);e.plot(z[:,0]+q[-1,0],z[:,1]+q[-1,1],c='#b0bcc9',alpha=.65,lw=.8)
e.scatter(0,0,c='black',zorder=5);setup(e);e.set_xlim(-.45,1.15);e.set_ylim(-.95,.95);e.set_title('4. Expand again from endpoint heading (schematic)')
fig.suptitle('Actual Smac Lattice primitive library: 5 cm grid / 0.5 m turning radius / omni',fontsize=16)
fig.supxlabel('Stored primitives only. Reverse expansion is enabled in the run but omitted here.\nNo obstacles, collision rejection, heuristic ordering or analytic expansion shown.',fontsize=10)
fig.savefig(R/'lattice_candidates.png',dpi=150);plt.close(fig)
frames=[]
for i,p in enumerate(ps):
 q=path(p)
 for k in range(13):
  idx=min(int(k/10*(len(q)-1)),len(q)-1);x,y,h=q[idx];rot=np.array([[np.cos(h),-np.sin(h)],[np.sin(h),np.cos(h)]])
  fig,ax=plt.subplots(figsize=(7,6),layout='constrained')
  for pp in ps:
   z=path(pp);ax.plot(z[:,0],z[:,1],c='#c8d0d8',lw=1)
  ax.plot(q[:,0],q[:,1],c=colors[i],lw=3);ax.plot(q[:,0],q[:,1],'.',c=colors[i],ms=4)
  ax.add_patch(Polygon(fp@rot.T+[x,y],facecolor=colors[i],alpha=.18,edgecolor=colors[i]));arrow(ax,x,y,h,colors[i],.20);ax.scatter(x,y,c='black',s=20)
  setup(ax);ax.set_xlim(-.5,1.05);ax.set_ylim(-.85,.85);ax.set_title(f'Primitive {i}: {labels[i]}\nx={x:.2f} m, y={y:.2f} m, yaw={np.degrees(np.arctan2(np.sin(h),np.cos(h))):.1f} deg')
  fig.suptitle('Lattice candidates from (0, 0, 0 deg)',fontsize=15);fig.supxlabel('Actual stored poses + full robot footprint\nAnimation advances pose samples; it is not a speed profile.',fontsize=10)
  buf=BytesIO();fig.savefig(buf,format='png',dpi=95);plt.close(fig);buf.seek(0);frames.append(Image.open(buf).convert('RGB').quantize(colors=128, method=0, dither=0))
frames[0].save(R/'lattice_candidates.gif',save_all=True,append_images=frames[1:],duration=110,loop=0)
(R/'provenance.json').write_text(json.dumps({'source':str(src),'sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'metadata':d['lattice_metadata'],'display':'stored primitives; no actual A* expansion trace; reverse expansion omitted; GIF time not physical'},indent=2))
print(R)
