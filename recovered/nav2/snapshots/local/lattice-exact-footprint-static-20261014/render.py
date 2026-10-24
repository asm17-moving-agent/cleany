from pathlib import Path
import json,math,yaml
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
O=Path(__file__).parent;d=json.loads((O/'results.json').read_text());r=d['results'][0];m=yaml.safe_load(Path(d['map']).read_text());im=np.asarray(Image.open(Path(d['map']).parent/m['image']));res=m['resolution'];ox,oy,_=m['origin'];p=np.array(r['path']);fig,ax=plt.subplots(figsize=(13,7));ax.imshow(im,cmap='gray',vmin=0,vmax=255,origin='upper',extent=(ox,ox+im.shape[1]*res,oy,oy+im.shape[0]*res));ax.plot(p[:,0],p[:,1],c='#1867cd',lw=2,label='Smac Lattice path')
body=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
for x,y,yaw in p[::max(1,len(p)//32)]:
 c,s=math.cos(yaw),math.sin(yaw);pts=body@np.array([[c,s],[-s,c]])+np.array([x,y]);ax.add_patch(Polygon(pts,fill=False,edgecolor='#2a9e78',lw=.9,alpha=.75));ax.arrow(x,y,.25*c,.25*s,color='#2a9e78',head_width=.09,length_includes_head=True)
ax.scatter(*d['start'][:2],c='green',s=90,label='Start');ax.scatter(*d['goal'][:2],c='red',s=160,marker='*',label='Other-room goal');ax.set_xlim(min(p[:,0])-1.2,max(p[:,0])+1.2);ax.set_ylim(min(p[:,1])-1.2,max(p[:,1])+1.2);ax.set_aspect('equal');ax.legend(loc='upper right');ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');ax.set_title(f'Smac Lattice / other room — path found in {r["wall_s"]:.2f} s, length {r["path_length_m"]:.2f} m\nPhysical body 47.7 × 65.5 cm | padding 0 | 5 cm map | stock omni primitives',fontsize=13);fig.text(.1,.035,'Static planning only. Iteration limit disabled; 80 s time budget. Destination chair removed. Green outlines show sampled body poses.',fontsize=9);fig.subplots_adjust(bottom=.13);fig.savefig(O/'lattice-other-room-exact-footprint.png',dpi=160)
(O/'REPORT.md').write_text(f'''# Smac Lattice static planning\n\n- Nav2 status: {r['status']} (SUCCEEDED)\n- Wall time: {r['wall_s']:.6f} s\n- Path length: {r['path_length_m']:.6f} m\n- Start: {d['start']}\n- Goal: {d['goal']}\n- Physical footprint: 0.477 × 0.6548 m, padding 0\n- Map: 5 cm static map, destination chair removed\n- Stock omni primitives: 16 headings, 144 trajectories\n- max_iterations=-1; max_planning_time=80 s\n- Both footprint and search limits changed; individual causal effects not isolated.\n- No driving performed in this test. Active Smac2D/MPPI Gazebo run kept separate.\n''')
print(O/'lattice-other-room-exact-footprint.png')
