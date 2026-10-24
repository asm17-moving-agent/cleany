from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).resolve().parent
d=json.loads((R/'primitives.json').read_text());heads=d['lattice_metadata']['heading_angles']
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax in axs:
 ax.set_aspect('equal');ax.set_xlim(-.75,.75);ax.set_ylim(-.75,.75);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid(alpha=.25);ax.scatter(0,0,c='black',s=35,zorder=10)
for p in d['primitives']:
 h=p['start_angle_index'];q=np.vstack(([0,0,heads[h]],p['poses']));c=plt.cm.hsv(h/16)
 axs[0].plot(q[:,0],q[:,1],color=c,lw=.9,alpha=.65)
 axs[0].scatter(*q[-1,:2],color=c,s=10)
 if h==0:
  axs[1].plot(q[:,0],q[:,1],lw=2,color='#1674aa');x,y,a=q[-1];axs[1].arrow(x,y,.085*np.cos(a),.085*np.sin(a),head_width=.022,color='#1674aa',length_includes_head=True)
for h in heads:
 axs[0].arrow(0,0,.1*np.cos(h),.1*np.sin(h),width=.002,head_width=.018,color='black',alpha=.6,length_includes_head=True)
axs[1].arrow(0,0,.2,0,width=.006,head_width=.035,color='black',length_includes_head=True)
axs[0].set_title('All 16 START headings overlaid\n144 stored primitives; color = start heading')
axs[1].set_title('ONE start heading: body faces right (0 deg)\n9 stored primitives; arrows = end body heading')
axs[0].text(0,-.69,'Different initial body orientations share the same origin',ha='center',fontsize=9)
axs[1].text(0,-.69,'No fixed-body-heading diagonal translation primitive',ha='center',fontsize=9)
fig.suptitle('Our actual lattice: a radial picture does not mean all moves are available at one heading',fontsize=14)
fig.supxlabel('5 cm grid / omni / 0.5 m turning radius. Stored library only; runtime reverse expansion omitted.',fontsize=10)
fig.savefig(R/'heading_comparison.png',dpi=150)
