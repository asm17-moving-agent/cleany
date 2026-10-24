from pathlib import Path
import json,math,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle
r=Path(__file__).parent;o=r/'actual-path-snapshot';d=json.loads((o/'snapshot.json').read_text());p=np.array(json.loads((o/'actual-path.json').read_text())['path']);a=np.load(o/'local-costmap.npy');v=json.loads((r/'stall-diagnosis/native-maneuver-check-result.json').read_text());origin=np.array(d['origin']);res=d['resolution'];b=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
def poly(p):
 c,s=np.cos(p[2]),np.sin(p[2]);return b@np.array([[c,s],[-s,c]])+p[:2]
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax in axs:
 ax.imshow(a,origin='lower',extent=[origin[0],origin[0]+a.shape[1]*res,origin[1],origin[1]+a.shape[0]*res],cmap='YlOrRd',vmin=0,vmax=254,alpha=.75)
 y,x=np.where(a==254);ax.scatter(origin[0]+(x+.5)*res,origin[1]+(y+.5)*res,c='#252525',s=35,marker='s')
 ax.plot(p[:,0],p[:,1],'.-',c='#057a58',label='Actual MPPI input path: 29 poses')
 ax.add_patch(Polygon(poly(np.array(d['robot_pose'])),fill=False,color='#1575cf',lw=2.5,label='Current robot footprint'))
 ax.set_aspect('equal');ax.set_xlabel('odom X (m)');ax.set_ylabel('odom Y (m)')
for pose in p[:9]:axs[0].add_patch(Polygon(poly(pose),fill=False,color='#057a58',lw=.7,alpha=.7))
for row in v['rotation_rows']:
 if row['cost']==254:axs[1].add_patch(Polygon(poly(np.array(row['pose'])),fill=False,color='#df324c',lw=1.1,alpha=.8))
for row in v['translation_rows'][::3]:
 if row['cost']==254:axs[1].add_patch(Polygon(poly(np.array(row['pose'])),fill=False,color='#e8831b',lw=.8,alpha=.6))
axs[0].set_xlim(5.15,6.6);axs[0].set_ylim(14.7,16);axs[0].set_title('Native Nav2 check: path + connector clear\n29 path samples / 244 interpolated samples');axs[0].legend(fontsize=8)
axs[1].set_xlim(5.25,6.35);axs[1].set_ylim(14.85,15.85);axs[1].set_title('Native Nav2 check: simple maneuvers collide\nRed: rotate here (8 / 31), orange: keep yaw (35 / 244)')
fig.suptitle('Doorway stall: entering requires coordinated translation and rotation',fontsize=14)
fig.savefig(r/'stall-diagnosis/doorway-stall.png',dpi=170)
