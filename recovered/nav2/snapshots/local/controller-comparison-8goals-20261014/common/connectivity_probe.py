from pathlib import Path
import json,math
import numpy as np
from scipy.ndimage import binary_dilation,label,distance_transform_edt
from PIL import Image,ImageDraw
from prepare_common import ROOT,RES,BOUNDS
p=ROOT/'goals/7_other_room';image=np.array(Image.open(p/'map.pgm'));obs=(image==0);mask0=np.ones((19,15),bool);mask90=np.ones((15,19),bool);free0=~binary_dilation(obs,mask0);free90=~binary_dilation(obs,mask90);free=free0|free90;labs,num=label(free,np.ones((3,3)));start=[-1.865,-4.705];goal=[-16.775,1.775];xmin,ymin,xmax,ymax=BOUNDS
def px(v):return [int((ymax-v[1])/RES),int((v[0]-xmin)/RES)]
a,b=px(start),px(goal);print('start/goal components',labs[tuple(a)],labs[tuple(b)],'center-only labels',label(~obs,np.ones((3,3)))[0][tuple(a)],label(~obs,np.ones((3,3)))[0][tuple(b)]);dist=distance_transform_edt(~obs)*RES;print('goal nearest obstacle',dist[tuple(b)]);out=Image.fromarray(np.stack([image]*3,axis=2));draw=ImageDraw.Draw(out)
for color,c in [('red',a),('blue',b)]:draw.ellipse([c[1]-4,c[0]-4,c[1]+4,c[0]+4],fill=color)
out.save(p/'map.preflight.png');(ROOT/'preflight.connectivity.json').write_text(json.dumps({'scope':'2D discrete rectangular footprint orientations0/90 union, lower bound only; does not certify transitions','start_label':int(labs[tuple(a)]),'goal_label':int(labs[tuple(b)]),'start_center_free_label':int(label(~obs,np.ones((3,3)))[0][tuple(a)]),'goal_center_free_label':int(label(~obs,np.ones((3,3)))[0][tuple(b)]),'goal_center_obstacle_distance_m':float(dist[tuple(b)])},indent=2))
