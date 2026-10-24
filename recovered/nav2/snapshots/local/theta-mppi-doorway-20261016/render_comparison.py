"""Actual GT replay with recorded plans, fixed simulation-time 2x playback."""
from pathlib import Path
import json,csv,bisect,math
from PIL import Image,ImageDraw,ImageFont,ImageChops,GifImagePlugin
import yaml
R=Path(__file__).resolve().parent;O=R/'media';O.mkdir(exist_ok=True)
VS=['baseline','xy_only','clearance'];LABELS=['Before: Theta* + MPPI','Only disable path yaw tracking','Improved: path yaw off + traversal 5'];COLORS=[(180,75,50),(175,135,30),(30,115,200)];W,H=640,430
font='/usr/share/fonts/google-noto/NotoSans-Regular.ttf';F=ImageFont.truetype(font,16);FT=ImageFont.truetype(font,19)
D={};FP=[[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]]
for v in VS:
 d=R/v;r=json.loads((d/'result.json').read_text());rows=[]
 for x in csv.DictReader((d/'samples.csv').open()):
  t,px,py,yaw=[float(x[k]) for k in ['sim_s','gt_x','gt_y','gt_yaw']]
  if r['navigation_sim_start_s']<=t<=r['navigation_sim_end_s']:rows.append((t-r['navigation_sim_start_s'],px,py,yaw))
 plans=json.loads((d/'plans.json').read_text());D[v]=(r,rows,[x[0] for x in rows],plans,[p['sim_s'] for p in plans])
img=Image.open(R/'baseline/map.pgm').convert('L');cfg=yaml.safe_load((R/'baseline/map.yaml').read_text());ox,oy,_=cfg['origin'];res=cfg['resolution'];BG={}
for z,limits in enumerate([(-18.3,-.4,-7.8,3.0),(-7.5,-4.,-7.5,-4.)]):
 x0,x1,y0,y1=limits;scale=min((W-32)/(x1-x0),(H-132)/(y1-y0));pw,ph=round((x1-x0)*scale),round((y1-y0)*scale);left,top=(W-pw)//2,115+(H-126-ph)//2
 crop=img.crop((round((x0-ox)/res),round(img.height-(y1-oy)/res),round((x1-ox)/res),round(img.height-(y0-oy)/res))).point(lambda v:45 if v<100 else (220 if v<240 else 249)).convert('RGB').resize((pw,ph),Image.Resampling.NEAREST)
 def tr(x,y,x0=x0,y1=y1,scale=scale):return (x-x0)*scale,(y1-y)*scale
 BG[z]=crop,tr,left,top,limits

def cell(v,z,time):
 k=VS.index(v);r,rows,times,plans,stamps=D[v];duration=r['navigation_sim_s'];t=min(time,duration);j=min(max(1,bisect.bisect_left(times,t)),len(rows)-1);a,b=rows[j-1],rows[j];f=max(0,min(1,(t-a[0])/(b[0]-a[0]) if b[0]!=a[0] else 0));x=a[1]+f*(b[1]-a[1]);y=a[2]+f*(b[2]-a[2]);yaw=a[3]+f*math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]));bg,tr,left,top,limits=BG[z];tile=bg.copy();d=ImageDraw.Draw(tile);index=bisect.bisect_right(stamps,r['navigation_sim_start_s']+t)-1
 if index>=0 and len(plans[index]['poses'])>1:d.line([tr(*p[:2]) for p in plans[index]['poses']],fill=(164,85,185),width=1)
 points=[tr(p[1],p[2]) for p in rows[:j:3]]+[tr(x,y)]
 if len(points)>1:d.line(points,fill=COLORS[k],width=2)
 poly=[tr(x+u*math.cos(yaw)-w*math.sin(yaw),y+u*math.sin(yaw)+w*math.cos(yaw)) for u,w in FP];d.polygon(poly,fill=COLORS[k],outline=(15,15,15));d.line([tr(x,y),tr(x+.23*math.cos(yaw),y+.23*math.sin(yaw))],fill='white',width=2)
 gx,gy=tr(*r['goal_map'][:2]);d.ellipse((gx-4,gy-4,gx+4,gy+4),outline=(20,150,90),width=2)
 im=Image.new('RGB',(W,H),(249,249,249));im.paste(tile,(left,top));d=ImageDraw.Draw(im);d.rectangle((0,0,W-1,H-1),outline=(155,155,155));d.text((12,7),LABELS[k],font=FT,fill=COLORS[k]);state='RUNNING' if time<duration else {4:'SUCCEEDED',6:'ABORTED',5:'CANCELED'}.get(r['action_status'],'ENDED');d.text((12,34),f'{t:.1f}/{duration:.1f} sim s | {state} | 2x replay',font=F,fill=(35,35,35));d.text((12,58),f'GT distance to goal {math.dist([x,y],r["goal_map"][:2]):.2f}m | purple: actual /plan',font=F,fill=(45,45,45));d.text((12,82),'Full route: recorded Gazebo GT + physical footprint' if z==0 else 'Zoom: initial failure corner (robot later exits this view)',font=F,fill=(55,55,55));return im
maxdur=max(d[0]['navigation_sim_s'] for d in D.values());count=math.ceil(maxdur/.4)+1
colors=[(i,i,i) for i in range(240)]+[(249,249,249),(255,255,255),*COLORS,(164,85,185),(20,150,90)];colors += [(0,0,0)]*(256-len(colors));pal=Image.new('P',(1,1));pal.putpalette([x for c in colors for x in c]);prev=None;path=O/'theta-mppi-doorway-before-after-2x.gif'
with path.open('wb') as out:
 for n in range(count):
  canvas=Image.new('RGB',(W*3,H*2))
  for z in [0,1]:
   for k,v in enumerate(VS):canvas.paste(cell(v,z,n*.4),(W*k,H*z))
  p=canvas.quantize(palette=pal,dither=Image.Dither.NONE)
  if prev is None:
   for chunk in GifImagePlugin.getheader(p,info={'loop':0,'duration':200})[0]:out.write(chunk)
   box=(0,0,*p.size)
  else:box=ImageChops.difference(p,prev).getbbox() or (0,0,1,1)
  for chunk in GifImagePlugin.getdata(p.crop(box),offset=box[:2],duration=200,disposal=1):out.write(chunk)
  prev=p
 out.write(b';')
canvas=Image.new('RGB',(W*3,H*2))
for z in [0,1]:
 for k,v in enumerate(VS):canvas.paste(cell(v,z,maxdur),(W*k,H*z))
canvas.save(O/'theta-mppi-doorway-before-after-final.png')
(R/'media-manifest.json').write_text(json.dumps({'type':'Actual recorded Gazebo GT and /plan replay, not GUI capture','frames':count,'frame_ms':200,'sim_step_s':.4,'speedup':2,'dimensions':[W*3,H*2],'finished_failed_runs_hold_final_pose':True,'columns':VS,'rows':['full route','first failure corner zoom']},indent=2));print(path)
