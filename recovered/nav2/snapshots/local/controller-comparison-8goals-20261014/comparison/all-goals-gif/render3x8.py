import json,csv,math,bisect,time
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont,ImageChops,GifImagePlugin
O=Path(__file__).resolve().parent;ROOT=O.parents[1];MP=ROOT/'mppi/20261014T002705_8e811a/final_validation';TE=O/'teb-source';DW=ROOT/'dwb/20261014T0035_5bfca0/campaign'
NAMES=['straight','turn_arrival','chair_gap','far_heading','distant','lateral','narrow_seat','other_room']
LABELS=['Straight','Turn arrival','Chair gap / seat29','Far heading','Distant / seat01','Lateral','Narrow seat / seat43','Other room / M1']
locked=json.loads((ROOT/'common/goals.locked.json').read_text());print('locktype',type(locked).__name__,flush=True)
compare=json.loads((O.parent/'comparison.json').read_text())['rows']
FONT='/usr/share/fonts/google-noto/NotoSans-Regular.ttf'
if not Path(FONT).exists():FONT='/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf'
f14=ImageFont.truetype(FONT,14);f17=ImageFont.truetype(FONT,17)
W,H=440,300;DATA={};BGS={};G=[]
for i,n in enumerate(NAMES):
 goal=json.loads((MP/f'{i}_{n}'/'result.json').read_text())['goal_map'];G.append(goal)
 for c,root in [('DWB',DW),('MPPI',MP),('TEB',TE)]:
  p=root/f'{i}_{n}';r=json.loads((p/'result.json').read_text());start=r['navigation_sim_start_s'];end=r['navigation_sim_end_s'];rows=[]
  for a in csv.DictReader((p/'samples.csv').open()):
   t=float(a['sim_s']);pose=[float(a[k]) for k in ['gt_x','gt_y','gt_yaw']]
   if start-.2<=t<=end+.2 and all(math.isfinite(v) for v in pose):rows.append((t-start,*pose))
  rows.sort();DATA[c,i]=(r,rows,[a[0] for a in rows],end-start)
 pts=[a[1:3] for c in ['DWB','MPPI','TEB'] for a in DATA[c,i][1]]+[goal[:2]]
 x0=min(a[0] for a in pts)-.65;x1=max(a[0] for a in pts)+.65;y0=min(a[1] for a in pts)-.65;y1=max(a[1] for a in pts)+.65
 scale=min((W-28)/(x1-x0),(H-87)/(y1-y0));pw=round((x1-x0)*scale);ph=round((y1-y0)*scale);ox=(W-pw)//2;oy=75+(H-84-ph)//2
 def tr(x,y,x0=x0,y1=y1,s=scale,ox=ox,oy=oy):return (ox+(x-x0)*s,oy+(y1-y)*s)
 m=Image.open(ROOT/f'common/goals/{i}_{n}/map.pgm').convert('L');mh=m.height
 crop=m.crop((round((x0+24)/.025),round(mh-(y1+15)/.025),round((x1+24)/.025),round(mh-(y0+15)/.025))).point(lambda v:55 if v<100 else (224 if v<240 else 249)).convert('RGB').resize((pw,ph),Image.Resampling.NEAREST)
 im=Image.new('RGB',(W,H),(249,249,249));im.paste(crop,(ox,oy));d=ImageDraw.Draw(im);gp=tr(*goal[:2]);rr=.3*scale;d.ellipse((gp[0]-rr,gp[1]-rr,gp[0]+rr,gp[1]+rr),outline=(20,150,90),width=2);d.line((gp[0]-4,gp[1],gp[0]+4,gp[1]),fill=(20,150,90),width=2);d.line((gp[0],gp[1]-4,gp[0],gp[1]+4),fill=(20,150,90),width=2)
 d.rectangle((0,0,W-1,H-1),outline=(190,190,190));BGS[i]=(im,tr,scale)
def frame(c,i,t):
 r,rows,times,dur=DATA[c,i];t=min(t,dur);j=min(max(1,bisect.bisect_left(times,t)),len(rows)-1);a,b=rows[j-1],rows[j];f=max(0,min(1,(t-a[0])/(b[0]-a[0]) if b[0]!=a[0] else 0));x=a[1]+f*(b[1]-a[1]);y=a[2]+f*(b[2]-a[2]);yaw=a[3]+f*math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]));bg,tr,s=BGS[i];im=bg.copy();d=ImageDraw.Draw(im);col=(137,65,190) if c=='DWB' else ((30,112,225) if c=='MPPI' else (225,116,29))
 d.text((12,7),f'{c} | {i+1}. {LABELS[i]}',font=f17,fill=col)
 state='RUNNING' if t<dur else ('ARRIVED' if r['action_status']==4 else ('TIMEOUT' if r['action_status']==5 else 'ABORTED'))
 d.text((12,31),f'{t:.1f}/{dur:.1f} sim s | {state}',font=f14,fill=(40,40,40));d.text((12,51),f'GT remaining {math.dist((x,y),G[i][:2]):.2f}m | 2x sim',font=f14,fill=(70,70,70))
 points=[tr(z[1],z[2]) for z in rows[:j:4]]+[tr(x,y)]
 if len(points)>1:d.line(points,fill=col,width=2)
 poly=[tr(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw)) for u,v in [(-.2385,-.3274),(.2385,-.3274),(.2385,.3274),(-.2385,.3274)]];d.polygon(poly,fill=col,outline=(20,20,20));d.line([tr(x,y),tr(x+.23*math.cos(yaw),y+.23*math.sin(yaw))],fill=(255,255,255),width=2)
 return im
# Fixed global palette; incremental delta rectangles avoid storing all frames in memory.
colors=[(i,i,i) for i in range(240)]+[(249,249,249),(255,255,255),(30,112,225),(225,116,29),(20,150,90),(137,65,190)]
colors+= [(0,0,0)]*(256-len(colors));pal=Image.new('P',(1,1));pal.putpalette([v for c in colors for v in c])
duration=max(v[3] for v in DATA.values());count=math.ceil(duration/.4)+1;previous=None;path=O/'dwb-mppi-teb-3rows-8cols-2x.gif'
with path.open('wb') as out:
 for k in range(count):
  t=k*.4;canvas=Image.new('RGB',(W*8,H*3))
  for i in range(8):
   for j,c in enumerate(['DWB','MPPI','TEB']):canvas.paste(frame(c,i,t),(i*W,j*H))
  if k==0:canvas.save(O/'preview-3rows-8cols.png')
  indexed=canvas.quantize(palette=pal,dither=Image.Dither.NONE)
  if previous is None:
   for chunk in GifImagePlugin.getheader(indexed,info={'loop':0,'duration':200})[0]:out.write(chunk)
   box=(0,0,W*8,H*3)
  else:box=ImageChops.difference(indexed,previous).getbbox() or (0,0,1,1)
  part=indexed.crop(box)
  for chunk in GifImagePlugin.getdata(part,offset=(box[0],box[1]),duration=200,disposal=1):out.write(chunk)
  previous=indexed
  if k%100==0:print(f'{k}/{count}',flush=True)
 out.write(b';')
im=Image.open(path);total=0
for k in range(im.n_frames):im.seek(k);total+=im.info.get('duration',0)
(O/'provenance-3rows-8cols.json').write_text(json.dumps({'kind':'original GT replay, not screen footage','layout':'3rows: DWB/MPPI/TEB top to bottom; 8columns: goals1-8 left to right','speedup':2,'time_basis':'simulation','frame_ms':200,'sim_step_s':.4,'frames':im.n_frames,'gif_duration_s':total/1000,'max_sim_duration_s':duration,'finished_runs_hold_last_pose':True,'failed_runs_stationary_segments_preserved':True,'sources':{f'{c}_{i}':{'duration_sim_s':v[3],'status':v[0]['action_status']} for (c,i),v in DATA.items()}},indent=2))
print(path,im.size,im.n_frames,total/1000,path.stat().st_size,flush=True)
