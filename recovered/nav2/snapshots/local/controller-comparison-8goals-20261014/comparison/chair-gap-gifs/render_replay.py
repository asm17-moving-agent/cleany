import csv,json,math,bisect
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
BASE=ROOT.parents[1]
MP=BASE/'mppi/20261014T002705_8e811a/final_validation/2_chair_gap'
TE=ROOT/'teb-source'
FONT='/usr/share/fonts/google-noto/NotoSans-Regular.ttf'
if not Path(FONT).exists(): FONT='/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf'
def ft(n): return ImageFont.truetype(FONT,n)
W,H=720,810; bounds=(-3.8,-5.6,1.8,-.4); x0,y0,x1,y1=bounds
scale=120
X,Y=24,135
def xy(x,y): return (X+(x-x0)*scale,Y+(y1-y)*scale)
mapimg=Image.open(BASE/'common/goals/2_chair_gap/map.pgm').convert('RGB')
# PGM top row is northern edge of the world map; origin [-24,-15], resolution .025.
h=mapimg.height
crop=mapimg.crop((round((x0+24)/.025),round(h-(y1+15)/.025),round((x1+24)/.025),round(h-(y0+15)/.025))).resize((round((x1-x0)*scale),round((y1-y0)*scale)),Image.Resampling.NEAREST)
# Occupied, free and unknown static-map pixels are recolored for readability.
pix=crop.load()
for yy in range(crop.height):
 for xx in range(crop.width):
  v=pix[xx,yy][0];pix[xx,yy]=(55,66,81) if v<100 else ((236,239,243) if v<240 else (249,250,252))
goal=(.575,-1.625)
base=Image.new('RGB',(W,H),(249,250,252));base.paste(crop,(X,Y));draw=ImageDraw.Draw(base)
for val in range(-3,2):
 xx=xy(val,y0)[0];draw.line((xx,Y,xx,Y+crop.height),fill=(216,222,230));draw.text((xx-8,Y+crop.height+7),str(val),font=ft(14),fill=(83,96,113))
for val in range(-5,0):
 yy=xy(x0,val)[1];draw.line((X,yy,X+crop.width,yy),fill=(216,222,230))
draw.text((24,777),'Recorded GT replay | 2x simulation time | seat 29 chair removed',font=ft(15),fill=(72,86,104))
start=xy(-1.865,-4.705);g=xy(*goal);rad=.3*scale
draw.ellipse((g[0]-rad,g[1]-rad,g[0]+rad,g[1]+rad),outline=(38,159,103),width=2)
draw.line((g[0]-8,g[1],g[0]+8,g[1]),fill=(38,159,103),width=3);draw.line((g[0],g[1]-8,g[0],g[1]+8),fill=(38,159,103),width=3)
draw.text((g[0]+12,g[1]-25),'Goal / 30 cm',font=ft(15),fill=(22,117,71));draw.text((start[0]-38,start[1]+15),'Start',font=ft(15),fill=(72,86,104))
def load(p):
 result=json.loads((p/'result.json').read_text());a=result['navigation_sim_start_s'];b=result['navigation_sim_end_s']
 rows=[]
 for row in csv.DictReader((p/'samples.csv').open()):
  t=float(row['sim_s']);pose=[float(row[k]) for k in ('gt_x','gt_y','gt_yaw')]
  if a-.2<=t<=b+.2 and all(math.isfinite(x) for x in pose):rows.append((t-a,*pose))
 rows.sort();return result,rows,b-a
sources={n:load(p) for n,p in [('MPPI',MP),('TEB',TE)]}
def render(n,t):
 result,rows,duration=sources[n];t=min(t,duration);times=[r[0] for r in rows];i=min(max(1,bisect.bisect_left(times,t)),len(rows)-1);a,b=rows[i-1],rows[i];f=max(0,min(1,(t-a[0])/(b[0]-a[0]) if b[0]!=a[0] else 0));x=a[1]+f*(b[1]-a[1]);y=a[2]+f*(b[2]-a[2]);yaw=a[3]+f*math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]));im=base.copy();d=ImageDraw.Draw(im);color=(30,112,225) if n=='MPPI' else (225,116,29)
 d.text((24,18),n+' / CHAIR GAP',font=ft(28),fill=color)
 d.text((24,60),f'Sim {t:05.1f} / {duration:.1f}s   |   playback 2x',font=ft(20),fill=(43,55,73))
 d.text((24,94),f'GT distance to goal: {math.dist((x,y),goal):.2f}m'+('   ARRIVED' if t>=duration else ''),font=ft(17),fill=(43,55,73))
 points=[xy(r[1],r[2]) for r in rows[:i]]+[xy(x,y)]
 if len(points)>1:d.line(points,fill=color,width=3)
 # Physical CAD planar body rectangle, width .477m, length .6548m in body axes.
 poly=[]
 for u,v in [(-.2385,-.3274),(.2385,-.3274),(.2385,.3274),(-.2385,.3274)]:poly.append(xy(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw)))
 d.polygon(poly,fill=color,outline=(17,35,59));center=xy(x,y);end=xy(x+.25*math.cos(yaw),y+.25*math.sin(yaw));d.line([center,end],fill='white',width=4);d.ellipse((center[0]-3,center[1]-3,center[0]+3,center[1]+3),fill='white')
 return im
for n in sources:
 dur=sources[n][2];count=math.ceil(dur/.1)+1;frames=[render(n,min(i*.1,dur)) for i in range(count)];frames[0].save(ROOT/(n.lower()+'-chair-gap-2x.gif'),save_all=True,append_images=frames[1:],duration=50,loop=0,optimize=False)
dur=max(s[2] for s in sources.values());frames=[]
for i in range(math.ceil(dur/.1)+1):
 im=Image.new('RGB',(W*2,H));im.paste(render('MPPI',i*.1),(0,0));im.paste(render('TEB',i*.1),(W,0));frames.append(im)
frames[0].save(ROOT/'mppi-teb-chair-gap-2x.gif',save_all=True,append_images=frames[1:],duration=50,loop=0,optimize=False)
render('MPPI',12).save(ROOT/'preview.png')
(ROOT/'provenance.json').write_text(json.dumps({'kind':'original measured GT trajectory replay on canonical static map, not Gazebo GUI footage','speedup':2,'time_basis':'simulation seconds','output_frame_duration_ms':50,'sim_step_s':.1,'sources':{n:{'samples':str(MP if n=='MPPI' else TE),'sim_duration_s':v[2],'nav2_status':v[0]['action_status']} for n,v in sources.items()},'robot_shape':'physical CAD planar body rectangle; arms excluded as in simulation; not padded navigation footprint','camera':'fixed same world map crop'},indent=2))
print({n:s[2] for n,s in sources.items()})
for p in ROOT.glob('*.gif'):
 im=Image.open(p);print(p.name,p.stat().st_size,im.n_frames,sum((im.seek(i) or im.info.get('duration',0)) for i in range(im.n_frames))/1000)
