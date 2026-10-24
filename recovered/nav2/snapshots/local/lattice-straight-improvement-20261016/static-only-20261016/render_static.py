"""Plot only native planned XY/yaw on the frozen costmap; no actual driving traces."""
from pathlib import Path
import json,math,csv
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'media';OUT.mkdir(exist_ok=True)
W,H=660,470
F=ImageFont.truetype('/usr/share/fonts/google-noto/NotoSans-Regular.ttf',16)
FT=ImageFont.truetype('/usr/share/fonts/google-noto/NotoSans-Regular.ttf',20)
ROUTES=[(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')]
VARIANTS=['original_exact','current','no_curves']
LABELS={'original_exact':'Original Lattice, exact goal','current':'Current refined Lattice','no_curves':'Diagnostic: curved motions removed'}
COLORS={'original_exact':(110,110,110),'current':(213,111,27),'no_curves':(35,101,210)}
DATA={};BGS={}
for mode in ['home','recorded']:
 folder=ROOT if mode=='home' else ROOT/'recorded-start'
 summary=json.loads((folder/'summary.json').read_text())
 for i,name in ROUTES:
  for v in VARIANTS:
   stem=folder/f'{i}-{name}-{v}'
   inp=json.loads(stem.with_suffix('.input.json').read_text());poses=json.loads(stem.with_suffix('.output.json').read_text())['poses'];rec=next(x for x in summary if x['route_index']==i+1 and x['variant']==v)
   DATA[mode,v,i]=(inp,poses,rec)
  points=[p for v in VARIANTS for p in DATA[mode,v,i][1]]
  x0,x1=min(p[0] for p in points)-.5,max(p[0] for p in points)+.5;y0,y1=min(p[1] for p in points)-.5,max(p[1] for p in points)+.5
  scale=min((W-36)/(x1-x0),(H-155)/(y1-y0));pw,ph=round((x1-x0)*scale),round((y1-y0)*scale);left,top=(W-pw)//2,140+(H-152-ph)//2
  def tr(x,y,x0=x0,y1=y1,scale=scale,left=left,top=top):return left+(x-x0)*scale,top+(y1-y)*scale
  inp=DATA[mode,'current',i][0];binary=folder/f'{i}-{name}.bin';nx,ny=inp['nx'],inp['ny'];native=Image.frombytes('L',(nx,ny),binary.read_bytes()).transpose(Image.Transpose.FLIP_TOP_BOTTOM)
  channels=[]
  for ch in range(3):
   lut=[]
   for c in range(256):
    color=(249,249,249) if c==0 else ((200,200,200) if c==255 else ((45,45,45) if c==254 else ((130,130,130) if c==253 else (255,round(249-75*c/252),round(249-75*c/252)))))
    lut.append(color[ch])
   channels.append(native.point(lut))
  cm=Image.merge('RGB',channels);ox,oy,_=inp['origin'];res=inp['resolution'];crop=cm.crop((round((x0-ox)/res),round(ny-(y1-oy)/res),round((x1-ox)/res),round(ny-(y0-oy)/res))).resize((pw,ph),Image.Resampling.NEAREST);bg=Image.new('RGB',(W,H),(249,249,249));bg.paste(crop,(left,top));d=ImageDraw.Draw(bg);d.rectangle((0,0,W-1,H-1),outline=(150,150,150));BGS[mode,i]=bg,tr

def body(p,tr,fp):
 x,y,yaw=p;return [tr(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw)) for u,v in fp]
def cell(mode,v,i,footprints):
 bg,tr=BGS[mode,i];im=bg.copy();inp,p,rec=DATA[mode,v,i];color=COLORS[v]
 if footprints:
  overlay=Image.new('RGBA',im.size);d=ImageDraw.Draw(overlay);last=None
  for pose in p:
   if last and math.dist(pose[:2],last[:2])<.15 and abs(math.atan2(math.sin(pose[2]-last[2]),math.cos(pose[2]-last[2])))<.22:continue
   d.polygon(body(pose,tr,inp['footprint']),fill=(*color,9),outline=(*color,95));last=pose
  im=Image.alpha_composite(im.convert('RGBA'),overlay).convert('RGB')
 d=ImageDraw.Draw(im)
 # Direct segment is a comparison reference, never substituted for planner output.
 a,b=tr(*p[0][:2]),tr(*p[-1][:2]);length=math.dist(a,b)
 for n in range(0,math.ceil(length),9):
  d.line([tuple(a[k]+(b[k]-a[k])*s/max(length,1e-9) for k in [0,1]) for s in [n,min(n+4,length)]],fill=(185,185,185),width=2)
 d.line([tr(*x[:2]) for x in p],fill=color,width=3)
 for index in [0,-1]:
  x,y,yaw=p[index];px,py=tr(x,y);d.ellipse((px-4,py-4,px+4,py+4),fill=(30,30,30));d.line([tr(x,y),tr(x+.2*math.cos(yaw),y+.2*math.sin(yaw))],fill=(30,30,30),width=2)
 d.text((12,8),f'{LABELS[v]} | {i+1}. {dict(ROUTES)[i]}',font=FT,fill=color)
 d.text((12,37),'STATIC PLAN ONLY | same frozen costmap and start/goal',font=F,fill=(40,40,40))
 d.text((12,61),f'XY max deviation {rec["max_cross_track_cm"]:.1f}cm | length {rec["length_m"]:.3f}m',font=F,fill=(40,40,40))
 d.text((12,85),f'Planned yaw travel {rec["yaw_variation_deg"]:.1f}deg | end yaw error {rec["goal_yaw_error_deg"]:.3f}deg',font=F,fill=(40,40,40))
 d.text((12,109),'Gray: obstacle | pink: inflation | '+('outlines: planned footprint' if footprints else 'dashed gray: direct reference'),font=F,fill=(70,70,70))
 return im
for mode in ['recorded','home']:
 for footprints,suffix in [(False,'xy'),(True,'footprints')]:
  im=Image.new('RGB',(W*4,H*3))
  for row,v in enumerate(VARIANTS):
   for col,(i,name) in enumerate(ROUTES):im.paste(cell(mode,v,i,footprints),(W*col,H*row))
  im.save(OUT/f'{mode}-static-{suffix}-3rows-4cols.png')
  if mode=='recorded':
   two=im.crop((0,H,W*4,H*3));two.save(OUT/f'current-vs-no-curves-static-{suffix}-2rows-4cols.png')
   for i,name in ROUTES:
    pair=Image.new('RGB',(W,H*2))
    for row,v in enumerate(['current','no_curves']):pair.paste(cell(mode,v,i,footprints),(0,H*row))
    pair.save(OUT/f'{i+1:02d}-{name}-static-{suffix}.png')
# Every row in this artifact is a static planner result; no smoothing/render changes to path poses.
rows=[]
for mode,folder in [('home',ROOT),('recorded',ROOT/'recorded-start')]:
 for row in json.loads((folder/'summary.json').read_text()):rows.append({'start_mode':mode,**row})
(ROOT/'comparison.json').write_text(json.dumps({'rows':rows,'visualization':'Actual native static path poses, no GT, controller, replanning, smoothing or hand-drawn replacement','footprint':'Native planned yaw, padding 0; sampled outlines are not a continuous swept-collision proof'},indent=2))
keys=['start_mode','route_index','route','variant','ok','max_cross_track_cm','length_m','straight_distance_m','yaw_variation_deg','goal_yaw_error_deg','search_ms']
with (ROOT/'comparison.csv').open('w') as f:
 w=csv.DictWriter(f,keys,extrasaction='ignore');w.writeheader();w.writerows(rows)
print(OUT)
