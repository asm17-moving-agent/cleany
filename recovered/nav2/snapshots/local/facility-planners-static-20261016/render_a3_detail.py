from pathlib import Path
import json,math
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;L=R.parent/'facility-all-seats-lattice-static-20261016';rows=json.loads((R/'comparison.json').read_text());FP=json.loads((L/'goals.json').read_text())['footprint'];font='/usr/share/fonts/google-noto/NotoSans-Regular.ttf';F=ImageFont.truetype(font,17);FT=ImageFont.truetype(font,24);W,H=500,540;canvas=Image.new('RGB',(W*5,H+90),'white');dr=ImageDraw.Draw(canvas);dr.text((15,10),'A3-01 doorway / room detail | actual planner output poses',font=FT,fill=(20,20,20));dr.text((15,45),'Blue: emitted path. Red: native polygon interference at emitted yaw. 2D yaw is not a body-orientation plan. No driving.',font=F,fill=(70,70,70))
xmin,xmax,ymin,ymax=-24.,-18.,-.6,3.8;pw,ph=480,352;fp=[]
base=Image.open(L/'maps/all-chairs-present/map.pgm').convert('RGB');crop=base.crop((round((xmin+24)/.05),round((6-ymax)/.05),round((xmax+24)/.05),round((6-ymin)/.05))).resize((pw,ph),Image.Resampling.NEAREST)
for col,n in enumerate(['lattice','navfn','smac2d','hybrid','theta']):
 row=next(x for x in rows if x['planner']==n and x['seat']=='seat-a3-01');panel=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(panel);d.text((10,8),n,font=FT,fill=(20,20,20));d.text((10,42),'PATH' if row['path_generated'] else 'NO PATH',font=F,fill=(30,110,190) if row['path_generated'] else (210,30,30));view=crop.copy();v=ImageDraw.Draw(view)
 def tr(x,y):return (x-xmin)/(xmax-xmin)*pw,(ymax-y)/(ymax-ymin)*ph
 if row['path_generated']:
  folder=L if n=='lattice' else R/n;p=json.loads((folder/'paths/seat-a3-01/path.json').read_text());bad=set(row['literal_pose_collision_indices']);last=None
  for i,z in enumerate(p):
   if not (xmin-.5<=z[0]<=xmax+.5 and ymin-.5<=z[1]<=ymax+.5):continue
   if last and math.dist(z[:2],last[:2])<.08 and abs(z[2]-last[2])<.12:continue
   c=(210,30,30) if i in bad else (140,180,220);x,y,t=z;v.polygon([tr(x+a*math.cos(t)-b*math.sin(t),y+a*math.sin(t)+b*math.cos(t)) for a,b in FP],outline=c);last=z
  v.line([tr(*z[:2]) for z in p],fill=(30,100,210),width=2)
  for i in bad:
   x,y=tr(*p[i][:2]);v.ellipse((x-2,y-2,x+2,y+2),fill=(210,30,30))
  d.text((10,450),f"Interference samples: {len(bad)}/{len(p)}",font=F,fill=(120,45,45))
 x,y=tr(*row['goal'][:2]);v.ellipse((x-5,y-5,x+5,y+5),outline=(0,150,90),width=3);panel.paste(view,(10,85));d.rectangle((0,0,W-1,H-1),outline=(180,180,180));canvas.paste(panel,(col*W,90))
canvas.save(R/'media/a3-01-planner-footprint-detail.png')
