from pathlib import Path
import json,math,csv
from collections import Counter,defaultdict
import numpy as np
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;rows=json.loads((R/'results.json').read_text());locked=json.loads((R/'goals.json').read_text());goals=locked['goals'];FP=locked['footprint'];home=locked['home'];OUT=R/'media';OUT.mkdir(exist_ok=True)
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf';F=ImageFont.truetype(font,14);FT=ImageFont.truetype(font,19);FS=ImageFont.truetype(font,11)
GREEN=(0,131,91);RED=(208,42,42);BLUE=(34,102,211);BG=(250,250,250)
PATHS={row['seat']:json.loads((R/'paths'/row['seat']/'path.json').read_text()) if (R/'paths'/row['seat']/'path.json').exists() else [] for row in rows};GROUPS=['D-hub','A1','A2','A3','A4','M1','M2','M3']

def scene(mapfile,points,size,top=80,margin=.6,bounds=None):
 w,h=size;src=Image.open(Path(mapfile).with_name('map.pgm')).convert('RGB');xmin,ymin,xmax,ymax=bounds if bounds is not None else (min(p[0] for p in points)-margin,min(p[1] for p in points)-margin,max(p[0] for p in points)+margin,max(p[1] for p in points)+margin)
 scale=min((w-22)/(xmax-xmin),(h-top-24)/(ymax-ymin));pw,ph=round((xmax-xmin)*scale),round((ymax-ymin)*scale);left=(w-pw)//2;vtop=top+(h-top-20-ph)//2
 def tr(x,y):return left+(x-xmin)*scale,vtop+(ymax-y)*scale
 crop=src.crop((round((xmin+24)/.05),round((6-ymax)/.05),round((xmax+24)/.05),round((6-ymin)/.05))).resize((pw,ph),Image.NEAREST);im=Image.new('RGB',size,BG);im.paste(crop,(left,vtop));ImageDraw.Draw(im).rectangle((0,0,w-1,h-1),outline=(170,170,170));return im,tr

def mark(d,tr,point,color,goal=False,label=None):
 x,y,yaw=point;px,py=tr(x,y)
 if goal:d.ellipse((px-4,py-4,px+4,py+4),outline=color,width=2)
 else:d.rectangle((px-3,py-3,px+3,py+3),fill=color)
 ax,ay=tr(x+.18*math.cos(yaw),y+.18*math.sin(yaw));d.line([(px,py),(ax,ay)],fill=color,width=2)
 if label:d.text((px+5,py-8),label,font=FS,fill=color)

def tile(row,footprint=False,size=(450,370)):
 p=PATHS[row['seat']];points=[home,row['goal']]+p;im,tr=scene(row['map'],points,size);d=ImageDraw.Draw(im);c=BLUE if row['path_generated'] else RED
 if footprint and p:
  ov=Image.new('RGBA',im.size);od=ImageDraw.Draw(ov);prev=None
  for pose in p:
   if prev and math.dist(pose[:2],prev[:2])<.3 and abs(math.atan2(math.sin(pose[2]-prev[2]),math.cos(pose[2]-prev[2])))<.4:continue
   x,y,t=pose;body=[tr(x+a*math.cos(t)-b*math.sin(t),y+a*math.sin(t)+b*math.cos(t)) for a,b in FP];od.polygon(body,outline=(*c,65));prev=pose
  im=Image.alpha_composite(im.convert('RGBA'),ov).convert('RGB');d=ImageDraw.Draw(im)
 if len(p)>1:d.line([tr(*z[:2]) for z in p],fill=c,width=2)
 mark(d,tr,home,(30,30,30));mark(d,tr,row['goal'],GREEN if row['path_generated'] else RED,True)
 d.text((10,7),row['seat']+' | '+('PATH' if row['path_generated'] else 'NO PATH'),font=FT,fill=c)
 if row['path_generated']:d.text((10,33),f"{row['path_length_m']:.2f} m | {row['planning_s']:.3f} s | end {row['goal_xy_error_m']*100:.1f} cm",font=F,fill=(40,40,40))
 else:d.text((10,33),f"status {row['action_status']} | {row['wall_s']:.2f} s",font=F,fill=RED)
 d.text((10,55),'Target chair removed' if row['removed_chair'] else 'All chairs retained',font=F,fill=(75,75,75));return im

for group,selected,cols in [('D-hub',[x for x in rows if x['zone']=='D-hub'],6),('rooms',[x for x in rows if x['zone']!='D-hub'],6)]:
 for footprints,suffix in [(False,'centerlines'),(True,'footprints')]:
  h=math.ceil(len(selected)/cols);canvas=Image.new('RGB',(450*cols,370*h+75),'white');dr=ImageDraw.Draw(canvas);dr.text((15,10),f'{group} | Theta* (traversal weight 5) | {sum(x["path_generated"] for x in selected)}/{len(selected)} paths',font=FT,fill=(20,20,20));dr.text((15,40),'HOME -> existing fixed approach goal | Theta* (traversal weight 5) | no driving',font=F,fill=(70,70,70))
  for k,row in enumerate(selected):canvas.paste(tile(row,footprints),(k%cols*450,k//cols*370+75))
  canvas.save(OUT/f'{group}-all-seats-{suffix}.png')
# Overview group panels use the all-chairs map only as a common visual background.
w,h=720,510;overview=Image.new('RGB',(w*4,h*2+90),'white');d=ImageDraw.Draw(overview);d.text((18,10),'18F facility | HOME to every requested seat | Theta* (traversal weight 5)',font=ImageFont.truetype(font,27),fill=(20,20,20));d.text((18,50),'Static planning only. Overview background shows full furniture; D-hub paths use a separate map with only their own target chair removed.',font=ImageFont.truetype(font,17),fill=(65,65,65))
for k,group in enumerate(GROUPS):
 selected=[x for x in rows if x['zone']==group];im,tr=scene(R/'maps/all-chairs-present/map.yaml',[home],(w,h),top=60,bounds=(-24,-15,12,6));dr=ImageDraw.Draw(im)
 for index,row in enumerate(selected):
  p=PATHS[row['seat']];c=BLUE if row['path_generated'] else RED
  if len(p)>1:dr.line([tr(*z[:2]) for z in p],fill=c,width=2)
  label=row['seat'].removeprefix('seat-');mark(dr,tr,row['goal'],GREEN if row['path_generated'] else RED,True,label)
 mark(dr,tr,home,(0,0,0),label='HOME');dr.text((12,8),f'{group}: {sum(x["path_generated"] for x in selected)}/{len(selected)} paths',font=FT,fill=(20,20,20));dr.text((12,34),'Green goal: path generated | Red goal: no path',font=F,fill=(70,70,70));overview.paste(im,(k%4*w,k//4*h+90))
overview.save(OUT/'facility-82-seats-overview.png')
fail=[x for x in rows if not x['path_generated']]
for row in fail:tile(row,True,size=(450,370)).save(OUT/(row['seat']+'-failure.png'))
summary={'total':len(rows),'paths_generated':sum(x['path_generated'] for x in rows),'failed_seats':[x['seat'] for x in fail],'by_zone':{z:{'total':sum(x['zone']==z for x in rows),'generated':sum(x['zone']==z and x['path_generated'] for x in rows)} for z in GROUPS},'scope':'Static native Nav2 planner actions with static collision-slice map + native inflation + polygon checker; not actual driving or full3D swept collision proof','duplicate_goal_groups':[[x['seat'] for x in goals if tuple(x['goal'])==g] for g in {tuple(x['goal']) for x in goals} if sum(tuple(x['goal'])==g for x in goals)>1]}
(R/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
