"""Actual initial global plans and recorded GT swept footprint snapshots."""
from pathlib import Path
import json,math
from PIL import Image,ImageDraw
R=Path(__file__).resolve().parent
s=(R/'render_results.py').read_text().split('\nfor i in INDICES:\n    name=NAMES[i]\n    write_gif')[0]
ns={'__file__':str(R/'render_results.py'),'__name__':'static_renderer'}
exec(compile(s,'render_results.py','exec'),ns)
W,H=ns['W'],ns['H']; ps=ns['PLANNERS']; indices=ns['INDICES'];out=R/'media'
canvas=Image.new('RGB',(W*len(indices),H*len(ps)))
fp=[(-.238525,-.327373),(-.238525,.327427),(.238475,.327427),(.238475,-.327373)]
for row,p in enumerate(ps):
 for col,i in enumerate(indices):
  result,rows,_,duration=ns['DATA'][p,i];im=ns['cell'](p,i,duration).convert('RGBA');ov=Image.new('RGBA',im.size);d=ImageDraw.Draw(ov);tr=ns['BACKGROUNDS'][i][1];color=ns['COLORS'][p];last=None
  for n,(_,x,y,yaw) in enumerate(rows):
   if last and n!=len(rows)-1 and math.dist((x,y),last[:2])<.16 and abs(math.atan2(math.sin(yaw-last[2]),math.cos(yaw-last[2])))<.15:continue
   poly=[tr(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw)) for u,v in fp];d.polygon(poly,fill=(*color,13),outline=(*color,90));last=(x,y,yaw)
  im=Image.alpha_composite(im,ov).convert('RGB');d=ImageDraw.Draw(im)
  plans=json.loads((R/p/f'{i}_{ns["NAMES"][i]}'/'plans.json').read_text());poses=plans[0]['poses'] if plans else []
  distance=0
  for a,b in zip(poses,poses[1:]):
   aa=tr(*a[:2]);bb=tr(*b[:2]);length=math.dist(aa,bb);steps=max(1,math.ceil(length))
   for k in range(steps):
    if int(distance+k*length/steps)%10<6:
     f=k/steps;g=min(1,(k+1)/steps);d.line((aa[0]+f*(bb[0]-aa[0]),aa[1]+f*(bb[1]-aa[1]),aa[0]+g*(bb[0]-aa[0]),aa[1]+g*(bb[1]-aa[1])),fill=(140,45,165),width=2)
   distance+=length
  d.rectangle((2,25,W-3,70),fill=(249,249,249));d.text((10,28),'Purple dashed: actual first global plan',font=ns['F14'],fill=(130,40,150));d.text((10,49),'Colored outlines: actual GT footprint, padding 0',font=ns['F14'],fill=(50,50,50))
  d.rectangle((2,69,W-3,94),fill=(249,249,249))
  state={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(result.get('action_status'),'ENDED')
  if result.get('timeout'):state='TIME LIMIT'
  d.text((10,70),f'{state}: GT {100*result["position_error_m"]:.1f}cm / {result["yaw_error_deg"]:.1f}deg',font=ns['F14'],fill=(45,45,45))
  canvas.paste(im,(W*col,H*row))
canvas.save(out/'initial-plan-and-gt-footprints-4rows-8cols.png')
(out/'static-provenance.json').write_text(json.dumps({'data':'Actual recorded samples.csv and plans.json','footprint_m':fp,'sampling':'0.16m translation or0.15radrotation plusfinal','continuous_collision_check':False,'orientation':'Actual GT yaw, not tangent assumptions','initial_global_plan_only':True},indent=2))
print(out/'initial-plan-and-gt-footprints-4rows-8cols.png')
