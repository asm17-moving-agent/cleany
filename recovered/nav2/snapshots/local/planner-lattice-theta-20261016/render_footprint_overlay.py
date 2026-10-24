from pathlib import Path
import math, json
from PIL import Image, ImageDraw
ROOT=Path(__file__).resolve().parent
source=(ROOT/'render_planner_overlay.py').read_text().split('\nfor show_gt,name in ')[0]
ns={'__file__':str(ROOT/'render_planner_overlay.py'),'__name__':'footprint_overlay'}
exec(compile(source,'render_planner_overlay.py','exec'),ns)
W,H=ns['W'],ns['H']; OUT=ns['OUT']; renderer=ns['namespace']
footprint=[(-.238525,-.327373),(-.238525,.327427),(.238475,.327427),(.238475,-.327373)]
def cell(p,i):
    im=ns['static_cell'](p,i,True).convert('RGBA')
    overlay=Image.new('RGBA',im.size);d=ImageDraw.Draw(overlay)
    rows=ns['DATA'][p,i][1]; tr=ns['BACKGROUNDS'][i][1];color=renderer['COLORS'][p]
    last=None
    for n,r in enumerate(rows):
        _,x,y,yaw=r
        if last and n!=len(rows)-1 and math.dist((x,y),last[:2])<.16 and abs(math.atan2(math.sin(yaw-last[2]),math.cos(yaw-last[2])))<.15:continue
        poly=[tr(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw)) for u,v in footprint]
        d.polygon(poly,fill=(*color,13),outline=(*color,90),width=1)
        last=(x,y,yaw)
    im=Image.alpha_composite(im,overlay).convert('RGB')
    ns['path_on'](im,p,i,ns['PLANS'][p,i][0][0]);ns['add_gt'](im,p,i)
    d=ImageDraw.Draw(im);d.rectangle((1,25,W-2,68),fill=(249,249,249))
    d.text((10,28),'Purple dashed: actual initial global plan',font=ns['FONT'],fill=(90,40,110))
    d.text((10,49),'Colored outlines: recorded GT footprint (no padding)',font=ns['FONT'],fill=(50,50,50))
    return im
im=Image.new('RGB',(W*8,H*2))
for row,p in enumerate(ns['PLANNERS']):
    for i in range(8):im.paste(cell(p,i),(W*i,H*row))
im.save(OUT/'initial-plan-and-gt-footprints-2rows-8cols.png')
for i in range(8):
    pair=Image.new('RGB',(W,H*2))
    for row,p in enumerate(ns['PLANNERS']):pair.paste(cell(p,i),(0,H*row))
    pair.resize((W*2,H*4)).save(OUT/f'{i+1:02d}-{ns["NAMES"][i]}-footprints.png')
(OUT/'footprint-provenance.json').write_text(json.dumps({'footprint_m':footprint,'padding_m':0,'orientation':'Recorded actual Gazebo GT yaw, both planners. Not tangent-assumed Theta orientations.','centerline':'First recorded global plan','sampling':'Footprints after >=0.16m translation or >=0.15rad rotation plus last sample; not a continuous collision check','sources':['samples.csv','plans.json','parameters.yaml']},indent=2))
print(OUT/'initial-plan-and-gt-footprints-2rows-8cols.png')
