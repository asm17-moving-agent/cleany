from pathlib import Path
import json,math,numpy as np,yaml
from PIL import Image,ImageDraw
from xml.etree import ElementTree as ET
out=Path(__file__).resolve().parent
world=out/"world.sdf"
# Static reference map rasterized from collision primitives at the LiDAR plane.
xmin,ymin,xmax,ymax=-24.,-15.,12.,6.;res=.05
img=Image.new('L',(round((xmax-xmin)/res),round((ymax-ymin)/res)),254);draw=ImageDraw.Draw(img)
def pose(element):
 v=list(map(float,(element.findtext('pose') or '0 0 0 0 0 0').split()));return v

def matrix(v):
 c,s=math.cos(v[5]),math.sin(v[5]);return np.array([[c,-s,v[0]],[s,c,v[1]],[0,0,1]])
def pixel(x,y):return ((x-xmin)/res,(ymax-y)/res)
count=0
for model in ET.parse(world).findall('./world/model'):
 if model.get('name') in ('cleany_mecanum','ground_plane','facility_ceiling'):continue
 mp=pose(model)
 for link in model.findall('link'):
  lp=pose(link)
  for collision in link.findall('collision'):
   cp=pose(collision);z=mp[2]+lp[2]+cp[2];transform=matrix(mp)@matrix(lp)@matrix(cp);g=collision.find('geometry')
   box=g.find('box')
   if box is not None:
    sx,sy,sz=map(float,box.findtext('size').split())
    if not z-sz/2<=.26<=z+sz/2:continue
    points=[transform@np.array([x,y,1]) for x,y in [(-sx/2,-sy/2),(sx/2,-sy/2),(sx/2,sy/2),(-sx/2,sy/2)]]
    draw.polygon([pixel(p[0],p[1]) for p in points],fill=0);count+=1
   else:
    cyl=g.find('cylinder');sphere=g.find('sphere')
    shape=cyl if cyl is not None else sphere
    if shape is None:continue
    radius=float(shape.findtext('radius'));height=float(shape.findtext('length')) if cyl is not None else 2*radius
    if not z-height/2<=.26<=z+height/2:continue
    center=transform@np.array([0,0,1]);a=pixel(center[0]-radius,center[1]+radius);b=pixel(center[0]+radius,center[1]-radius)
    draw.ellipse([a,b],fill=0);count+=1
img.save(out/'facility_integration.pgm')
(out/'facility_integration.yaml').write_text(yaml.safe_dump(dict(image='facility_integration.pgm',resolution=res,origin=[xmin,ymin,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
(out/'map-provenance.json').write_text(json.dumps(dict(method='static collision primitive LiDAR-plane rasterization, not SLAM',height_m=.26,primitive_count=count,resolution_m=res),indent=2))
