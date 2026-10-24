#!/usr/bin/env python3
"""Portable canonical goal worlds and collision slices; no controller tuning."""
from pathlib import Path
import copy,json,math,hashlib,xml.etree.ElementTree as ET
import numpy as np,yaml
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parent;FIX=ROOT/'fixture';RES=.025;BOUNDS=(-24.,-15.,12.,6.);HEIGHT=.30
FOOT=[[-.318525,-.40737300000024484],[-.318525,.40742700000024484],[.318475,.40742700000024484],[.318475,-.40737300000024484]];PADDING=.02

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def pose(e):return list(map(float,e.findtext('pose','0 0 0 0 0 0').split()))
def mat(v):
 x,y,z,r,p,a=v;cr,sr,cp,sp,ca,sa=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(a),math.sin(a)
 m=np.eye(4);m[:3,:3]=np.array([[ca*cp,ca*sp*sr-sa*cr,ca*sp*cr+sa*sr],[sa*cp,sa*sp*sr+ca*cr,sa*sp*cr-ca*sr],[-sp,cp*sr,cp*cr]]);m[:3,3]=[x,y,z];return m

def materialize(source,dest,fixture=FIX):
 text=Path(source).read_text().replace('fixture://',str(Path(fixture).resolve())+'/');assert 'repo://' not in text and 'fixture://' not in text
 tree=ET.fromstring(text)
 for u in tree.findall('.//uri'):
  if u.text and u.text.startswith('/') and not Path(u.text).exists():raise FileNotFoundError(u.text)
 Path(dest).write_text(text);return sha(Path(dest))

def raster(world,out):
 xmin,ymin,xmax,ymax=BOUNDS;img=Image.new('L',(round((xmax-xmin)/RES),round((ymax-ymin)/RES)),254);draw=ImageDraw.Draw(img);counts={'box':0,'cylinder':0,'sphere':0,'mesh_slices':0,'mesh_slice_segments':0};cache={}
 def pix(x,y):return ((x-xmin)/RES,(ymax-y)/RES)
 for m in ET.parse(world).findall('./world/model'):
  if m.get('name') in ['cleany_mecanum','ground_plane','facility_ceiling']:continue
  for link in m.findall('link'):
   for c in link.findall('collision'):
    t=mat(pose(m))@mat(pose(link))@mat(pose(c));g=c.find('geometry');box=g.find('box');cyl=g.find('cylinder');sphere=g.find('sphere');mesh=g.find('mesh')
    if box is not None or cyl is not None:
     if box is not None:
      sx,sy,sz=map(float,box.findtext('size').split());vertices=np.array([[x,y,z,1] for x in [-sx/2,sx/2] for y in [-sy/2,sy/2] for z in [-sz/2,sz/2]]);edges=[(i,j) for i in range(8) for j in range(i+1,8) if bin(i^j).count('1')==1];kind='box'
     else:
      radius=float(cyl.findtext('radius'));height=float(cyl.findtext('length'));n=64;vertices=np.array([[radius*math.cos(2*math.pi*i/n),radius*math.sin(2*math.pi*i/n),z,1] for z in [-height/2,height/2] for i in range(n)]);edges=[(i,(i+1)%n) for i in range(n)]+[(i+n,(i+1)%n+n) for i in range(n)]+[(i,i+n) for i in range(n)];kind='cylinder'
     vertices=vertices@t.T;hit=[]
     for i,j in edges:
      a,b=vertices[i],vertices[j]
      if (a[2]-HEIGHT)*(b[2]-HEIGHT)<=0 and abs(a[2]-b[2])>1e-12:hit.append(a+(b-a)*(HEIGHT-a[2])/(b[2]-a[2]))
     if len(hit)<3:continue
     center=np.mean(hit,axis=0);hit.sort(key=lambda q:math.atan2(q[1]-center[1],q[0]-center[0]));draw.polygon([pix(*q[:2]) for q in hit],fill=0);counts[kind]+=1
    elif sphere is not None:
     radius=float(sphere.findtext('radius'));z=t[2,3]
     if abs(HEIGHT-z)>radius:continue
     radius=math.sqrt(max(0,radius**2-(HEIGHT-z)**2));draw.ellipse([pix(t[0,3]-radius,t[1,3]+radius),pix(t[0,3]+radius,t[1,3]-radius)],fill=0);counts['sphere']+=1
    elif mesh is not None:
     uri=mesh.findtext('uri');path=FIX/uri.removeprefix('fixture://') if uri.startswith('fixture://') else Path(uri);scale=np.array(list(map(float,mesh.findtext('scale','1 1 1').split())))
     assert path.suffix.lower()=='.obj',f'Unsupported static collision mesh {path}'
     if str(path) not in cache:
      verts=[];faces=[]
      for line in path.read_text().splitlines():
       if line.startswith('v '):verts.append(list(map(float,line.split()[1:4])))
       if line.startswith('f '):
        f=[int(x.split('/')[0])-1 for x in line.split()[1:]];faces.extend([[f[0],f[i],f[i+1]] for i in range(1,len(f)-1)])
      cache[str(path)]=(np.array(verts),np.array(faces))
     verts,faces=cache[str(path)];v=(np.column_stack([verts*scale,np.ones(len(verts))])@t.T)[:,:3];tri=v[faces];idx=np.where((tri[:,:,2].min(axis=1)<=HEIGHT)&(tri[:,:,2].max(axis=1)>=HEIGHT))[0];n=0
     for tr in tri[idx]:
      hit=[]
      for i,j in [(0,1),(1,2),(2,0)]:
       a,b=tr[i],tr[j]
       if (a[2]-HEIGHT)*(b[2]-HEIGHT)<=0 and abs(a[2]-b[2])>1e-12:hit.append(a+(b-a)*(HEIGHT-a[2])/(b[2]-a[2]))
      if len(hit)>=2:draw.line([pix(*hit[0][:2]),pix(*hit[1][:2])],fill=0,width=1);n+=1
     counts['mesh_slices']+=1;counts['mesh_slice_segments']+=n
 img.save(out/'map.pgm');meta={'image':'map.pgm','resolution':RES,'origin':[xmin,ymin,0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196};(out/'map.yaml').write_text(yaml.safe_dump(meta));(out/'map-provenance.json').write_text(json.dumps({'method':'static collision boxes/cylinders/spheres and OBJ triangle cross-sections at actual LiDAR plane; not SLAM','height_m':HEIGHT,'resolution_m':RES,'bounds':BOUNDS,'counts':counts,'canonical_world_sha256':sha(world),'map_sha256':sha(out/'map.pgm'),'mesh_interior_note':'OBJ slice surfaces rasterized; no invented convex-hull occupancy'},indent=2));return img

def occupied(img,p):
 xmin,ymin,xmax,ymax=BOUNDS;x,y,a=p;c,s=math.cos(a),math.sin(a);pts=[(x+c*xx-s*yy,y+s*xx+c*yy) for xx,yy in [[q[0]+math.copysign(PADDING,q[0]),q[1]+math.copysign(PADDING,q[1])] for q in FOOT]];mask=Image.new('L',img.size,0);ImageDraw.Draw(mask).polygon([((xx-xmin)/RES,(ymax-yy)/RES) for xx,yy in pts],fill=1);return int(np.sum((np.array(img)==0)&(np.array(mask)>0)))

def main():
 proposed=json.loads((ROOT/'plan.proposed.json').read_text());tree=ET.parse(FIX/'world.template.sdf');world=tree.getroot().find('world');old_names={m.get('name') for m in world.findall('model')};restore=[];src=ET.parse(FIX/'world.study.sdf')
 for name in ['office_chair_13','office_chair_43']:
  if name not in old_names:world.append(copy.deepcopy(src.find(f"./world/model[@name='{name}']")));restore.append(name)
 for uri in tree.findall('.//uri'):
  assert uri.text.startswith('fixture://') and (FIX/uri.text[len('fixture://'):]).exists(),uri.text
 full=ROOT/'world.full.template.sdf';tree.write(full,encoding='unicode',xml_declaration=True);results=[]
 for goal in proposed['goals']:
  out=ROOT/'goals'/f"{goal['index']}_{goal['id']}";out.mkdir(parents=True,exist_ok=False);w=copy.deepcopy(tree);root=w.getroot().find('world');removed=goal['remove_chair']
  if removed is not None:root.remove(root.find(f"model[@name='{removed}']"))
  canonical=out/'world.canonical.sdf';w.write(canonical,encoding='unicode',xml_declaration=True);img=raster(canonical,out)
  record={**goal,'world':'goals/'+out.name+'/world.canonical.sdf','map':'goals/'+out.name+'/map.yaml','world_sha256':sha(canonical),'map_sha256':sha(out/'map.pgm'),'model_count':len(root.findall('model')),'chair_count':len([m for m in root.findall('model') if 'chair' in m.get('name')]),'start_occupied_cells':occupied(img,proposed['home_world_map']),'proposed_goal_occupied_cells':occupied(img,goal['pose_map_world']),'coordinate_change':None};results.append(record)
 (ROOT/'preflight.static.json').write_text(json.dumps({'restored_chairs':restore,'restore_source':'fixture/world.study.sdf','restored_source_sha256':sha(FIX/'world.study.sdf'),'original_template_sha256':sha(FIX/'world.template.sdf'),'canonical_full_sha256':sha(full),'footprint':FOOT,'padding_m':PADDING,'lidar_height_m':HEIGHT,'goals':results},indent=2));print(json.dumps(results,indent=2))
if __name__=='__main__':main()
