from pathlib import Path
import json,subprocess,math,csv
from PIL import Image,ImageDraw,ImageFont
R=Path(__file__).resolve().parent;L=R.parent/'facility-all-seats-lattice-static-20261016';OUT=R/'media';OUT.mkdir(exist_ok=True)
summary=json.loads((R/'summary.json').read_text());rows=json.loads((R/'comparison.json').read_text());names={'lattice':'Lattice (official 144, R=10 cm)','navfn':'NavFn (Dijkstra)','smac2d':'Smac 2D','hybrid':'Hybrid-A* (Reeds-Shepp, R=10 cm)','theta':'Theta* (traversal weight 5)'}
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf';F=ImageFont.truetype(font,19);FT=ImageFont.truetype(font,25);FS=ImageFont.truetype(font,15)
# Keep seat-by-seat atlases for every new planner, using the same rendering as the Lattice baseline.
template=(L/'render_results.py').read_text()
for planner in ['navfn','smac2d','hybrid','theta']:
 folder=R/planner
 if not (folder/'maps').exists():(folder/'maps').symlink_to(L/'maps',target_is_directory=True)
 code=template.replace('static native Lattice',names[planner]).replace('official 144 primitives, R=10cm',names[planner]).replace('18F facility | HOME to every requested seat | Lattice, official 144 primitives','18F facility | HOME to every requested seat | '+names[planner])
 if planner in ['navfn','smac2d','theta']:
  code=code.replace('footprints.png','emitted-pose-footprints.png')
 (folder/'render_results.py').write_text(code)
 subprocess.run(['python3',str(folder/'render_results.py')],check=True,stdout=subprocess.DEVNULL)
W,H=960,700;canvas=Image.new('RGB',(W*3,H*2+80),'white');d=ImageDraw.Draw(canvas);d.text((20,12),'18F facility | 82 fixed seat requests | static planner comparison',font=ImageFont.truetype(font,35),fill=(20,20,20));d.text((20,53),'Same HOME, goals, 5 cm maps and inflation. D-hub: only the destination chair removed. No driving.',font=F,fill=(65,65,65))
colors={'D-hub':(33,108,214),'A1':(145,67,193),'A2':(191,88,178),'A3':(201,64,91),'A4':(209,134,35),'M1':(14,143,91),'M2':(0,140,155),'M3':(73,129,35)}
base=Image.open(L/'maps/all-chairs-present/map.pgm').convert('RGB')
for k,planner in enumerate(names):
 im=Image.new('RGB',(W,H),'white');dr=ImageDraw.Draw(im);s=summary[planner];dr.text((15,12),names[planner],font=FT,fill=(20,20,20));dr.text((15,48),f"Path generated: {s['generated']}/82  |  no path: {82-s['generated']}",font=F,fill=(30,30,30));dr.text((15,78),f"Output-pose footprint clear: {s['emitted_pose_footprint_clear']}/{s['generated']} generated paths *",font=F,fill=(100,65,35))
 left,top,pw,ph=15,125,930,543;im.paste(base.resize((pw,ph),Image.NEAREST),(left,top));dr=ImageDraw.Draw(im)
 def tr(x,y):return left+(x+24)/36*pw,top+(6-y)/21*ph
 folder=L if planner=='lattice' else R/planner
 for row in [x for x in rows if x['planner']==planner]:
  if row['path_generated']:
   p=json.loads((folder/'paths'/row['seat']/'path.json').read_text());dr.line([tr(*z[:2]) for z in p],fill=colors[row['zone']],width=2)
  x,y=tr(*row['goal'][:2]);c=(5,134,85) if row['path_generated'] else (220,30,40)
  if row['path_generated']:dr.ellipse((x-3,y-3,x+3,y+3),outline=c,width=1)
  else:dr.line([(x-5,y-5),(x+5,y+5)],fill=c,width=3);dr.line([(x-5,y+5),(x+5,y-5)],fill=c,width=3)
 x,y=tr(-1.865,-4.705);dr.rectangle((x-4,y-4,x+4,y+4),fill=(0,0,0));dr.text((x+5,y+4),'HOME',font=FS,fill=(0,0,0));dr.rectangle((0,0,W-1,H-1),outline=(170,170,170));canvas.paste(im,(k%3*W,k//3*H+80))
legend=Image.new('RGB',(W,H),'white');dr=ImageDraw.Draw(legend);dr.text((25,20),'How to read the comparison',font=FT,fill=(20,20,20));y=80
for z,c in colors.items():dr.line([(30,y+8),(70,y+8)],fill=c,width=4);dr.text((85,y),z,font=F,fill=(30,30,30));y+=34
y+=18
for line in ['Green goal: path generated; red X: no path.','* Polygon checker at actual emitted XY/yaw samples.','For 2D planners, intermediate yaw is not an SE2 plan.','A footprint hit here is a diagnostic, not proof that','MPPI cannot find another feasible body orientation.','','Footprint-clear samples do not prove swept clearance.','82 seat IDs share 77 distinct geometric goals.','Overview background retains all chairs; per-seat','D-hub maps remove only their destination chair.','No controller, Gazebo or physical robot was run.']:
 dr.text((25,y),line,font=F,fill=(65,65,65));y+=27
canvas.paste(legend,(W*2,H+80));canvas.save(OUT/'planner-comparison-2x3.png')
# Zone-level count table.
cols=['planner','D-hub','A1','A2','A3','A4','M1','M2','M3','total','emitted_pose_footprint_clear','emitted_pose_footprint_interference']
with (R/'zone-comparison.csv').open('w') as f:
 w=csv.DictWriter(f,cols);w.writeheader()
 for planner in names:
  s=summary[planner];w.writerow({'planner':planner,**{z:f"{s['by_zone'][z]['generated']}/{s['by_zone'][z]['total']}" for z in colors},'total':f"{s['generated']}/82",'emitted_pose_footprint_clear':s['emitted_pose_footprint_clear'],'emitted_pose_footprint_interference':s['emitted_pose_footprint_interference']})
print(json.dumps(summary),flush=True)
