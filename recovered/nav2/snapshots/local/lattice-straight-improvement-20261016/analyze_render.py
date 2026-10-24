"""Compare preserved baseline with a new baseline and planner-only candidate."""
from pathlib import Path
import bisect,csv,json,math
import yaml
from PIL import Image,ImageDraw,ImageFont,ImageChops,GifImagePlugin
ROOT=Path(__file__).resolve().parent
OLD=ROOT.parent/'planner-lattice-theta-20261016'
OUT=ROOT/'media';OUT.mkdir(exist_ok=True)
ROUTES=[(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')]
FP=[(-.238525,-.327373),(-.238525,.327427),(.238475,.327427),(.238475,-.327373)]
VARIANTS=['baseline','refined'];COLORS={'baseline':(35,101,210),'refined':(22,139,104)}
W,H=640,500;PURPLE=(151,48,178)
font='/usr/share/fonts/google-noto/NotoSans-Regular.ttf'
if not Path(font).exists():font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
F=ImageFont.truetype(font,17);F2=ImageFont.truetype(font,21)
DATA={};records=[]
def distance_metrics(poses,start,goal):
 dx,dy=goal[0]-start[0],goal[1]-start[1];den=math.hypot(dx,dy)
 return {'length_m':sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:])), 'max_cross_track_m':max(abs(dx*(p[1]-start[1])-dy*(p[0]-start[0]))/den for p in poses)}
for v in ['original','baseline','improved','refined']:
 for i,name in ROUTES:
  folder=(OLD/'lattice' if v=='original' else ROOT/v)/f'{i}_{name}'
  r=json.loads((folder/'result.json').read_text());metrics=json.loads((folder/'metrics.json').read_text());plans=json.loads((folder/'plans.json').read_text())
  rows=[]
  for row in csv.DictReader((folder/'samples.csv').open()):
   t,x,y,yaw=[float(row[k]) for k in ['sim_s','gt_x','gt_y','gt_yaw']]
   if r['navigation_sim_start_s']<=t<=r['navigation_sim_end_s']:rows.append((t-r['navigation_sim_start_s'],x,y,yaw))
  poses=[list(x[1:]) for x in rows];a,b=plans[0]['poses'][0],plans[0]['poses'][-1]
  rec={'variant':v,'route_index':i+1,'route':name,'nav2_status':r['action_status'],'sim_s':r['navigation_sim_s'],'actual_rtf':r['achieved_rtf'],'gt_xy_error_cm':r['position_error_m']*100,'gt_yaw_error_deg':r['yaw_error_deg'],'initial_plan':distance_metrics(plans[0]['poses'],a,b),'actual_gt':distance_metrics(poses,r['initial_truth_world'],r['goal_map']),'plan_message_count':len(plans),'initial_plan_elapsed_s':plans[0]['sim_s']-r['navigation_sim_start_s'],'smoothness':r['smoothness'],'command_sim_interval_s':r.get('command_sim_interval_s'),'max_wheel_command_rad_s':r['max_command_wheel_rad_s']['/cmd_vel'],'max_joint_rad_s':metrics.get('measured_joint_max_rad_s'),'contacts':metrics.get('contacts'),'compute':{k:val for k,val in metrics.items() if 'compute' in k or 'timing' in k}}
  rec['initial_plan_start_pose']=a
  rec['initial_plan_end_pose']=b
  rec['initial_truth_world']=r['initial_truth_world']
  rec['initial_plan_start_distance_from_initial_gt_m']=math.dist(a[:2],r['initial_truth_world'][:2])
  rec['verified_planner_parameters']=r.get('verified_common_planner_parameters')
  rec['gt_yaw_total_variation_deg']=sum(abs(math.degrees(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))) for a,b in zip(poses,poses[1:]))
  rec['initial_plan_yaw_total_variation_deg']=sum(abs(math.degrees(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))) for a,b in zip(plans[0]['poses'],plans[0]['poses'][1:]))
  records.append(rec)
  if v in VARIANTS:DATA[v,i]={'result':r,'rows':rows,'times':[x[0] for x in rows],'plans':plans,'stamps':[p['sim_s'] for p in plans],'duration':r['navigation_sim_s'],'rec':rec,'folder':folder}
# Confirm only planner inputs changed; normalize artifact destinations only.
def normalized_controller(folder):
 d=yaml.safe_load((folder/'parameters.yaml').read_text())['controller_server'];d['ros__parameters']['FollowPath'].pop('benchmark_timing_file',None);return d
checks=[]
for i,name in ROUTES:
 b=DATA['baseline',i];a=DATA['refined',i]
 checks.append({'route':name,'controller_parameters_equal':normalized_controller(b['folder'])==normalized_controller(a['folder']),'loaded_mppi_binary_equal':b['result']['loaded_mppi_libraries_sha256']==a['result']['loaded_mppi_libraries_sha256'],'world_source_equal':b['result']['scene_validation']['canonical_world_sha256']==a['result']['scene_validation']['canonical_world_sha256'],'map_equal':(b['folder']/'map.pgm').read_bytes()==(a['folder']/'map.pgm').read_bytes(),'behavior_tree_equal':(b['folder']/'behavior.xml').read_bytes()==(a['folder']/'behavior.xml').read_bytes(),'goal_equal':b['result']['goal_map']==a['result']['goal_map'],'goal_tolerances_maintained':all(d['result']['verified_relaxed_parent_parameters']['goal_checker.xy_goal_tolerance']==.3 and abs(d['result']['verified_relaxed_parent_parameters']['goal_checker.yaw_goal_tolerance']-math.radians(10))<1e-9 for d in [b,a]),'wheel_command_limit_maintained':all(d['rec']['max_wheel_command_rad_s']<=5.000001 for d in [b,a]),'refined_initial_plan_goal_yaw_exact':abs(math.atan2(math.sin(a['plans'][0]['poses'][-1][2]-a['result']['goal_map'][2]),math.cos(a['plans'][0]['poses'][-1][2]-a['result']['goal_map'][2])))<1e-5})
assert all(all(v for k,v in row.items() if k!='route') for row in checks),checks
(ROOT/'comparison.json').write_text(json.dumps({'records':records,'checks':checks,'notes':['Original experiment and fresh baseline are separate.','Only planner parameters/primitive file changed; MPPI binary, parameters, footprint and goal tolerance held.','Sequential runs, requested RTF 2; actual RTF/control periods reported, stochastic single trials.','First-plan cross-track is measured against that plan own endpoints; GT cross-track against actual initial GT and requested goal.','No observed contacts is not proof of zero geometric interference.']},indent=2))
with (ROOT/'comparison.csv').open('w') as f:
 keys=['variant','route_index','route','nav2_status','sim_s','actual_rtf','gt_xy_error_cm','gt_yaw_error_deg','initial_plan_max_cross_track_cm','actual_gt_max_cross_track_cm','gt_yaw_total_variation_deg','linear_jerk_rms','angular_jerk_rms','max_wheel_command_rad_s','max_joint_rad_s'];w=csv.DictWriter(f,keys);w.writeheader()
 for r in records:w.writerow({**{k:r[k] for k in keys if k in r},'initial_plan_max_cross_track_cm':r['initial_plan']['max_cross_track_m']*100,'actual_gt_max_cross_track_cm':r['actual_gt']['max_cross_track_m']*100,'linear_jerk_rms':r['smoothness'].get('linear_jerk_rms_m_s3'),'angular_jerk_rms':r['smoothness'].get('angular_jerk_rms_rad_s3')})
BACKGROUNDS={}
for i,name in ROUTES:
 points=[]
 for v in VARIANTS:
  d=DATA[v,i];points.extend(x[1:3] for x in d['rows']);points.extend(x[:2] for x in d['plans'][0]['poses']);points.append(d['result']['goal_map'][:2])
 x0,x1=min(p[0] for p in points)-.55,max(p[0] for p in points)+.55;y0,y1=min(p[1] for p in points)-.55,max(p[1] for p in points)+.55
 scale=min((W-40)/(x1-x0),(H-155)/(y1-y0));pw,ph=round((x1-x0)*scale),round((y1-y0)*scale);left,top=(W-pw)//2,140+(H-150-ph)//2
 def tr(x,y,x0=x0,y1=y1,scale=scale,left=left,top=top):return left+(x-x0)*scale,top+(y1-y)*scale
 folder=DATA['baseline',i]['folder'];cfg=yaml.safe_load((folder/'map.yaml').read_text());img=Image.open(folder/'map.pgm').convert('L');mx,my,_=cfg['origin'];res=cfg['resolution'];crop=img.crop((round((x0-mx)/res),round(img.height-(y1-my)/res),round((x1-mx)/res),round(img.height-(y0-my)/res)));crop=crop.point(lambda v:55 if v<100 else (220 if v<240 else 249)).convert('RGB').resize((pw,ph),Image.Resampling.NEAREST);bg=Image.new('RGB',(W,H),(249,249,249));bg.paste(crop,(left,top));draw=ImageDraw.Draw(bg);gx,gy=tr(*d['result']['goal_map'][:2]);rad=.3*scale;draw.ellipse((gx-rad,gy-rad,gx+rad,gy+rad),outline=(20,150,90),width=2);draw.rectangle((0,0,W-1,H-1),outline=(150,150,150));BACKGROUNDS[i]=bg,tr

def polygon(pose,tr):
 x,y,yaw=pose;return [tr(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw)) for u,v in FP]
def dashed(draw,points):
 dist=0.
 for a,b in zip(points,points[1:]):
  length=math.dist(a,b);used=0.
  if length<1e-8:continue
  while used<length-1e-8:
   phase=dist%12;step=min(length-used,8-phase if phase<8 else 12-phase)
   if step<1e-8:step=min(length-used,1e-6)
   if phase<8:draw.line([tuple(a[k]+(b[k]-a[k])*u/length for k in [0,1]) for u in [used,used+step]],fill=PURPLE,width=2)
   used+=step;dist+=step

def cell(v,i,t=None,planned_footprints=False):
 data=DATA[v,i];r=data['result'];rec=data['rec'];bg,tr=BACKGROUNDS[i];im=bg.copy();color=COLORS[v];rows=data['rows'];duration=data['duration'];time=duration if t is None else min(t,duration);end=max(1,bisect.bisect_right(data['times'],time));selected=rows[:end]
 plan=data['plans'][0] if t is None else next((p for p in reversed(data['plans']) if p['sim_s']<=r['navigation_sim_start_s']+time),None)
 overlay=Image.new('RGBA',im.size);d=ImageDraw.Draw(overlay);last=None
 footprint_poses=plan['poses'] if planned_footprints and plan else [x[1:] for x in selected]
 for p in footprint_poses:
  if last and math.dist(p[:2],last[:2])<.16 and abs(math.atan2(math.sin(p[2]-last[2]),math.cos(p[2]-last[2])))<.2:continue
  d.polygon(polygon(p,tr),fill=(*(PURPLE if planned_footprints else color),12),outline=(*(PURPLE if planned_footprints else color),100));last=p
 im=Image.alpha_composite(im.convert('RGBA'),overlay).convert('RGB');d=ImageDraw.Draw(im)
 if plan:dashed(d,[tr(*p[:2]) for p in plan['poses']])
 if len(selected)>1:d.line([tr(x[1],x[2]) for x in selected[::2]]+[tr(*selected[-1][1:3])],fill=color,width=3)
 d.polygon(polygon(rows[0][1:],tr),outline=(100,100,100),width=2);d.polygon(polygon(selected[-1][1:],tr),fill=color,outline=(20,20,20));x,y,yaw=selected[-1][1:];d.line([tr(x,y),tr(x+.23*math.cos(yaw),y+.23*math.sin(yaw))],fill=(255,255,255),width=2)
 title='Baseline' if v=='baseline' else 'Improved Lattice'
 d.text((12,8),f'{title} + unchanged MPPI | {i+1}. {dict(ROUTES)[i]}',font=F2,fill=color)
 line='Purple dashed: first actual /plan | solid: actual GT' if t is None else 'Purple dashed: latest actual /plan | solid: actual GT'
 d.text((12,37),line,font=F,fill=(65,45,70));d.text((12,61),'Outlines: '+('initial planned footprint' if planned_footprints else 'recorded GT footprint')+' | padding 0',font=F,fill=(60,60,60))
 if t is None:
  d.text((12,85),f'Plan bend {rec["initial_plan"]["max_cross_track_m"]*100:.1f}cm | GT bend {rec["actual_gt"]["max_cross_track_m"]*100:.1f}cm',font=F,fill=(40,40,40))
  d.text((12,109),f'{duration:.2f} sim s | final {rec["gt_xy_error_cm"]:.1f}cm / {rec["gt_yaw_error_deg"]:.1f}deg | status {r["action_status"]}',font=F,fill=(40,40,40))
 else:
  state='RUNNING' if t<duration else ('SUCCEEDED' if r['action_status']==4 else 'ENDED')
  d.text((12,85),f'{time:.1f}/{duration:.1f} sim s | {state} | replay 2x',font=F,fill=(40,40,40));d.text((12,109),f'GT distance to goal {math.dist((x,y),r["goal_map"][:2]):.2f}m',font=F,fill=(40,40,40))
 return im
for pf,name in [(False,'before-after-gt-footprints.png'),(True,'before-after-planned-footprints.png')]:
 im=Image.new('RGB',(W*4,H*2))
 for row,v in enumerate(VARIANTS):
  for col,(i,n) in enumerate(ROUTES):im.paste(cell(v,i,planned_footprints=pf),(W*col,H*row))
 im.save(OUT/name)
 for i,n in ROUTES:
  pair=Image.new('RGB',(W,H*2))
  for row,v in enumerate(VARIANTS):pair.paste(cell(v,i,planned_footprints=pf),(0,H*row))
  pair.save(OUT/f'{i+1:02d}-{n}-{name}')
# Fixed simulation timestamp replay: .4 simulated seconds per 200 ms frame.
colors=[(i,i,i) for i in range(240)]+[(249,249,249),(255,255,255),*COLORS.values(),PURPLE,(20,150,90)]
colors += [(0,0,0)]*(256-len(colors));palette=Image.new('P',(1,1));palette.putpalette([x for c in colors for x in c])
duration=max(d['duration'] for d in DATA.values());count=math.ceil(duration/.4)+1;previous=None;path=OUT/'before-after-2rows-4cols-2x.gif'
with path.open('wb') as output:
 for frame in range(count):
  canvas=Image.new('RGB',(W*4,H*2))
  for row,v in enumerate(VARIANTS):
   for col,(i,n) in enumerate(ROUTES):canvas.paste(cell(v,i,frame*.4),(W*col,H*row))
  indexed=canvas.quantize(palette=palette,dither=Image.Dither.NONE)
  if previous is None:
   for chunk in GifImagePlugin.getheader(indexed,info={'loop':0,'duration':200})[0]:output.write(chunk)
   box=(0,0,*indexed.size)
  else:box=ImageChops.difference(indexed,previous).getbbox() or (0,0,1,1)
  for chunk in GifImagePlugin.getdata(indexed.crop(box),offset=box[:2],duration=200,disposal=1):output.write(chunk)
  previous=indexed
 output.write(b';')
(ROOT/'media-manifest.json').write_text(json.dumps({'pngs':list(p.name for p in OUT.glob('*.png')),'gif':path.name,'frames':count,'dimensions':[W*4,H*2],'frame_ms':200,'speed':2,'type':'Actual recorded GT and /plan replay, not a GUI recording','static':'Initial plan plus full GT; planned-footprint variant uses actual recorded plan yaw','animation':'Latest plan received by simulation timestamp, with true GT footprints; finished runs hold final pose'},indent=2))
print(json.dumps({'records':len(records),'checks':checks,'gif_frames':count,'output':str(OUT)}),flush=True)
