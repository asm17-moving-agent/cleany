from pathlib import Path
import json,csv,math,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from PIL import Image,ImageDraw,ImageFont
import yaml
R=Path(__file__).resolve().parent
folders={'previous272':R.parent.parent/'lattice-straight-improvement-20261016/refined/0_straight','official144':R/'trial'}
D={};fp=np.array([[-.238525,-.327373],[-.238525,.327427],[.238475,.327427],[.238475,-.327373]])
for key,f in folders.items():
 result=json.loads((f/'result.json').read_text());plans=json.loads((f/'plans.json').read_text());start=result['navigation_sim_start_s'];end=result['navigation_sim_end_s']
 rows=[x for x in csv.DictReader((f/'samples.csv').open()) if start<=float(x['sim_s'])<=end];samples=np.array([[float(x[k]) for k in ['sim_s','gt_x','gt_y','gt_yaw','raw_vx','raw_vy','raw_wz']] for x in rows]);samples[:,0]-=start
 yaw=np.unwrap(samples[:,3]);yawchange=np.degrees(yaw-yaw[0]);xy=samples[:,1:3];a=np.array(result['initial_truth_world'][:2]);b=np.array(result['goal_map'][:2]);vec=b-a
 rec={'status':result['action_status'],'sim_s':result['navigation_sim_s'],'gt_error_cm':result['position_error_m']*100,'gt_yaw_error_deg':result['yaw_error_deg'],'gt_max_deviation_cm':max(abs(np.cross(vec,z-a))/np.linalg.norm(vec) for z in xy)*100,'gt_yaw_max_departure_deg':max(abs(yawchange)),'gt_yaw_total_variation_deg':sum(abs(np.diff(np.degrees(yaw)))),'initial_plan_yaw_total_variation_deg':sum(abs(math.degrees(math.atan2(math.sin(y[2]-x[2]),math.cos(y[2]-x[2])))) for x,y in zip(plans[0]['poses'],plans[0]['poses'][1:])),'initial_plan_start':plans[0]['poses'][0],'plan_count':len(plans),'wheel_command_max':result.get('max_command_wheel_rad_s'),'actual_rtf':result.get('achieved_rtf')}
 D[key]={'r':result,'p':plans,'s':samples,'yaw':yawchange,'rec':rec}
colors={'previous272':'#2563eb','official144':'#ea580c'}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
f,axs=plt.subplots(1,3,figsize=(15,7),dpi=180);f.subplots_adjust(top=.80,bottom=.16,wspace=.35)
for ax,(key,d) in zip(axs[:2],D.items()):
 origin=np.array(d['r']['initial_truth_world'][:2]);s=d['s'];p=np.array(d['p'][0]['poses']);c=colors[key]
 ax.plot(p[:,0]-origin[0],p[:,1]-origin[1],ls='--',color='#9333b8',label='Initial global plan',lw=1.5);ax.plot(s[:,1]-origin[0],s[:,2]-origin[1],color=c,label='Actual Gazebo GT',lw=2.3)
 last=None
 for row in s:
  if last is not None and math.dist(row[1:3],last[1:3])<.09 and abs(row[3]-last[3])<.1:continue
  t=row[3];rot=np.array([[math.cos(t),-math.sin(t)],[math.sin(t),math.cos(t)]]);ax.add_patch(Polygon(fp@rot.T+row[1:3]-origin,fill=False,edgecolor=c,alpha=.24,lw=.7));last=row
 for row in [s[0],s[-1]]:
  x,y=row[1:3]-origin;ax.arrow(x,y,.18*math.cos(row[3]),.18*math.sin(row[3]),width=.009,color=c,length_includes_head=True)
 ax.set_aspect('equal');ax.set_xlim(-.5,.5);ax.set_ylim(-.4,1.6);ax.grid(alpha=.2);ax.set_xlabel('x from GT start (m)');ax.set_ylabel('y from GT start (m)');q=d['rec'];ax.set_title(('Previous tuned 272' if key=='previous272' else 'Official regenerated 144')+f'\nGT deviation {q["gt_max_deviation_cm"]:.1f} cm | yaw departure {q["gt_yaw_max_departure_deg"]:.1f} deg\n{q["sim_s"]:.2f} sim s | final {q["gt_error_cm"]:.1f} cm / {q["gt_yaw_error_deg"]:.1f} deg',loc='left',color=c);ax.legend(fontsize=8,loc='upper left')
for key,d in D.items():axs[2].plot(d['s'][:,0],d['yaw'],color=colors[key],label=key,lw=2)
axs[2].set_title('Actual chassis yaw change from start',loc='left');axs[2].set_xlabel('Elapsed simulation time (s)');axs[2].set_ylabel('Yaw change (deg)');axs[2].grid(alpha=.2);axs[2].legend()
f.suptitle('Route 1 actual driving | Lattice + unchanged MPPI',fontsize=18,fontweight='bold');f.text(.05,.06,'Footprint outlines follow recorded Gazebo ground truth, padding 0. Dashed purple is the initial received /plan.\nOne trial per setting; independent AMCL initialization and runtime timing can change the initial global plan.',fontsize=10)
f.savefig(R/'route1-actual-before-after.png',facecolor='white')
def normalized(f):
 d=yaml.safe_load((f/'parameters.yaml').read_text())['controller_server'];d['ros__parameters']['FollowPath'].pop('benchmark_timing_file',None);return d
checks={'same_controller_config':normalized(folders['previous272'])==normalized(folders['official144']),'same_mppi_binary':D['previous272']['r']['loaded_mppi_libraries_sha256']==D['official144']['r']['loaded_mppi_libraries_sha256'],'same_map':(folders['previous272']/'map.pgm').read_bytes()==(folders['official144']/'map.pgm').read_bytes(),'same_BT':(folders['previous272']/'behavior.xml').read_bytes()==(folders['official144']/'behavior.xml').read_bytes(),'same_goal':D['previous272']['r']['goal_map']==D['official144']['r']['goal_map']}
(R/'actual-comparison.json').write_text(json.dumps({'results':{k:v['rec'] for k,v in D.items()},'checks':checks,'scope':'Actual Gazebo GT, not hardware; one new route1 trial; compare initial plan start before attributing differences solely to primitives'},indent=2));print(json.dumps({'results':{k:v['rec'] for k,v in D.items()},'checks':checks},indent=2))
# Timestamp-correct 2x GT replay; 0.1 sim seconds per 50 ms GIF frame.
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf';font=ImageFont.truetype(font,16)
frames=[];duration=max(x['r']['navigation_sim_s'] for x in D.values())
for time in np.arange(0,duration+.1001,.1):
 im=Image.new('RGB',(1000,700),'white');dr=ImageDraw.Draw(im)
 for col,(key,d) in enumerate(D.items()):
  origin=np.array(d['r']['initial_truth_world'][:2]);c=colors[key];s=d['s'];s=s[s[:,0]<=min(time,d['r']['navigation_sim_s'])];s=s if len(s) else d['s'][:1]
  def tr(x,y):return col*500+250+(x-origin[0])*250,610-(y-origin[1])*250
  dr.text((col*500+15,15),key+' | recorded GT | 2x',font=font,fill=c);dr.text((col*500+15,43),f'{min(time,d["r"]["navigation_sim_s"]):.1f}/{d["r"]["navigation_sim_s"]:.1f} sim s',font=font,fill='#334155')
  p=d['p'][0]['poses'];dr.line([tr(*z[:2]) for z in p],fill='#9333b8',width=2)
  if len(s)>1:dr.line([tr(*z[1:3]) for z in s],fill=c,width=3)
  row=s[-1];t=row[3];rot=np.array([[math.cos(t),-math.sin(t)],[math.sin(t),math.cos(t)]]);body=fp@rot.T+row[1:3];dr.polygon([tr(*z) for z in body],fill=c,outline='#111827');dr.line([tr(*row[1:3]),tr(row[1]+.2*math.cos(t),row[2]+.2*math.sin(t))],fill='white',width=3)
 frames.append(im)
frames[0].save(R/'route1-actual-before-after-2x.gif',save_all=True,append_images=frames[1:],duration=50,loop=0)
