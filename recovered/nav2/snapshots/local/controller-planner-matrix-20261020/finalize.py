from pathlib import Path
import subprocess,json,time,shlex,sys,shutil,hashlib,csv
from PIL import Image
R=Path(__file__).resolve().parent
REMOTE='/home/donglee/workspace/cleany/cleany-data/runs/controller-planner-matrix-20261020'
SSH=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8','-o','HostKeyAlias=142spp-endeavour','donglee@100.78.53.88']
query=f"from pathlib import Path; import json; r=Path({REMOTE!r}); print(json.dumps([ (r/('postload-completed-dwb_'+p+'.json')).exists() for p in ['lattice','theta']]))"
print('Waiting for both campaigns and declared load reruns.',flush=True)
while True:
 try:ready=json.loads(subprocess.check_output(SSH+['python3 -c '+shlex.quote(query)],text=True))
 except (subprocess.CalledProcessError,json.JSONDecodeError):ready=[False,False]
 if all(ready):break
 time.sleep(30)
print('Final runs ready: synchronizing measured data.',flush=True)
transport='ssh -o HostKeyAlias=142spp-endeavour -o ConnectTimeout=8'
for folder in ['dwb_lattice','dwb_theta','load-pilot']:
 subprocess.run(['rsync','-az','-e',transport,'--exclude','bag','--exclude','runtime','--exclude','snapshots','--exclude','config','--exclude','__pycache__',f'donglee@100.78.53.88:{REMOTE}/{folder}/',str(R/folder)+'/'],check=True)
for name in ['runtime-provenance.txt','environment-load-after-reduction.json','dwb-build.log','dwb-generator-tests.log','campaign-lattice.log','campaign-theta.log','postload-lattice.log','postload-theta.log','postload-completed-dwb_lattice.json','postload-completed-dwb_theta.json','restore-audit.json']:
 subprocess.run(['rsync','-az','-e',transport,f'donglee@100.78.53.88:{REMOTE}/{name}',str(R/name)],check=True)
subprocess.run([sys.executable,str(R/'summarize_matrix.py')],check=True)
print('Frozen-input audit passed. Rendering all four combinations.',flush=True)
proto=R/'media/prototypes-before-load-rerun';proto.mkdir(parents=True,exist_ok=True)
for p in (R/'media').glob('controller-planner-4rows-[12]cols-2x*'):p.rename(proto/p.name)
for script in ['render_results.py','render_static.py','write_report.py']:
 subprocess.run([sys.executable,str(R/script)],check=True)
# The renderer itself validates frame count and total playback duration after writing.
clips=list((R/'media').glob('*.gif'));assert len(clips)==9,len(clips)
checks=[]
for p in clips:
 meta=json.loads(p.with_suffix('.json').read_text());im=Image.open(p)
 assert meta['speedup']==2 and meta['sim_step_s']==.4 and meta['frame_ms']==200
 assert tuple(meta['dimensions'])==im.size and meta['actual_first_global_plan_overlay']
 checks.append(meta)
d=json.loads((R/'comparison.json').read_text());assert len(d['runs'])==32
assert len(list(csv.DictReader((R/'comparison.csv').open())))==32
assert d['condition_audit']['all_frozen_input_checks_equal']
(R/'artifact-validation.json').write_text(json.dumps({'gif_count':9,'runs':32,'frozen_input_audit':True,'gif_checks':checks,'validation':'Each GIF renderer reopens output, traverses frames and sums durations. Finalizer checks saved metadata/dimensions,32 CSV/JSON rows and paired input audit.'},indent=2))
# Refresh path-heading evidence from the final fresh Theta/DWB run, not the excluded load pilot.
x=json.loads((R/'dwb_theta/0_straight/plans.json').read_text())[0]['poses']
e=json.loads((R/'path-heading-compatibility-evidence.json').read_text());e['actual_new_theta_straight_first_plan']={'first':x[0],'middle':x[len(x)//2],'last':x[-1],'interior_zero_yaw_fraction':sum(abs(p[2])<1e-8 for p in x[:-1])/len(x[:-1])};e['source_final_run']=str(R/'dwb_theta/0_straight');(R/'path-heading-compatibility-evidence.json').write_text(json.dumps(e,indent=2))
load=[]
for combo,i,name in [('dwb_lattice',1,'turn_arrival'),('dwb_theta',0,'straight')]:
 old=json.loads((R/'load-pilot'/f'{combo}-{i}_{name}-mixed-load/result.json').read_text());new=json.loads((R/combo/f'{i}_{name}/result.json').read_text());keys=['action_status','navigation_sim_s','navigation_wall_s','achieved_rtf','position_error_m','yaw_error_deg','controller_deadline_warning_count']
 load.append({'combination':combo,'goal':name,'excluded_mixed_load':{k:old.get(k) for k in keys},'fresh_final':{k:new.get(k) for k in keys},'note':'Diagnostic before/after load change; not repeated success-rate evidence or isolated causal proof'})
(R/'load-sensitivity.json').write_text(json.dumps(load,indent=2))
manifest=[]
for p in sorted(R.rglob('*')):
 if p.is_file() and not p.is_symlink() and p.suffix in {'.gif','.png','.csv','.json','.txt'} and p.name not in {'artifact-manifest.json','FINALIZED.json'} and '__pycache__' not in p.parts:
  manifest.append({'path':str(p.relative_to(R)),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(R/'artifact-manifest.json').write_text(json.dumps(manifest,indent=2))
(R/'FINALIZED.json').write_text(json.dumps({'finished_unix':time.time(),'rows':32,'gifs':9,'summary':d['summary']},indent=2))
print('FINALIZED',json.dumps(d['summary']),flush=True)
