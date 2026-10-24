from pathlib import Path
from collections import Counter
import json,subprocess,os,math,hashlib,csv
R=Path(__file__).resolve().parent;L=R.parent/'facility-all-seats-lattice-static-20261016'
NATIVE=R.parent/'lattice-straight-improvement-20261016/static-only-20261016/direct_check'
folders={'lattice':L,**{n:R/n for n in ['navfn','smac2d','hybrid','theta']}}
FP=json.loads((L/'goals.json').read_text())['footprint'];combined=[];summary={}
for planner,folder in folders.items():
 rows=json.loads((folder/'results.json').read_text());assert len(rows)==82
 outdir=R/'footprint-audit'/planner;outdir.mkdir(parents=True,exist_ok=True)
 for row in rows:
  pathfile=folder/'paths'/row['seat']/'path.json';p=json.loads(pathfile.read_text()) if pathfile.exists() else []
  rec={'planner':planner,**row,'literal_pose_footprint_collision_count':None,'literal_pose_footprint_clear':None,'literal_pose_checked_count':0}
  if row['path_generated']:
   cmfolder=folder/'paths'/(row['seat'] if row['zone']=='D-hub' else 'seat-a1-01');meta=json.loads((cmfolder/'costmap-metadata.json').read_text())
   # Reuse native Nav2 footprint checker at the *actual emitted poses*, without inventing body yaw for 2D paths.
   j={'nx':meta['size'][0],'ny':meta['size'][1],'resolution':meta['resolution'],'origin':[*meta['origin'],0.0],'binary':str(cmfolder/'costmap.bin'),'footprint':FP,'mode':'snapshot','query_poses':p}
   inp=outdir/(row['seat']+'.input.json');out=outdir/(row['seat']+'.output.json');inp.write_text(json.dumps(j))
   with (outdir/(row['seat']+'.log')).open('w') as log:subprocess.run([str(NATIVE),str(inp),str(out)],check=True,stdout=log,stderr=subprocess.STDOUT,env={**os.environ,'ROS_DOMAIN_ID':'195','ROS_LOCALHOST_ONLY':'1'},timeout=15)
   q=json.loads(out.read_text());rec.update(literal_pose_footprint_collision_count=len(q['collision_indices']),literal_pose_footprint_clear=q['all_samples_valid'],literal_pose_checked_count=q['sample_count']);rec['literal_pose_collision_indices']=q['collision_indices']
  combined.append(rec)
 zones=['D-hub','A1','A2','A3','A4','M1','M2','M3'];mine=[x for x in combined if x['planner']==planner]
 summary[planner]={'requests':82,'generated':sum(x['path_generated'] for x in mine),'failed_seats':[x['seat'] for x in mine if not x['path_generated']],'emitted_pose_footprint_clear':sum(x['literal_pose_footprint_clear'] is True for x in mine),'emitted_pose_footprint_interference':sum(x['literal_pose_footprint_clear'] is False for x in mine),'by_zone':{z:{'total':sum(x['zone']==z for x in mine),'generated':sum(x['zone']==z and x['path_generated'] for x in mine)} for z in zones}}
 print(json.dumps({planner:summary[planner]}),flush=True);(R/'summary.json').write_text(json.dumps(summary,indent=2));(R/'comparison.json').write_text(json.dumps(combined,indent=2))
with (R/'comparison.csv').open('w') as f:
 keys=['planner','seat','zone','action_status','path_generated','pose_count','path_length_m','planning_s','wall_s','goal_xy_error_m','goal_yaw_error_deg','literal_pose_footprint_collision_count','literal_pose_footprint_clear','removed_chair'];w=csv.DictWriter(f,keys,extrasaction='ignore');w.writeheader();w.writerows(combined)
# Enforce paired goals/maps and costmap settings rather than just trusting identical YAMLs.
expected={x['seat']:x for x in json.loads((L/'goals.json').read_text())['goals']};checks={}
for name,folder in folders.items():
 records=json.loads((folder/'results.json').read_text());live=json.loads((folder/'verified-runtime.json').read_text());native_bins=[]
 for row in records:
  target=expected[row['seat']];assert row['goal']==target['goal'] and row['start']==json.loads((L/'goals.json').read_text())['home'] and row['map_sha256']==target['map_sha256'] and row['removed_chair']==target['removed_chair']
  cmfolder=folder/'paths'/(row['seat'] if row['zone']=='D-hub' else 'seat-a1-01');original=L/'paths'/(row['seat'] if row['zone']=='D-hub' else 'seat-a1-01');assert (cmfolder/'costmap.bin').read_bytes()==(original/'costmap.bin').read_bytes()
 checks[name]={'same82goals_start_maps_chair_removal':True,'identical_native_costmap_bytes_to_lattice':True,'costmap_parameters':live['costmap'],'planner_parameters':live['planner']}
(R/'validation.json').write_text(json.dumps({'checks':checks,'scope':'Native emitted-pose polygon checks using 72-bin cache; no interpolated swept-volume guarantee. Intermediate yaw of 2D paths is not an orientation-aware plan, so interference is a diagnostic, not proof every controller must fail. No driving. Timing collected with concurrent planners; do not rank CPU speed against earlier isolated Lattice run.','native_checker_sha256':hashlib.sha256(NATIVE.read_bytes()).hexdigest()},indent=2))
