from pathlib import Path
import json,subprocess,math,hashlib,csv
R=Path(__file__).resolve().parent;O=R/'static-comparison';O.mkdir(exist_ok=True)
P=Path('/home/donglee/workspace/cleany/lattice-straight-improvement-20261016')
summary=[]
for i,name in [(0,'straight'),(1,'turn_arrival'),(3,'far_heading'),(5,'lateral')]:
 original=P/'static-only-20261016/recorded-start'/f'{i}-{name}-current.input.json'
 base=json.loads(original.read_text())
 for variant,prim in [('previous272',P/'grid5cm.json'),('official144_r10',R/'omni-r10cm/output.json')]:
  j={**base,'primitive_file':str(prim)}
  stem=O/f'{i}-{name}-{variant}';inp=stem.with_suffix('.input.json');out=stem.with_suffix('.output.json');inp.write_text(json.dumps(j))
  with stem.with_suffix('.log').open('w') as log:
   p=subprocess.run([str(P/'native_probe'),str(inp),str(out)],stdout=log,stderr=subprocess.STDOUT,timeout=50)
  q=json.loads(out.read_text()) if out.exists() else {};poses=q.get('poses',[])
  row={'route_index':i+1,'route':name,'variant':variant,'exit':p.returncode,'ok':q.get('ok'),'search_ms':q.get('search_ms'),'iterations':q.get('iterations'),'input_map_sha256':hashlib.sha256(Path(j['binary']).read_bytes()).hexdigest(),'primitive_sha256':hashlib.sha256(prim.read_bytes()).hexdigest(),'start_requested':j['start'],'goal_requested':j['goal']}
  if poses:
   a,b=poses[0],poses[-1];dx,dy=b[0]-a[0],b[1]-a[1];den=math.hypot(dx,dy)
   row.update(max_cross_track_cm=max(abs(dx*(z[1]-a[1])-dy*(z[0]-a[0]))/den for z in poses)*100,length_m=sum(math.dist(x[:2],y[:2]) for x,y in zip(poses,poses[1:])),yaw_variation_deg=sum(abs(math.degrees(math.atan2(math.sin(y[2]-x[2]),math.cos(y[2]-x[2])))) for x,y in zip(poses,poses[1:])),goal_yaw_error_deg=abs(math.degrees(math.atan2(math.sin(b[2]-j['goal'][2]),math.cos(b[2]-j['goal'][2])))),start_pose=a,end_pose=b)
  summary.append(row);print(json.dumps(row),flush=True)
  (O/'comparison.json').write_text(json.dumps(summary,indent=2))
with (O/'comparison.csv').open('w') as f:
 keys=['route_index','route','variant','ok','max_cross_track_cm','length_m','yaw_variation_deg','goal_yaw_error_deg','search_ms','iterations'];w=csv.DictWriter(f,keys,extrasaction='ignore');w.writeheader();w.writerows(summary)
(O/'provenance.json').write_text(json.dumps({'scope':'Native Nav2 1.1.20 AStarAlgorithm<NodeLattice> + GridCollisionChecker on frozen global costmaps. No simulator/controller/localization started.','comparison':'Identical recorded planner starts/goals, 5cm map, footprint/padding0, planner parameters, native binary and host; only primitive JSON changes. Minimum-turning-radius heuristic follows each primitive metadata. No path smoothing.','native_probe_sha256':hashlib.sha256((P/'native_probe').read_bytes()).hexdigest(),'footprint':base['footprint'],'collision_threshold':'Native checker possible-inscribed-cost derived from shared footprint and inflation scale 6; 72 orientation bins.','timing':'Single cold-process search per combination; exploratory only, not repeatable timing benchmark.'},indent=2))
