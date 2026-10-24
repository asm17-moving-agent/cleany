from pathlib import Path
import json,hashlib,shutil,subprocess
W=Path('/home/ehdrms/workspace/cleany/nav2-tuning-recovered');O=W/'recovered/nav2';O.mkdir(parents=True,exist_ok=True);(O/'COLCON_IGNORE').touch()
R=Path('/home/ehdrms/workspace/cleany-data/runs');selected=[];errors=[]
skip={'bag','bags','build','install','log','runtime','snapshots','__pycache__','.git','media','recording','python-deps','quality-checks','visualizations'}
exts={'.py','.cpp','.hpp','.h','.yaml','.yml','.xml','.json','.patch','.sh','.md','.txt','.cmake','.cfg','.ini','.rst','.csv'}
roots=[p for p in R.iterdir() if p.is_dir() and p.name.startswith(('controller-','planner-','lattice-','theta-','smac','standard-planner','facility-'))]
for root in roots:
 for p in root.rglob('*'):
  rel=p.relative_to(R)
  if not p.is_file() or p.is_symlink() or skip.intersection(rel.parts):continue
  if p.suffix.lower() not in exts and p.name not in ['CMakeLists.txt','LICENSE','Makefile']:continue
  n=p.stat().st_size
  if n>4_000_000:continue
  if p.suffix.lower()=='.json' and n>128_000 and p.name not in ['output.json','existing-stock-144.json','existing-tuned-272.json']:continue
  if p.suffix.lower()=='.csv' and (n>128_000 or any(x in p.name for x in ['samples','commands','timing','candidate_costs','velocity'])):continue
  dest=O/'snapshots/local'/rel;dest.parent.mkdir(parents=True,exist_ok=True)
  try:
   shutil.copy2(p,dest);sha=hashlib.sha256(p.read_bytes()).hexdigest()
  except OSError as e:errors.append({'source':str(p),'error':str(e)});continue
  selected.append({'source':str(p),'destination':str(dest.relative_to(W)),'sha256':sha,'bytes':n})
(O/'provenance').mkdir(exist_ok=True);(O/'provenance/local-files.json').write_text(json.dumps(selected,indent=2));(O/'provenance/local-read-errors.json').write_text(json.dumps(errors,indent=2))
m=Path('/home/ehdrms/.codex/worktrees/mppi-optimization/main')
shutil.copytree(m/'tools/navigation_evaluation/mppi_humble_experiment',O/'snapshots/mppi-local-20261013/mppi_humble_experiment',dirs_exist_ok=True)
# Git reads run outside the namespace to retain the host ownership checks.
print('local snapshots:',len(selected),'files,',sum(x['bytes'] for x in selected),'bytes; errors:',len(errors))
