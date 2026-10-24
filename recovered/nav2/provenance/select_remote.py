from pathlib import Path
import gzip,json,collections
R=Path(__file__).parent
roots=('controller-optimization-20261013/','controller-comparison-20261013/','controller-comparison-8goals-20261014/','doorway-diagnosis-20261014/','planner-lattice-theta-20261016/','lattice-straight-improvement-20261016/','lattice-official-regeneration-20261016/','theta-mppi-doorway-20261016/','facility-planners-static-20261016/','facility-all-seats-lattice-static-20261016/')
skip={'build','install','log','__pycache__','.git','python-deps','validation-deps','bag','bags','.pytest_cache','.ruff_cache','node_modules','runtime','snapshots','media','visualizations'}
exts={'.py','.cpp','.hpp','.h','.yaml','.yml','.xml','.json','.patch','.sh','.md','.txt','.cmake','.cfg','.ini','.rst','.csv'}
selected=[]
for line in gzip.open(R/'data-manifest.jsonl.gz','rt'):
 j=json.loads(line);p=Path(j['path'])
 if j.get('type')!='file' or not str(p).startswith(roots) or skip.intersection(p.parts):continue
 if 'source' in p.parts or 'rpp-source' in p.parts:
  if not any(k in str(p) for k in ['/cleany_navigation/','/cleany_gazebo_sim/','/measurement_ws/']):continue
 if p.suffix.lower() not in exts and p.name not in ['CMakeLists.txt','LICENSE','LICENSE.txt','package.xml','setup.cfg','setup.py','Makefile','READY']:continue
 if j.get('bytes',0)>4_000_000:continue
 if p.suffix.lower()=='.json' and j.get('bytes',0)>128_000 and p.name not in ['output.json','existing-stock-144.json','existing-tuned-272.json']:continue
 if p.suffix.lower()=='.csv' and (j.get('bytes',0)>128_000 or any(x in p.name for x in ['samples','commands','timing','candidate_costs','velocity'])):continue
 selected.append(j)
(R/'remote-selection.json').write_text(json.dumps(selected,indent=2))
(R/'remote-selection.txt').write_text('\n'.join(j['path'] for j in selected)+'\n')
print(len(selected),'files',sum(j.get('bytes',0) for j in selected),'bytes')
print(collections.Counter(j['path'].split('/')[0] for j in selected))
