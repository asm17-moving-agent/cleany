from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parent;parts={json.loads((R/v/'session.json').read_text())['partition'] for v in ['baseline','xy_only','clearance']};running=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  env=(p/'environ').read_bytes().split(b'\0')
  if any(x.startswith(b'IGN_PARTITION=') and x.split(b'=',1)[1].decode() in parts for x in env):running.append({'pid':int(p.name),'executable':(p/'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')})
 except (PermissionError,FileNotFoundError,ProcessLookupError):continue
assert not running,running
files=['REPORT.md','comparison.json','comparison.csv','COMPLETED.json','selected-overrides.yaml','source-change.patch','media/theta-static-clearance-before-after.png','media/theta-mppi-doorway-before-after-2x.gif','media/theta-mppi-doorway-before-after-final.png']
audit={'owned_trial_processes_remaining':running,'artifact_sha256':{f:hashlib.sha256((R/f).read_bytes()).hexdigest() for f in files}}
(R/'final-audit.json').write_text(json.dumps(audit,indent=2));print('Owned trial processes remaining:',len(running))
