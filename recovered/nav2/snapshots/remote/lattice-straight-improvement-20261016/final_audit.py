from pathlib import Path
import json,hashlib
ROOT=Path(__file__).resolve().parent
running=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  env=(p/'environ').read_bytes().split(b'\0')
  domain=next((x.split(b'=',1)[1].decode() for x in env if x.startswith(b'ROS_DOMAIN_ID=')),None)
  if domain in ['110','111','113','115','130','131','133','135','150','151','153','155']:running.append({'pid':int(p.name),'domain':domain,'executable':(p/'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')})
 except (PermissionError,FileNotFoundError,ProcessLookupError):continue
files=['REPORT.md','comparison.json','comparison.csv','refined-planner-override.yaml','grid5cm.json','media/before-after-gt-footprints.png','media/before-after-planned-footprints.png','media/before-after-2rows-4cols-2x.gif']
record={'owned_trial_processes_remaining':running,'sha256':{f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in files},'trial_results':{f'{v}/{p.parent.name}':json.loads(p.read_text())['action_status'] for v in ['baseline','improved','refined'] for p in (ROOT/v).glob('*/result.json')}}
assert not running,running
assert len(record['trial_results'])==12 and all(x==4 for x in record['trial_results'].values())
(ROOT/'final-audit.json').write_text(json.dumps(record,indent=2));print(json.dumps({'owned_trial_processes_remaining':running,'completed_gazebo_trials':len(record['trial_results']),'all_status_4':True}))
