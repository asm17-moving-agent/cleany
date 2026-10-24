"""Read-only ownership scan using exact saved partitions and recorded runner PIDs."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sessions=[json.loads(p.read_text()) for p in ROOT.glob('*/*/session.json')]
sessions += [json.loads(p.read_text()) for p in (ROOT.parent/'sol61_20261013/01_baseline_resume_0512').glob('*/session.json')]
partitions={s['partition'] for s in sessions}
owners={s['supervisor_pid'] for s in sessions}
found=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        env=(p/'environ').read_bytes().split(b'\0'); cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        matches=[x for x in partitions if any(v == ('IGN_PARTITION='+x).encode() or v == ('GZ_PARTITION='+x).encode() for v in env)]
        if matches or int(p.name) in owners and '/teb/' in cmd and 'runner.py' in cmd:
            found.append({'pid':int(p.name),'cmd':cmd,'partitions':matches})
    except (OSError,ValueError):pass
print(json.dumps(found,indent=2))
