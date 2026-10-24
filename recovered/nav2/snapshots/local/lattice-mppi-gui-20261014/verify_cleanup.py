#!/usr/bin/env python3
"""Verify only this experiment's environment identities; never kill processes."""
from pathlib import Path
import json
import os

ROOT = Path(__file__).resolve().parent
partitions = {json.loads(p.read_text())['partition'] for p in ROOT.glob('**/session.json')}
remaining = []
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) == os.getpid():
        continue
    try:
        env = dict(x.split('=', 1) for x in (p / 'environ').read_bytes().decode(errors='replace').split('\0') if '=' in x)
        owned_partition = env.get('IGN_PARTITION') in partitions or env.get('GZ_PARTITION') in partitions
        if owned_partition or env.get('ROS_DOMAIN_ID') == '226':
            remaining.append({'pid': int(p.name),
                              'argv': (p / 'cmdline').read_bytes().decode(errors='replace').replace('\0', ' ').strip(),
                              'domain': env.get('ROS_DOMAIN_ID'),
                              'ign_partition': env.get('IGN_PARTITION'),
                              'gz_partition': env.get('GZ_PARTITION'),
                              'matches_owned_partition': owned_partition})
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
        continue
(ROOT / 'remaining_owned_processes.json').write_text(json.dumps(remaining, indent=2))
print(json.dumps(remaining, indent=2))
assert not remaining, 'Owned identity still active; inspect before targeted cleanup'
