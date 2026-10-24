"""Read-only completed JSON and live timing text; never opens live bag DB."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
for run in sorted((ROOT/'final8').iterdir()):
 if not run.is_dir():continue
 if (run/'result.json').exists():
  r=json.loads((run/'result.json').read_text());print(json.dumps({'goal':run.name,**{k:r.get(k) for k in ('completed','goal_issued','action_status','navigation_sim_s','navigation_wall_s','position_error_m','yaw_error_deg','error')}}));continue
 f=run/'compute_timing.csv';lines=f.read_text().splitlines() if f.exists() else [];print(json.dumps({'goal':run.name,'state':'active','last_compute_row':lines[-1] if lines else None}))
