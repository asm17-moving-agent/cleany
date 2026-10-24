"""Condense actual per-critic telemetry; retain the full CSV as primary evidence."""
import argparse,csv,json,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();r=args.run
snapshots=[json.loads(p.read_text()) for p in sorted((r/'snapshots').glob('*.json'))]
groups={}
with (r/'candidate_costs.csv').open() as f:
 for row in csv.DictReader(f):groups.setdefault(row['snapshot'],[]).append({k:float(v) for k,v in row.items()})
rows=[]
for batch in groups.values():
 first=batch[0];best=min(batch,key=lambda b:b['critic_total']);t=first['sim_s']
 snapshot=min(snapshots,key=lambda s:abs(s['sim_s']-t)) if snapshots else None
 yawchanges=[math.degrees(math.atan2(math.sin(b['yaw4']-b['robot_yaw']),math.cos(b['yaw4']-b['robot_yaw']))) for b in batch]
 rows.append({'sim_s':t,'nearest_gt':snapshot['gt'] if snapshot else None,'gt_time_offset_s':snapshot['sim_s']-t if snapshot else None,'failed':int(first['failed']),'furthest':int(first['furthest']),'yaw_change_range_deg':[min(yawchanges),max(yawchanges)],'best_candidate':int(best['candidate']),'best_yaw_change_deg':yawchanges[batch.index(best)],'best_costs':{k:v for k,v in best.items() if k.endswith('Critic') or k=='critic_total'},'collision_penalized_count':sum(b['CostCritic']>1e4 for b in batch)})
(r/'candidate_summary.json').write_text(json.dumps({'cost_scope':'critic deltas before MPPI control regularization','rows':rows},indent=2))
print(json.dumps(rows[-3:],indent=2))
