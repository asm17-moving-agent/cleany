from pathlib import Path
import csv,json
import numpy as np
ROOT=Path(__file__).resolve().parent
rows=[]
for c in ['dwb','mppi','teb','rpp']:
 p=ROOT/('profile_'+c)/'0_straight/compute_timing.csv'
 samples=list(csv.DictReader(p.open()));successful=[float(s['compute_us'])/1000 for s in samples if s['success']=='1'];alltimes=[float(s['compute_us'])/1000 for s in samples]
 outcome=json.loads((p.parent/'result.json').read_text())
 rows.append({'candidate':c,'host':outcome['host'],'case':'supplemental straight-route profile','action_status':outcome.get('action_status'),'calls':len(samples),'exceptions':sum(s['success']!='1' for s in samples),'median_ms':float(np.median(successful)),'p95_ms':float(np.percentile(successful,95)),'p99_ms':float(np.percentile(successful,99)),'all_calls_p95_ms':float(np.percentile(alltimes,95)),'calls_over_50ms':sum(t>50 for t in alltimes),'note':'same ARM64 host; wall time includes scheduling; each algorithm follows its own trajectory; not a standardized frozen-input microbenchmark'})
(ROOT/'profile_compute.json').write_text(json.dumps(rows,indent=2))
with (ROOT/'profile_compute.csv').open('w') as f:w=csv.DictWriter(f,list(rows[0]));w.writeheader();w.writerows(rows)
print(json.dumps(rows,indent=2))
