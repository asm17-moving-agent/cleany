from pathlib import Path
import subprocess,json,time
R=Path(__file__).resolve().parent
children=[]
for name in ['navfn','smac2d','hybrid','theta']:
 folder=R/name
 if (folder/'results.json').exists():raise RuntimeError('Preserve existing results: '+name)
 f=(folder/'campaign.log').open('w')
 cmd=['python3',str(folder/'run_plans.py'),'--zones','D-hub','A1','A2','A3','A4','M1','M2','M3']
 p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT);children.append((name,p,f))
(R/'campaign-pids.json').write_text(json.dumps({n:p.pid for n,p,f in children},indent=2))
while any(p.poll() is None for n,p,f in children):
 statuses={n:{'running':p.poll() is None,'exit':p.poll()} for n,p,f in children}
 for name in statuses:
  file=R/name/'progress.json'
  if file.exists():statuses[name]['progress']=json.loads(file.read_text())
 (R/'campaign-status.json').write_text(json.dumps(statuses,indent=2));time.sleep(2)
for n,p,f in children:f.close()
statuses={n:{'running':False,'exit':p.returncode} for n,p,f in children}
(R/'campaign-completed.json').write_text(json.dumps(statuses,indent=2));print(json.dumps(statuses))
raise SystemExit(1 if any(p.returncode for n,p,f in children) else 0)
