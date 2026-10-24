from pathlib import Path
import json,time
R=Path(__file__).resolve().parent
for combo in ['dwb_lattice','dwb_theta']:
 done=[];active=[]
 for p in sorted((R/combo).glob('*_*')):
  if not p.is_dir():continue
  try:
   if (p/'result.json').exists():
    d=json.loads((p/'result.json').read_text());done.append((p.name,d.get('action_status'),round(d.get('navigation_sim_s',0),1),d.get('error')))
   elif (p/'live_status.json').exists():
    d=json.loads((p/'live_status.json').read_text());active.append((p.name,round(d.get('navigation_sim_s',0),1),[round(v,3) for v in d.get('truth',[])]))
   else:active.append((p.name,'startup'))
  except json.JSONDecodeError:active.append((p.name,'writing status'))
 print(combo,'done',done,'active',active)
 print('campaign done', (R/('completed-'+combo+'.json')).exists(),'postload done',(R/('postload-completed-'+combo+'.json')).exists())
 for name in ['campaign-'+combo.removeprefix('dwb_')+'.log','postload-'+combo.removeprefix('dwb_')+'.log']:
  p=R/name
  if p.exists() and 'Traceback' in p.read_text():print('ERROR',name,p.read_text()[-1800:])
print(time.strftime('%H:%M:%S'))
