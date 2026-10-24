"""Targeted checks for the regression fix; no five-repeat campaign."""
from pathlib import Path
import json,os,subprocess
R=Path(__file__).resolve().parent
checks=[('fault_recovery',0,214,'12:28'),('replan_fix_turn_arrival',1,215,None),('replan_fix_m1',7,216,None)]
for name,goal,domain,hold in checks:
 target=R/name
 if target.exists():raise RuntimeError('Refuse to overwrite '+str(target))
 env=os.environ.copy();env.update(MPPI_INSTALL='candidate_v2/install',MPPI_EXPERIMENT_MODE='pathseed',MPPI_CONFIG_OVERRIDE=str(R/'prealign.yaml'),MPPI_APPROACH_CONFIG=str(R/'portal_approaches.yaml'),MPPI_BT_OVERRIDE=str(R/'recovery_keep_path.xml'))
 if goal==7:env.update(MPPI_CONFIG_OVERRIDE=str(R/'prealign_v2.yaml'),MPPI_APPROACH_CONFIG=str(R/'portal_approaches_v2.yaml'))
 env.pop('MPPI_TEST_HOLD',None)
 if hold:env['MPPI_TEST_HOLD']=hold
 (R/'postchecks_active.json').write_text(json.dumps({'name':name,'goal_index':goal,'hold':hold,'domain':domain},indent=2))
 with (R/(name+'_supervisor.log')).open('w') as log:
  ret=subprocess.run(['bash',str(R/'run.sh'),'--output',str(target),'--goal',str(goal),'--domain',str(domain)],env=env,stdout=log,stderr=subprocess.STDOUT).returncode
 if not (target/'result.json').exists():raise RuntimeError(f'{name} exited {ret} without result')
 for script in ['summarize_run.py','audit_run.py','analyze_recovery.py']:
  subprocess.run(['python3',str(R/script),str(target)],check=True,stdout=subprocess.DEVNULL)
 m=json.loads((target/'metrics.json').read_text());print(json.dumps({'name':name,'nav':m['action_status'],'gt_xy':m['position_error_m'],'gt_yaw':m['yaw_error_deg'],'success':m['success']}),flush=True)
 if not m['success']:raise RuntimeError(name+' failed; retain evidence and diagnose')
 if hold:
  e=json.loads((target/'recovery_evidence.json').read_text())
  if not e['route_timeout_log_count'] or not e['backup_completed_log_count']:raise RuntimeError('Fault run did not demonstrate timeout and successful backup')
(R/'postchecks_completed.json').write_text(json.dumps({'completed':[c[0] for c in checks]},indent=2))
