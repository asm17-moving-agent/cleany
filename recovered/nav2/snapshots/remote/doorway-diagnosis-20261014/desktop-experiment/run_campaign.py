"""Sequential eight-goal regression; each goal gets a fresh world and ROS graph."""
from pathlib import Path
import argparse,json,os,subprocess,sys,time
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=ROOT/'generalization/diverse8');p.add_argument('--domain-base',type=int,default=220);p.add_argument('--profile',choices=['initial','final','general'],default='general');p.add_argument('--config',type=Path);p.add_argument('--bt',type=Path);p.add_argument('--install',default='candidate_diverse/install');p.add_argument('--order',default='0,1,2,3,4,5,6,7');args=p.parse_args()
out=args.output.resolve();out.mkdir(exist_ok=True)
common=ROOT.parents[1]/'controller-comparison-8goals-20261014/common'
goals=json.loads((common/'goals.locked.json').read_text())['goals']
order=[int(x) for x in args.order.split(',')]
if sorted(order)!=list(range(8)):raise ValueError('--order must contain each goal exactly once')
goals=[next(g for g in goals if g['index']==i) for i in order]
results=[]
for g in goals:
 target=out/f"{g['index']}_{g['id']}"
 if target.exists():raise RuntimeError(f'Refuse to overwrite {target}')
 env=os.environ.copy();env.update(MPPI_INSTALL='candidate_v2/install',MPPI_EXPERIMENT_MODE='pathseed',MPPI_CONFIG_OVERRIDE=str(ROOT/'prealign.yaml'),MPPI_APPROACH_CONFIG=str(ROOT/'portal_approaches.yaml'))
 env['MPPI_BT_OVERRIDE']=str(ROOT/('recovery_keep_path.xml' if args.profile=='final' else 'recovery_bt.xml'))
 env.pop('MPPI_TEST_HOLD',None)
 if args.profile=='final' and g['index']==7:env.update(MPPI_CONFIG_OVERRIDE=str(ROOT/'prealign_v2.yaml'),MPPI_APPROACH_CONFIG=str(ROOT/'portal_approaches_v3.yaml'))
 if args.profile=='general':
  env.pop('MPPI_APPROACH_CONFIG',None)
  env.update(MPPI_INSTALL='candidate_v2/install',MPPI_CONFIG_OVERRIDE=str(ROOT/'generalization/common.yaml'),MPPI_BT_OVERRIDE=str(ROOT/'generalization/recovery_replan.xml'))
 if args.profile=='general':env['MPPI_INSTALL']=args.install
 if args.config:
  if args.profile!='general':raise ValueError('--config requires general profile')
  env['MPPI_CONFIG_OVERRIDE']=str(args.config.resolve())
 if args.bt:
  if args.profile!='general':raise ValueError('--bt requires general profile')
  env['MPPI_BT_OVERRIDE']=str(args.bt.resolve())
 command=['bash',str(ROOT/'run.sh'),'--output',str(target),'--goal',str(g['index']),'--domain',str(args.domain_base+g['index'])]
 (out/'active.json').write_text(json.dumps({'goal_index':g['index'],'profile':args.profile,'command':command,'started_wall':time.time()},indent=2))
 with (out/f"{g['index']}_supervisor.log").open('w') as log:
  status=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT).returncode
 if not (target/'result.json').exists():raise RuntimeError(f'Runner exited {status} without result: {target}')
 raw=json.loads((target/'result.json').read_text())
 if not raw.get('navigation_goal_sent'):raise RuntimeError(f'Startup failure: {raw.get("error")}')
 subprocess.run(['python3',str(ROOT/'summarize_run.py'),str(target)],check=True,stdout=subprocess.DEVNULL)
 subprocess.run(['python3',str(ROOT/'audit_run.py'),str(target)],check=True,stdout=subprocess.DEVNULL)
 results.append(json.loads((target/'metrics.json').read_text()))
 (out/'results.json').write_text(json.dumps(results,indent=2))
 print(json.dumps({'goal':g['id'],'status':raw.get('action_status'),'xy_m':raw.get('position_error_m'),'yaw_deg':raw.get('yaw_error_deg') }),flush=True)
(out/'completed.json').write_text(json.dumps({'runs':len(results),'finished_wall':time.time()},indent=2))
subprocess.run(['python3',str(ROOT/'analyze_campaign.py'),str(out)],check=True)
