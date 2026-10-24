from pathlib import Path
import json,yaml,hashlib,sys,argparse
root=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--campaign',default='regression8');parser.add_argument('--config',default='common.yaml');parser.add_argument('--bt',default='../recovery_keep_path.xml');args=parser.parse_args()
campaign=root/args.campaign
rows=[]
expected=yaml.safe_load((root/args.config).read_text())
for p in sorted(campaign.glob('*/result.json')):
 r=p.parent; s=json.loads((r/'scenario.json').read_text()); result=json.loads(p.read_text())
 loaded=yaml.safe_load((r/'controller_server.yaml').read_text())['/controller_server']['ros__parameters']
 stages=result['navigation_stages']; checks={
  'no_manual_approach':s['approach_stages']==[],
  'one_original_goal':len(stages)==1 and stages[0]['name']=='final_goal' and stages[0]['pose']==result['goal_map'],
  'common_BT':(r/'behavior.xml').read_bytes()==(root/args.bt).read_bytes(),
  'no_stage_checker':'stage_goal_checker' not in loaded,
  'common_parameters':all(all(loaded['FollowPath'][k][a]==b for a,b in v.items()) if isinstance(v,dict) else loaded['FollowPath'][k]==v for k,v in expected.items()),
  'constraint_audit':json.loads((r/'audit.json').read_text())['passed']}
 rows.append({'run':r.name,'checks':checks,'passed':all(checks.values()),'libraries':result['loaded_mppi_libraries_sha256']})
libs=[json.dumps(x['libraries'],sort_keys=True) for x in rows]
output={'runs':rows,'all_eight_present':len(rows)==8,'common_libraries':len(set(libs))==1,'passed':len(rows)==8 and len(set(libs))==1 and all(x['passed'] for x in rows)}
(campaign/'general_audit.json').write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2))
