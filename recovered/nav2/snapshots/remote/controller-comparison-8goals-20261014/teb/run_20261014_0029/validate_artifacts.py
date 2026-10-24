"""Recorded input/measurement audits; no composite navigation acceptance score."""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parent
rows=json.loads((ROOT/'results.json').read_text());frozen=json.loads((ROOT/'frozen.json').read_text());checks=[]
def check(name,value,details=None):checks.append({'check':name,'verified':bool(value),'details':details})
check('eight unique goal rows',len(rows)==8 and sorted(r['goal_index'] for r in rows)==list(range(8)))
for r in rows:
 run=Path('/home/donglee/workspace/cleany')/r['run_directory'];label=r['goal_name']
 events=json.loads((run/'events.json').read_text()) if (run/'events.json').exists() else []
 issued=[e for e in events if e['phase']=='comparison_navigation']
 check(label+' at most one goal issued',len(issued)<=1)
 if not r.get('goal_issued'):
  check(label+' startup failure reported',bool(r.get('error')),r.get('error'));continue
 check(label+' exactly one goal issued',len(issued)==1)
 check(label+' goal tolerance actually loaded',r['verified_goal_checker']['goal_checker.xy_goal_tolerance']==.30 and abs(r['verified_goal_checker']['goal_checker.yaw_goal_tolerance']-.17453292519943295)<1e-12)
 check(label+' actual command wheel <=5',r['max_command_wheel_rad_s']['/cmd_vel'] is not None and r['max_command_wheel_rad_s']['/cmd_vel']<=5+1e-8)
 check(label+' actual joint wheel <=5',r['actual_wheel_feedback_rad_s'].get('over_5_plus_1e8_count')==0,r['actual_wheel_feedback_rad_s'].get('max'))
 scene=json.loads((run/'scene_validation.json').read_text())
 check(label+' scene complete and chair removal correct',not scene['missing_models'] and not scene['missing_visual_meshes'] and scene['removed_chair_absent'] and not scene['server_mesh_load_error_lines'])
 check(label+' contacts attributable',r['actual_contact_count'] is not None,r.get('actual_contact_note'))
 check(label+' stationary definition aligned',r['smoothness'].get('stationary_definition')=='hypot(vx,vy)<.01 AND abs(wz)<.02 rad/s')
 check(label+' full bag metadata saved',(run/'bag/metadata.yaml').exists())
 check(label+' hard bounds unchanged',r['hard_timeout_sim_s']==1200 and r['hard_timeout_wall_s']==1800)
 for name,value in frozen['scripts_sha256'].items():
  if (run/name).exists():check(label+' frozen '+name,hashlib.sha256((run/name).read_bytes()).hexdigest()==value)
 def geometry(w):
  return [ET.tostring(c,encoding='unicode') for m in w.findall('model') for link in m.findall('link') for c in link.findall('collision')]
 canonical=ET.parse(run/'world.canonical.sdf').getroot().find('world');actual=ET.parse(run/'world.sdf').getroot().find('world')
 # URI materialization changes literal paths only; normalize to fixture://.
 def normalize(items):return [x.replace((run/'fixture').resolve().as_uri()+'/','fixture://').replace(str((run/'fixture').resolve())+'/','fixture://') for x in items]
 check(label+' canonical collisions unchanged',normalize(geometry(actual))==geometry(canonical))
 check(label+' no parent-host path leaked','/home/ehdrms/' not in (run/'world.sdf').read_text())
 check(label+' original per-goal TEB runtime parameter query available',bool(r['verified_teb_parameters']),r.get('teb_runtime_parameter_query_note'))
 replay=json.loads((ROOT/'parameter_replay/evidence.json').read_text())['parameters']
 check(label+' same binary/profile configuration replay preserves feasibility',r.get('teb_profile_matches_configuration_replay') and replay['FollowPath.feasibility_check_no_poses']==5 and replay['FollowPath.include_costmap_obstacles'])
summary={'all_verified':all(c['verified'] for c in checks),'checks':checks,'note':'Input, implementation and measurement audit only. Navigation failures remain in results; no composite pass/fail score.'}
(ROOT/'artifact_checks.json').write_text(json.dumps(summary,indent=2));print(json.dumps({'all_verified':summary['all_verified'],'checks':len(checks),'failed':[c for c in checks if not c['verified']]}))
