"""Audit recorded final evidence without assigning navigation pass/fail scores."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
results=json.loads((ROOT/'results.json').read_text())
frozen=json.loads((ROOT/'frozen_overrides.json').read_text())
checks=[]
def check(name,value,details=None):
    checks.append({'check':name,'verified':bool(value),'details':details})
check('exactly five unique goal measurements',len(results)==5 and sorted(r['goal_index'] for r in results)==list(range(5)))
for r in results:
    run=ROOT/'final'/f"{r['goal_index']}_{r['goal_name']}"
    events=json.loads((run/'events.json').read_text())
    label=r['goal_name']
    check(label+' one navigation goal issued',sum(e['phase']=='comparison_navigation' for e in events)==1)
    check(label+' complete terminal evidence',r['measurement_completed'])
    check(label+' command wheel <=5',r['max_command_wheel_rad_s']['/cmd_vel']<=5+1e-8)
    check(label+' joint wheel <=5',r['actual_wheel_feedback_rad_s']['over_5_plus_1e8_count']==0)
    check(label+' contact stream attributable',r['actual_contact_count'] is not None,r['contact_audit']['ground_pair_samples'])
    check(label+' final stopped',max(map(abs,r['final_actual_velocity']))<1e-5 and max(map(abs,r['command_stage_audit']['/cmd_vel']['last_command']))<1e-5)
    check(label+' hard watchdog unchanged',r['hard_timeout_sim_s']==1200 and r['hard_timeout_wall_s']==1800)
    check(label+' no pilot segment limit',not r.get('pilot_segment_limit_sim_s'))
    for name,digest in frozen['scripts_sha256'].items():
        check(label+' frozen '+name,hashlib.sha256((run/name).read_bytes()).hexdigest()==digest)
    world=ET.parse(run/'world.sdf').getroot().find('world')
    reference=ET.parse(run/'world_without_contact_sensors.sdf').getroot().find('world')
    def geometries(w):
        return [ET.tostring(c,encoding='unicode') for m in w.findall('model') for link in m.findall('link') for c in link.findall('collision')]
    check(label+' collision geometry unchanged',geometries(world)==geometries(reference))
    drive=world.find("model/plugin[@name='ignition::gazebo::systems::MecanumDrive']")
    check(label+' coupled limiter owns acceleration',drive.find('min_acceleration') is None and drive.find('max_acceleration') is None)
    check(label+' internal feasibility enabled',r['verified_teb_parameters']['FollowPath.feasibility_check_no_poses']==5 and r['verified_teb_parameters']['FollowPath.include_costmap_obstacles'])
    check(label+' complete rosbag', (run/'bag/metadata.yaml').exists())
summary={'all_verified':all(x['verified'] for x in checks),'checks':checks,'note':'These are evidence and implementation checks. Navigation outcomes remain descriptive; no navigation acceptance score is assigned.'}
(ROOT/'artifact_checks.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({'all_verified':summary['all_verified'],'checks':len(checks),'failed':[x for x in checks if not x['verified']]}))
if not summary['all_verified']:raise SystemExit(1)
