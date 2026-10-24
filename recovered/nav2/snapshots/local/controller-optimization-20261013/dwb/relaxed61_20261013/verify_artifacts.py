#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,math,re,sqlite3,xml.etree.ElementTree as ET
R=Path(__file__).parent;D=R.parent;F=R/'final5_v1';REF=D/'final61_20261013_v1/0_straight'
def read(p):return json.loads(p.read_text())
def canonical(path,drop=False):
 root=ET.parse(path).getroot()
 if drop:
  for parent in root.iter():
   for child in list(parent):
    if (child.tag=='plugin' and child.get('name')=='gz::sim::systems::Contact') or (child.tag=='sensor' and child.get('name','').startswith('benchmark_contact_')):parent.remove(child)
 s=ET.tostring(root,encoding='unicode');s=re.sub(r'/home/ehdrms/workspace/cleany/(?:controller-opt-dwb|feat-runtime-integration|main)', '<REPO>',s);return re.sub(r'>\s+<','><',s)
def main():
 rows=read(F/'metrics.json');audit={'runs':[],'owned_partition_processes_remaining':[],'expected_goal_count':5,'actual_goal_count':len(rows)}
 for x in rows:
  p=Path(x['run_directory']);par=read(p/'controller_parameters.json');v=read(p/'runtime_verified.json');dbs=[]
  for db in (p/'bag').glob('*.db3'):
   con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);dbs.append({'path':str(db),'integrity':con.execute('pragma integrity_check').fetchone()[0],'topic_message_counts':dict(con.execute('select topics.name,count(*) from messages join topics on topics.id=messages.topic_id group by topics.id').fetchall())});con.close()
  audit['runs'].append({'goal':x['goal_name'],'completed':x['completed'],'status':x.get('action_status'),'initial_GT':x['initial_pose_snapshot']['GT'],'goal_map':x['goal_map'],'owned_launch_stopped':x['owned_launch_stopped'],'owned_bag_stopped':x['owned_bag_stopped'],'bag_checks':dbs,'canonical_world_equal_excluding_contact_instrumentation':canonical(p/'world.sdf',True)==canonical(REF/'world.sdf'),'model_metadata_equal':read(p/'world.model.json')==read(REF/'world.model.json'),'map_equal':(p/'map.pgm').read_bytes()==(REF/'map.pgm').read_bytes(),'XY_goal_tolerance_m':par['goal_checker.xy_goal_tolerance'],'yaw_goal_tolerance_rad':par['goal_checker.yaw_goal_tolerance'],'full_rollout_s':par['FollowPath.sim_time'],'RotateToGoal_score_lookahead_s':par['FollowPath.RotateToGoal.lookahead_time'],'contact_sensor_count':len(read(p/'contact_instrumentation.json')['sensors']),'source_commit':read(p/'provenance.json')['source_checkout_commit'],'source_library_hashes':read(p/'provenance.json')['library_sha256'],'live_local_costmap':v['parameters']['local_costmap'],'live_global_costmap':v['parameters']['global_costmap'],'command_wheel_limit_observed':all(st['wheel_hard_cap_observed'] for st in x['command_stages'].values()),'joint_wheel_max_rad_s':x['measured_wheel_max_rad_s'],'contact_heartbeat_available':x['collision_measurement']['available']})
 partitions={read(p)['partition'] for p in D.rglob('session.json') if 'partition' in read(p)}
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   env=(p/'environ').read_bytes().split(b'\0');matched=[item.decode(errors='replace').split('=',1)[1] for item in env if item.startswith((b'IGN_PARTITION=',b'GZ_PARTITION=')) and item.decode(errors='replace').split('=',1)[1] in partitions]
   if matched:audit['owned_partition_processes_remaining'].append({'pid':int(p.name),'partitions':sorted(set(matched)),'cmd':(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')})
  except OSError:pass
 (R/'shutdown_and_bag_verification.json').write_text(json.dumps(audit,indent=2));(F/'shutdown_and_bag_verification.json').write_text(json.dumps(audit,indent=2))
 selected=[R/n for n in ['run.sh','run_relaxed.py','safety_probe_relaxed.py','prepare_contact_world.py','wheel_cap_relay.py','analyze_relaxed.py','write_reports.py','verify_artifacts.py','final_patch.yaml','final.yaml','final.patch','source_manifest.json','selection.json','pilot_inventory.json','metric_definitions.json','REPORT.md','REPRODUCE.md','shutdown_and_bag_verification.json']]+[F/n for n in ['metrics.json','metrics.csv','REPORT.md']]+[D/n for n in ['REPORT.md','comparison_metrics.json','comparison_metrics.csv','REPRODUCE.md']]
 selected += [p for p in F.glob('*/*') if p.is_file() and p.name in ['provenance.json','loaded_controller_libraries.json','controller_parameters.json','parameter_patch.yaml','runtime_verified.json','backend_parameters.json','delivery_publishers.json','runner.py']]
 (R/'artifact_hashes.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in selected if p.exists()},indent=2))
 print(json.dumps({'goal_count':len(rows),'remaining_owned_processes':audit['owned_partition_processes_remaining'],'bags':[{'goal':r['goal'],'integrity':[d['integrity'] for d in r['bag_checks']],'geometry_equal':r['canonical_world_equal_excluding_contact_instrumentation'],'joint_max':r['joint_wheel_max_rad_s']} for r in audit['runs']]},indent=2))
if __name__=='__main__':main()
