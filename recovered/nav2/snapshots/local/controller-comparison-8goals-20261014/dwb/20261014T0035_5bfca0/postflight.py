from pathlib import Path
import hashlib,json,math,os,sqlite3,time,struct
RUN=Path(__file__).resolve().parent;COMMON=RUN.parents[1]/'common';rows=json.loads((RUN/'metrics.json').read_text());lock=json.loads((COMMON/'goals.locked.json').read_text());assert len(rows)==8 and sorted(r['goal_index'] for r in rows)==list(range(8))
ctrl_ref=None;planner_ref=None;checks=[];partitions=set()
for r in rows:
 p=Path(r['run_directory']);session=json.loads((p/'session.json').read_text());partitions.add(session['partition']);scene=json.loads((p/'scene_verification.json').read_text());ctrl=json.loads((p/'controller_parameters.json').read_text());plan=json.loads((p/'planner_parameters.json').read_text())
 if ctrl_ref is None:ctrl_ref=ctrl;planner_ref=plan
 assert ctrl==ctrl_ref and plan==planner_ref,(r['goal_name'],'profile changed')
 assert r.get('navigation_started') is True and r['owned_launch_stopped'] and r['owned_bag_stopped'],r
 assert abs(ctrl['goal_checker.xy_goal_tolerance']-.3)<1e-10 and abs(ctrl['goal_checker.yaw_goal_tolerance']-math.radians(10))<1e-10
 assert math.dist(scene['initial_gt'][:2],lock['home_world_map'][:2])<.05 and abs(math.atan2(math.sin(scene['initial_gt'][2]-lock['home_world_map'][2]),math.cos(scene['initial_gt'][2]-lock['home_world_map'][2])))<.05
 bags=[]
 for db in (p/'bag').glob('*.db3'):
  c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True);integrity=c.execute('pragma quick_check').fetchall();assert integrity==[('ok',)],integrity
  clock_id=c.execute("select id from topics where name='/clock'").fetchone()[0]
  stamps=[]
  for direction in ['asc','desc']:
   blob=c.execute('select data from messages where topic_id=? order by timestamp '+direction+' limit 1',(clock_id,)).fetchone()[0];sec,ns=struct.unpack_from('<iI',blob,4);stamps.append(sec+ns*1e-9)
  covered=stamps[0]<=r['navigation_sim_start_s']+.05 and stamps[1]>=r['navigation_sim_end_s']-.05
  bags.append({'file':str(db.relative_to(RUN)),'quick_check':'ok','message_count':c.execute('select count(*) from messages').fetchone()[0],'topic_count':c.execute('select count(*) from topics').fetchone()[0],'clock_sim_bounds':stamps,'navigation_window_covered':covered,'coverage_gap_reason':None if covered else 'Recorder SqliteException database is locked; raw bag only covers a prefix. Metadata reindex cannot restore lost messages.'});c.close()
 assert bags and (p/'bag/metadata.yaml').exists(),p
 checks.append({'goal':r['goal_name'],'navigation_started_once':True,'action_status':r.get('action_status'),'timeout':r.get('timeout'),'error':r.get('error'),'GT_xy_error_m':r.get('position_error_m'),'GT_yaw_error_deg':r.get('yaw_error_deg'),'measured_joint_max_rad_s':r.get('measured_wheel_max_rad_s'),'wheel_joint_hard_cap_observed':r.get('wheel_joint_hard_cap_observed'),'common_controller_and_planner_parameters_match':True,'initial_world_pose_matches':True,'bags':bags})
remaining=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  env=(p/'environ').read_bytes().split(b'\0')
  if any(('IGN_PARTITION='+x).encode() in env or ('GZ_PARTITION='+x).encode() in env for x in partitions):remaining.append({'pid':int(p.name),'cmd':(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')})
 except OSError:pass
manifest=json.loads((COMMON/'sha256.locked.json').read_text());changed=[n for n,h in manifest.items() if hashlib.sha256((COMMON/n).read_bytes()).hexdigest()!=h]
out={'verified_epoch_s':time.time(),'goal_count':8,'goal_indices_once':True,'common_controller_and_planner_profile_identical':True,'common_input_hash_changes':changed,'remaining_exact_owned_partition_processes':remaining,'checks':checks,'all_raw_bags_cover_navigation_window':all(b['navigation_window_covered'] for x in checks for b in x['bags']),'all_measured_wheel_joint_values_le_5':all(x['wheel_joint_hard_cap_observed'] is True for x in checks),'success_claim':'Action statuses and GT errors are separate; no combined score or cross-host ranking'}
(RUN/'final_validation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2));assert not changed and not remaining
