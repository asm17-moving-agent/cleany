"""Recover telemetry only; does not issue/retry navigation goals."""
import json,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent;run=ROOT/'final8/4_distant';s=json.loads((run/'session.json').read_text());scenario=json.loads((run/'scenario.json').read_text())
if (run/'result.json').exists():raise SystemExit('Navigation already complete; no recorder restarted')
os.environ.update(ROS_DOMAIN_ID=str(s['domain']),ROS_LOCALHOST_ONLY='1',IGN_IP='127.0.0.1',GZ_IP='127.0.0.1',IGN_PARTITION=s['partition'],GZ_PARTITION=s['partition'])
topics=['/clock','/ground_truth/odom','/odom','/wheel/odom','/joint_states','/nav2/cmd_vel','/cmd_vel','/scan','/tf','/tf_static','/amcl_pose','/plan','/wheel_limiter/state','/gazebo_cmd_vel',*scenario['contact_topics']]
p=subprocess.Popen(['ros2','bag','record','-o',str(run/'bag_continuation'),*topics],stdout=(run/'bag_continuation.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
(run/'recorder_recovery.json').write_text(json.dumps({'recorder_pid':p.pid,'supervisor_pid':os.getpid(),'partition':s['partition'],'wall_epoch_start':time.time(),'reason':'Original recorder stopped after live SQLite read lock. Preserve original and continuation; navigation not retried; coverage gap must be reported.'},indent=2))
try:
 while not (run/'result.json').exists() and p.poll() is None:time.sleep(2)
finally:
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 info=json.loads((run/'recorder_recovery.json').read_text());info.update(recorder_exit_code=p.returncode,wall_epoch_end=time.time());(run/'recorder_recovery.json').write_text(json.dumps(info,indent=2))
