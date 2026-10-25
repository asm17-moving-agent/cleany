"""Verify an already completed real Mission's conservative review outcome."""
import json
import math
from pathlib import Path
import sqlite3
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from cleany_interfaces.msg import MissionResult, MissionStatus
from nav_msgs.msg import Odometry
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('isaac_review_validation')
seen = {}
samples = []
qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(MissionResult,'/mission/result',lambda m:seen.update(result=m),qos)
node.create_subscription(MissionStatus,'/mission/status',lambda m:seen.update(status=m),qos)
node.create_subscription(Odometry,'/wheel/odom',lambda m:samples.append((time.monotonic(),m)),20)
report = {'passed':False,'scope':'sensor-driven review and return; no physical collection'}
try:
    deadline = time.monotonic()+180
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if ('result' in seen and 'status' in seen and seen['status'].state=='IDLE'
                and seen['status'].ready and len(samples)>=20
                and samples[-1][0]-samples[0][0]>=2):
            break
    assert 'result' in seen and seen['status'].ready
    result = json.loads(seen['result'].report_json)
    report['mission_result'] = result
    assert result['outcome']=='HUMAN_REVIEW_REQUIRED' and result['needs_human_review']
    assert result['return_result']=='OK' and not result['completed_tasks']
    assert result['cleaning_mode']=='isaac' and result['skipped_tasks']
    assert result['before_observation']!=result['after_observation']
    with sqlite3.connect('file:/output/executions.db?mode=ro',uri=True) as db:
        report['manipulation_executions'] = db.execute('SELECT count(*) FROM executions').fetchone()[0]
    assert report['manipulation_executions']==0
    recent = [m for t,m in samples if t>samples[-1][0]-1.]
    assert len(recent)>=10 and time.monotonic()-samples[-1][0]<.8
    assert all(math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)<.03
        and abs(m.twist.twist.angular.z)<.05 for m in recent)
    position = samples[-1][1].pose.pose.position
    home = json.loads(Path('/output/config/room.json').read_text())['home']
    report['home_error_m'] = math.hypot(position.x-home[0],position.y-home[1])
    assert report['home_error_m']<.08
    report['final_status'] = message_to_ordereddict(seen['status'])
    report['passed'] = True
    print(json.dumps(report,indent=2),flush=True)
except Exception as error:
    report['error'] = repr(error)
    raise
finally:
    Path('/output/evaluations/review.json').write_text(json.dumps(report,indent=2))
    node.destroy_node()
    rclpy.shutdown()
