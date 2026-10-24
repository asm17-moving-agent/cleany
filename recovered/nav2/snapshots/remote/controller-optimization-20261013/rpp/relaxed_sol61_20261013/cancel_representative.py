import json,os,time,sys
from pathlib import Path
run=Path(sys.argv[1]);session=json.loads((run/'session.json').read_text())
assert session['domain']==228
os.environ.update(ROS_DOMAIN_ID='228',ROS_LOCALHOST_ONLY='1',IGN_PARTITION=session['partition'],GZ_PARTITION=session['partition'],IGN_IP='127.0.0.1',GZ_IP='127.0.0.1')
import rclpy
from action_msgs.srv import CancelGoal
rclpy.init();node=rclpy.create_node('rpp_representative_cancel');client=node.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
assert client.wait_for_service(timeout_sec=10)
f=client.call_async(CancelGoal.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=10);assert f.done()
(run/'representative_early_cancel.json').write_text(json.dumps({'reason':sys.argv[2] if len(sys.argv)>2 else 'Diagnostic early cancellation','cancel_return_code':f.result().return_code,'goals_canceling':len(f.result().goals_canceling),'wall_monotonic_s':time.monotonic()},indent=2))
print(f.result());node.destroy_node();rclpy.shutdown()
