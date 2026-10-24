import rclpy,math,time,json
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from pathlib import Path
rclpy.init();n=Node('door_plan_diagnostic');c=ActionClient(n,ComputePathToPose,'compute_path_to_pose');assert c.wait_for_server(timeout_sec=5);g=ComputePathToPose.Goal();g.goal.header.frame_id='map';g.goal.pose.position.x=-16.775;g.goal.pose.position.y=1.775;g.goal.pose.orientation.z=math.sin(1.570796/2);g.goal.pose.orientation.w=math.cos(1.570796/2);g.planner_id='GridBased';g.use_start=False
f=c.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=5);h=f.result();f=h.get_result_async();begin=time.monotonic();rclpy.spin_until_future_complete(n,f,timeout_sec=90);res=f.result();d={'status':res.status,'wall_s':time.monotonic()-begin,'path':[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in res.result.path.poses],'note':'Diagnostic new plan from current estimated pose, not injected into running BT'};(Path(__file__).parent/'stall-diagnosis/door-new-plan.json').write_text(json.dumps(d,indent=2));print({k:v if k!='path' else len(v) for k,v in d.items()});n.destroy_node();rclpy.shutdown()
