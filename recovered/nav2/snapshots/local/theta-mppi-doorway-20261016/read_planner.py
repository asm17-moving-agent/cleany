from pathlib import Path
import rclpy,json
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
rclpy.init();n=Node('theta_planner_parameter_audit');c=n.create_client(GetParameters,'/planner_server/get_parameters');assert c.wait_for_service(timeout_sec=10)
names=['GridBased.plugin','GridBased.w_traversal_cost','GridBased.w_euc_cost','GridBased.how_many_corners'];q=GetParameters.Request();q.names=names;f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=10);assert f.done();result={k:({'type':v.type,'string_value':v.string_value,'double_value':v.double_value,'integer_value':v.integer_value}) for k,v in zip(names,f.result().values)};Path(__file__).with_name('xy-runtime-planner.json').write_text(json.dumps(result,indent=2));print(json.dumps(result));n.destroy_node();rclpy.shutdown()
