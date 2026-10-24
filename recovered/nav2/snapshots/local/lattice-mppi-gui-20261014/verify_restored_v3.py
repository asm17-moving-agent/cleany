import json,rclpy
from pathlib import Path
from rcl_interfaces.srv import GetParameters
rclpy.init();n=rclpy.create_node('verify_diagnosis_restore');c=n.create_client(GetParameters,'/controller_server/get_parameters');assert c.wait_for_service(timeout_sec=5)
q=GetParameters.Request();q.names=['FollowPath.CostCritic.cost_weight','FollowPath.TwirlingCritic.cost_weight','FollowPath.vx_std','FollowPath.vy_std','FollowPath.wz_std','FollowPath.PathAlignCritic.use_path_orientations','FollowPath.visualize','progress_checker.movement_time_allowance','failure_tolerance','FollowPath.PathAlignCritic.offset_from_furthest','FollowPath.temperature','FollowPath.PathFollowCritic.cost_weight']
f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=20);assert f.result();rows={k:{'type':v.type,'double':v.double_value,'bool':v.bool_value,'integer':v.integer_value} for k,v in zip(q.names,f.result().values)}
c2=n.create_client(GetParameters,'/local_costmap/local_costmap/get_parameters');assert c2.wait_for_service(timeout_sec=5);q2=GetParameters.Request();q2.names=['depth_layer.enabled'];f2=c2.call_async(q2);rclpy.spin_until_future_complete(n,f2,timeout_sec=10);rows['local.depth_layer.enabled']=f2.result().values[0].bool_value
Path(__file__).with_name('stall-diagnosis').joinpath('restored-v3-parameters.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows));n.destroy_node();rclpy.shutdown()
