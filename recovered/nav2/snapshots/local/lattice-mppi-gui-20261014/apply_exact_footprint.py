from pathlib import Path
import json
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters,GetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
root=Path(__file__).parent
scenario=json.loads((root/'gui-runtime-v2/7_other_room/scenario.json').read_text());lo,hi=scenario['model_metadata']['collision_bounds'] if 'model_metadata' in scenario else (None,None)
footprint=[[lo[0],lo[1]],[lo[0],hi[1]],[hi[0],hi[1]],[hi[0],lo[1]]]
rclpy.init();n=Node('exact_body_footprint_simulation');record={'footprint':footprint,'footprint_padding':0.,'scope':'current Gazebo experiment only','nodes':{}}
for name in ['/local_costmap/local_costmap','/global_costmap/global_costmap']:
 c=n.create_client(SetParameters,name+'/set_parameters');assert c.wait_for_service(timeout_sec=5)
 req=SetParameters.Request();req.parameters=[Parameter(name='footprint',value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=json.dumps(footprint))),Parameter(name='footprint_padding',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=0.))]
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);reply=[{'successful':x.successful,'reason':x.reason} for x in f.result().results];assert all(x['successful'] for x in reply),reply
 c=n.create_client(GetParameters,name+'/get_parameters');assert c.wait_for_service(timeout_sec=5);req=GetParameters.Request();req.names=['footprint','footprint_padding'];f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5);v=f.result().values;assert json.loads(v[0].string_value)==footprint and v[1].double_value==0.;record['nodes'][name]={'set_results':reply,'verified_footprint':json.loads(v[0].string_value),'verified_padding':v[1].double_value}
(root/'exact-footprint-applied.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2));n.destroy_node();rclpy.shutdown()
