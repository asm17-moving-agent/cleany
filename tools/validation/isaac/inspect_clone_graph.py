"""Persist the shared ROS graph before sending any trajectory commands."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.action import get_action_names_and_types
rclpy.init()
node=rclpy.create_node('cleany_clone_graph_probe')
end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
result={'nodes':node.get_node_names_and_namespaces(),'topics':node.get_topic_names_and_types(),
        'services':node.get_service_names_and_types(),'actions':get_action_names_and_types(node)}
Path('/output/evaluations/ros-graph.json').write_text(json.dumps(result,indent=2)+'\n')
node.destroy_node();rclpy.shutdown()
