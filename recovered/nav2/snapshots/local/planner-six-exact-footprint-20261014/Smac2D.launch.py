from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
def generate_launch_description():
 p=Path(__file__).resolve().parent
 return LaunchDescription([Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[str(p/'Smac2D.yaml')]),Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[str(p/'Smac2D.yaml')]),Node(package='tf2_ros',executable='static_transform_publisher',arguments=['-1.865','-4.705','0','1.5708','0','0','map','base_link'])])
