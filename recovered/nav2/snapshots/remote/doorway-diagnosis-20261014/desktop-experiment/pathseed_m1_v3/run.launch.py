from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from cleany_navigation.launch_runtime import nav2_nodes
def generate_launch_description():
 p=Path(__file__).resolve().parent
 parameters=str(p/'parameters.yaml')
 import yaml
 camera=yaml.safe_load((p/'config/nav2_safety.yaml').read_text())['depth_camera_tf']
 adapters=[
  Node(package='ros_gz_bridge',executable='parameter_bridge',name='head_pan_bridge',arguments=['/model/cleany_mecanum/head_pan/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'],remappings=[('/model/cleany_mecanum/head_pan/cmd_pos','/head_pan/command')],output='screen'),
  Node(package='cleany_gazebo_sim',executable='gazebo_pan_tf_publisher',parameters=[camera,{'use_sim_time':True}],output='screen'),
  Node(package='tf2_ros',executable='static_transform_publisher',name='depth_optical_tf',parameters=[{'use_sim_time':True}],arguments=['--frame-id','head_camera_depth_frame','--child-frame-id','head_camera_depth_optical_frame','--roll','-1.5707963267948966','--pitch','0','--yaw','-1.5707963267948966'])]
 gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource('/home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install/cleany_gazebo_sim/share/cleany_gazebo_sim/launch/gazebo_fortress.launch.py'),
  launch_arguments={'world':str(p/'world.sdf'),'headless':'true','headless_rendering':'true','use_sim_time':'true','sensor_profile':'lidar_depth_nav',
   'sensor_config':str(p/'config/sensor_tf.yaml'),'odometry_source':'wheel','wheel_odometry_config':str(p/'config/wheel_odometry.yaml'),
   'odometry_error_config':str(p/'config/odometry_error_ideal.yaml')}.items())
 navigation=GroupAction([SetRemap(src='cmd_vel',dst='nav2/cmd_vel'),*nav2_nodes(parameters,str(p/'map.yaml'),True,True)])
 counterfactual=GroupAction([SetRemap(src='cmd_vel',dst='ignored/cmd_vel'),
  *[Node(package='cleany_navigation',executable=n,name=n,output='screen',parameters=[parameters,{'use_sim_time':True}])
    for n in ['depth_clearance','pan_motion_gate','obstacle_memory','voxel_guard']],
  Node(package='nav2_collision_monitor',executable='collision_monitor',name='collision_monitor',output='screen',parameters=[parameters,{'use_sim_time':True}]),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_safety',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['collision_monitor']}])])
 limiter=ExecuteProcess(cmd=['python3','/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment/laptop-runtime/wheel_limit_node.py'],output='screen')
 contacts=Node(package='ros_gz_bridge',executable='parameter_bridge',name='contact_bridge',arguments=['/evaluation/contacts/base_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/front_left_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/front_right_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/rear_left_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/rear_right_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/head_pan_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],parameters=[{'use_sim_time':True}],output='screen')
 snapshots=ExecuteProcess(cmd=['python3','/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment/snapshot_logger.py','/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment/pathseed_m1_v3/snapshots'],output='screen')
 return LaunchDescription([gazebo,*adapters,navigation,counterfactual,limiter,contacts,snapshots])
