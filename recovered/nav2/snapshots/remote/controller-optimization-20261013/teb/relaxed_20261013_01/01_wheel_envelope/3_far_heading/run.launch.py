from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from cleany_navigation.launch_runtime import nav2_nodes
def generate_launch_description():
 p=Path(__file__).resolve().parent
 parameters=str(p/'config/relaxed_parameters.yaml')
 gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'gazebo_relaxed.launch.py')),
  launch_arguments={'world':str(p/'world.sdf'),'headless':'true','use_sim_time':'true','sensor_profile':'lidar_depth_nav',
   'sensor_config':str(p/'config/sensor_tf.yaml'),'odometry_source':'wheel','wheel_odometry_config':str(p/'config/wheel_odometry.yaml'),
   'odometry_error_config':str(p/'config/odometry_error_ideal.yaml')}.items())
 navigation=GroupAction([SetRemap(src='cmd_vel',dst='nav2/cmd_vel'),*nav2_nodes(parameters,str(p/'map.yaml'),True,True)])
 counterfactual=GroupAction([SetRemap(src='cmd_vel',dst='ignored/cmd_vel'),
  *[Node(package='cleany_navigation',executable=n,name=n,output='screen',parameters=[parameters,{'use_sim_time':True}])
    for n in ['depth_clearance','pan_motion_gate','obstacle_memory','voxel_guard']],
  Node(package='nav2_collision_monitor',executable='collision_monitor',name='collision_monitor',output='screen',parameters=[parameters,{'use_sim_time':True}]),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_safety',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['collision_monitor']}])])
 limiter=ExecuteProcess(cmd=['python3','/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/wheel_limiter.py','--ros-args','-p','use_sim_time:=true'],output='screen')
 contacts=Node(package='ros_gz_bridge',executable='parameter_bridge',name='contact_bridge',parameters=[{'config_file':str(p/'config/contact_bridge.yaml'),'use_sim_time':True}],output='screen')
 return LaunchDescription([gazebo,navigation,counterfactual,limiter,contacts])
