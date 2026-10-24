from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap
def generate_launch_description():
    p = Path(__file__).resolve().parent
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/gazebo_fortress.launch.py')),
            launch_arguments={'world':str(p/'world.sdf'), 'headless':'true', 'use_sim_time':'true',
                              'sensor_profile':'lidar_depth_nav', 'sensor_config':str(p/'config/sensor_tf.yaml'),
                              'odometry_source':'wheel', 'wheel_odometry_config':str(p/'config/wheel_odometry.yaml'),
                              'odometry_error_config':str(p/'config/odometry_error_ideal.yaml')}.items()),
        GroupAction([SetRemap(src='voxel_guard:cmd_vel',dst='/shadow/cmd_vel'), IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),
            launch_arguments={'map':str(p/'map.yaml'), 'params_file':str(p/'config/nav2_amcl.yaml'),
                              'safety_params_file':str(p/'config/nav2_safety.yaml'),
                              'axis_motion':'false', 'omni_params_file':str(p/'config/comparison_motion.yaml'), 'output_directory':str(p/'runtime'), 'axis_params_file':str(p/'config/nav2_axis_motion.yaml'),
                              **({'body_profile':str(p/'config/cad_frame.yaml')} if (p/'config/cad_frame.yaml').exists() else {}),
                              'memory_enabled':str((p/'config/obstacle_memory.yaml').exists()).lower(),
                              'voxel_enabled':str((p/'config/voxel_guard.yaml').exists()).lower(),
                              'safety_mode':'serial',
                              **({'memory_params_file':str(p/'config/obstacle_memory.yaml')} if (p/'config/obstacle_memory.yaml').exists() else {}),
                              **({'voxel_params_file':str(p/'config/voxel_guard.yaml')} if (p/'config/voxel_guard.yaml').exists() else {})}.items())]),
        ExecuteProcess(cmd=['/usr/bin/python3','/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/mppi/20261014T002705_8e811a/wheel_limit_node.py'],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='mppi_contact_bridge',
             arguments=['/evaluation/contacts/base_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/front_left_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/front_right_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/rear_left_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/rear_right_wheel_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts', '/evaluation/contacts/head_pan_link@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],parameters=[{'use_sim_time':True}],output='screen'),
    ])
