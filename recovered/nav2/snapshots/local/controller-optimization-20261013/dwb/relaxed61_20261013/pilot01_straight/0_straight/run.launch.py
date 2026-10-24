from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,GroupAction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node,SetRemap
def generate_launch_description():
    p = Path(__file__).resolve().parent
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/gazebo_fortress.launch.py')),
            launch_arguments={'world':str(p/'world.sdf'), 'headless':'true', 'use_sim_time':'true',
                              'sensor_profile':'lidar_depth_nav', 'sensor_config':str(p/'config/sensor_tf.yaml'),
                              'odometry_source':'wheel', 'wheel_odometry_config':str(p/'config/wheel_odometry.yaml'),
                              'odometry_error_config':str(p/'config/odometry_error_ideal.yaml')}.items()),
        GroupAction([SetRemap(src='cmd_vel',dst='shadow/cmd_vel'),IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),
            launch_arguments={'map':str(p/'map.yaml'), 'params_file':str(p/'config/nav2_amcl.yaml'),
                              'safety_params_file':str(p/'config/nav2_safety.yaml'),
                              'axis_motion':'false', 'omni_params_file':str(p/'config/comparison_motion.yaml'), 'output_directory':str(p/'runtime'), 'axis_params_file':str(p/'config/nav2_axis_motion.yaml'),
                              **({'body_profile':str(p/'config/cad_frame.yaml')} if (p/'config/cad_frame.yaml').exists() else {}),
                              'memory_enabled':str((p/'config/obstacle_memory.yaml').exists()).lower(),
                              'voxel_enabled':str((p/'config/voxel_guard.yaml').exists()).lower(),
                              'safety_mode':'serial',
                              **({'memory_params_file':str(p/'config/obstacle_memory.yaml')} if (p/'config/obstacle_memory.yaml').exists() else {}),
                              **({'voxel_params_file':str(p/'config/voxel_guard.yaml')} if (p/'config/voxel_guard.yaml').exists() else {})}.items())]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='contact_bridge',arguments=['/benchmark/contacts/0@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/1@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/2@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/3@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/4@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/5@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/6@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/7@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/8@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/9@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/10@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/11@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/12@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/13@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/14@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/15@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/16@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/17@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/18@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/19@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/20@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/21@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/22@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/23@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/24@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/25@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/26@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/27@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/28@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/29@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts', '/benchmark/contacts/30@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'],parameters=[{'use_sim_time':True}],output='screen'),
        ExecuteProcess(cmd=['python3','/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/dwb/relaxed61_20261013/wheel_cap_relay.py'],output='screen'),
    ])
