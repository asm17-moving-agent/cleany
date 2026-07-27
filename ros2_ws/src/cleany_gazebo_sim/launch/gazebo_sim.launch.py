from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_gazebo_sim.world_generator import materialize_articulated_roller_world


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    mujoco_hardware = (
        Path(get_package_share_directory('cleany_mujoco_sim')) / 'hardware'
    )
    world_template = package_share / 'worlds' / 'cleany_mecanum_prototype.sdf'
    default_world = materialize_articulated_roller_world(world_template)
    base_config = package_share / 'config' / 'base.yaml'
    bridge_config = package_share / 'config' / 'bridge.yaml'

    world_arg = DeclareLaunchArgument('world', default_value=str(default_world))
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', LaunchConfiguration('world')],
        condition=IfCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        condition=UnlessCondition(LaunchConfiguration('headless')),
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_parameter_bridge',
        parameters=[{'config_file': str(bridge_config)}],
        output='screen',
    )
    command_guard = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_command_guard',
        name='gazebo_command_guard',
        parameters=[base_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    odom_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_odom_tf_publisher',
        name='gazebo_odom_tf_publisher',
        parameters=[base_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription(
        [
            world_arg,
            headless_arg,
            use_sim_time_arg,
            # Reuse the MuJoCo package's source mesh directory instead of
            # committing duplicate, large STL assets to this package.
            AppendEnvironmentVariable(
                'IGN_GAZEBO_RESOURCE_PATH', str(mujoco_hardware)
            ),
            server,
            gui,
            bridge,
            command_guard,
            odom_tf,
        ]
    )
