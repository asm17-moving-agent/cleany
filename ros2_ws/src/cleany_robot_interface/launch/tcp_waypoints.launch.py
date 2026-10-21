"""One frozen, selected-arm controller and a matching plan-only MoveIt server."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_robot_interface.tcp_waypoints import Dataset, validate_control_contract


def setup(context):
    dataset = Dataset.load(Path(LaunchConfiguration('session').perform(context)))
    validate_control_contract(dataset)
    folder = dataset.root / 'profile'
    controller = dataset.side + '_arm_controller'
    model = {'robot_description': (folder / 'robot_control.urdf').read_text(), 'use_sim_time': False}
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[model, str(folder / 'controllers.yaml')], output='screen')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[model], output='screen')
    guard = Node(package='cleany_robot_interface', executable='control_state_guard',
                 parameters=[{'controller_names': [controller]}], output='screen')
    broadcaster = Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'], output='screen')
    arm = Node(package='controller_manager', executable='spawner',
        arguments=[controller, '--controller-manager-timeout', '30'], output='screen')
    moveit = Path(get_package_share_directory('cleany_moveit_config')) / 'launch/physical_move_group.launch.py'
    planner = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit)), launch_arguments={
        'robot_description_file': str(folder / 'robot_control.urdf'),
        'controllers_file': str(folder / 'moveit_controllers.yaml'),
        'joint_limits_file': str(folder / 'joint_limits.yaml'),
        'include_head_camera': 'true', 'allow_trajectory_execution': 'false',
        'use_sim_time': 'false', 'use_rviz': 'false', 'enable_depth_octomap': 'false',
    }.items())

    def spawned(event, _context):
        return [EmitEvent(event=Shutdown(reason='TCP controller spawner failed'))] if event.returncode else []

    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='TCP state/control process exited'))]))
        for node in (manager, state, guard)]
    guards.extend(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=spawned))
                  for node in (broadcaster, arm))
    return [*guards, manager, state, guard, broadcaster, arm, planner]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('session'), OpaqueFunction(function=setup)])
