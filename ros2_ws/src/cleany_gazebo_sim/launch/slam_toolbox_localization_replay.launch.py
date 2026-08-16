from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    posegraph_arg = DeclareLaunchArgument(
        'posegraph',
        description='Posegraph basename to load (without .posegraph/.data).',
    )
    params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=str(
            package_share / 'config' / 'slam_toolbox_localization.yaml'
        ),
    )
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': True,
                'mode': 'localization',
                'map_file_name': LaunchConfiguration('posegraph'),
                # Every comparison bag starts at the pose used to build the map.
                'map_start_pose': [0.0, 0.0, 0.0],
            },
        ],
        output='screen',
    )
    odom_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_odom_tf_publisher',
        name='localization_replay_odom_tf_publisher',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/odom',
            'publish_odometry': False,
        }],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        )
    )
    return LaunchDescription(
        [posegraph_arg, params_arg, odom_tf, slam, configure, activate]
    )
