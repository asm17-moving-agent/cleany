from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    default_params = package_share / 'config' / 'slam_toolbox.yaml'

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock.',
    )
    params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=str(default_params),
        description='slam_toolbox parameter file.',
    )
    loop_closing_arg = DeclareLaunchArgument(
        'do_loop_closing',
        default_value='true',
        description='Enable slam_toolbox loop closure.',
    )
    loop_search_distance_arg = DeclareLaunchArgument(
        'loop_search_maximum_distance',
        default_value='3.0',
        description='Maximum distance for loop-closure candidate scans.',
    )
    loop_search_dimension_arg = DeclareLaunchArgument(
        'loop_search_space_dimension',
        default_value='8.0',
        description='Loop-closure correlation search-window dimension.',
    )
    loop_coarse_response_arg = DeclareLaunchArgument(
        'loop_match_minimum_response_coarse',
        default_value='0.35',
        description='Minimum coarse loop-match response.',
    )
    loop_fine_response_arg = DeclareLaunchArgument(
        'loop_match_minimum_response_fine',
        default_value='0.45',
        description='Minimum fine loop-match response.',
    )
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'do_loop_closing': LaunchConfiguration('do_loop_closing'),
                'loop_search_maximum_distance': LaunchConfiguration(
                    'loop_search_maximum_distance'
                ),
                'loop_search_space_dimension': LaunchConfiguration(
                    'loop_search_space_dimension'
                ),
                'loop_match_minimum_response_coarse': LaunchConfiguration(
                    'loop_match_minimum_response_coarse'
                ),
                'loop_match_minimum_response_fine': LaunchConfiguration(
                    'loop_match_minimum_response_fine'
                ),
            },
        ],
        output='screen',
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
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            loop_closing_arg,
            loop_search_distance_arg,
            loop_search_dimension_arg,
            loop_coarse_response_arg,
            loop_fine_response_arg,
            slam,
            configure,
            activate,
        ]
    )
