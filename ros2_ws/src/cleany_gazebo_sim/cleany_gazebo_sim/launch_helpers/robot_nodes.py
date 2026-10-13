"""ROS nodes shared by the Fortress and Harmonic robot launches."""
from __future__ import annotations

from pathlib import Path

from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def robot_runtime_nodes(base_config: Path) -> list[Node]:
    """Command guard, odometry chain and sensor TF, configured by launch arguments."""
    command_guard = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_command_guard',
        name='gazebo_command_guard',
        parameters=[
            base_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    odom_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_odom_tf_publisher',
        name='gazebo_odom_tf_publisher',
        parameters=[
            base_config,
            {
                'input_topic': PythonExpression(
                    [
                        "'wheel/odom' if '",
                        LaunchConfiguration('odometry_source'),
                        "' == 'wheel' else 'gazebo_odom'",
                    ]
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
    )
    wheel_odometry = Node(
        package='cleany_base_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        parameters=[
            LaunchConfiguration('wheel_odometry_config'),
            {
                'input_topic': 'wheel_encoder/joint_states',
                'output_topic': 'wheel/odom_raw',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        output='screen',
    )
    simulated_encoder = Node(
        package='cleany_gazebo_sim',
        executable='simulated_encoder_node',
        name='simulated_encoder',
        parameters=[
            LaunchConfiguration('encoder_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    simulated_odometry_error = Node(
        package='cleany_gazebo_sim',
        executable='simulated_odometry_error_node',
        name='simulated_odometry_error',
        parameters=[
            LaunchConfiguration('odometry_error_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    sensor_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_sensor_tf_publisher',
        name='gazebo_sensor_tf_publisher',
        parameters=[
            LaunchConfiguration('sensor_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    return [
        command_guard,
        odom_tf,
        simulated_encoder,
        wheel_odometry,
        simulated_odometry_error,
        sensor_tf,
    ]
