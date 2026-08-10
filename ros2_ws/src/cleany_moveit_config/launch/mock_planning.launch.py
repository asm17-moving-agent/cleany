from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _mock_moveit_config():
    return (
        MoveItConfigsBuilder(
            'cleany', package_name='cleany_moveit_config'
        )
        .robot_description(file_path='config/cleany_mock.urdf.xacro')
        .robot_description_semantic(file_path='config/cleany.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
            moveit_manage_controllers=False,
        )
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl']
        )
        .planning_scene_monitor()
        .to_moveit_configs()
    )


def _controller_spawner(controller_name: str) -> Node:
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
    )


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    moveit_config = _mock_moveit_config()
    common_parameters = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description, common_parameters],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            str(
                Path(moveit_config.package_path)
                / 'config'
                / 'mock_ros2_controllers.yaml'
            ),
            common_parameters,
        ],
        remappings=[('~/robot_description', '/robot_description')],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'allow_trajectory_execution': True,
                'publish_robot_description_semantic': True,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='log',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d',
            str(Path(moveit_config.package_path) / 'config' / 'moveit.rviz'),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            common_parameters,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            robot_state_publisher,
            ros2_control_node,
            _controller_spawner('joint_state_broadcaster'),
            _controller_spawner('left_arm_controller'),
            _controller_spawner('right_arm_controller'),
            move_group,
            rviz,
        ]
    )
