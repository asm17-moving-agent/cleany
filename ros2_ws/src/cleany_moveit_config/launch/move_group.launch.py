from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_config():
    description_xacro = (
        Path(get_package_share_directory('cleany_description'))
        / 'urdf'
        / 'cleany.urdf.xacro'
    )
    return (
        MoveItConfigsBuilder(
            'cleany', package_name='cleany_moveit_config'
        )
        .robot_description(file_path=str(description_xacro))
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


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    allow_trajectory_execution = LaunchConfiguration(
        'allow_trajectory_execution'
    )
    moveit_config = _moveit_config()

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'allow_trajectory_execution': ParameterValue(
                    allow_trajectory_execution, value_type=bool
                ),
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
            {
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'allow_trajectory_execution', default_value='true'
            ),
            move_group,
            rviz,
        ]
    )
