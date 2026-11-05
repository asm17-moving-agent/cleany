from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from cleany_skill_executor.motion_runtime import motion_launch_arguments, motion_runtime_prefix, selector_exit_handler


def _setup(context):
    backend = LaunchConfiguration('motion_backend').perform(context)
    config = Path(get_package_share_directory('cleany_skill_executor')) / 'config'
    parameters = [str(config / 'physical_grasp_selection.yaml')]
    if backend == 'curobo_ik_moveit':
        parameters.append(str(config / 'hybrid_motion.yaml'))
    extra = LaunchConfiguration('selector_config_file').perform(context)
    if extra:
        parameters.append(extra)
    selector = Node(
            package='cleany_skill_executor',
            executable='physical_grasp_selection_server',
            prefix=motion_runtime_prefix(),
            parameters=[*parameters, {'motion_backend': backend}],
            output='screen',
        )
    return [selector_exit_handler(selector), selector]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('repository_root', default_value=str(Path(__file__).resolve().parents[4])),
        *motion_launch_arguments(),
        DeclareLaunchArgument('selector_config_file', default_value=''),
        DeclareLaunchArgument('motion_backend', default_value='moveit',
                              choices=['moveit','curobo','curobo_ik_moveit']),
        OpaqueFunction(function=_setup),
    ])
