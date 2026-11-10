"""기존 센서, Nav2와 Mission Runtime 옆에 감독 스택을 실행합니다.

Supervisor, 독립 안전 근거 어댑터, Nav2 lifecycle 복구 소유자를 시작하며
선택적으로 팔 가용성 보고자를 추가합니다. 센서, Nav2, 제어기, 인식과
mission_runtime은 각자의 launch 파일을 유지합니다.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = get_package_share_directory("cleany_system_supervisor") + "/config/system_supervisor.yaml"
    arg = LaunchConfiguration
    sim_time = {"use_sim_time": ParameterValue(arg("use_sim_time"), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("tof_required", default_value="false"),
        DeclareLaunchArgument("supervisor_config", default_value=config),
        # 빈 값이면 로봇이 해당 하드웨어 근거를 제공하지 않으므로 운용 잠금을 유지합니다.
        DeclareLaunchArgument("estop_released_topic", default_value=""),
        DeclareLaunchArgument("protection_ready_topic", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="odom"),
        DeclareLaunchArgument("lifecycle_recovery", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("arm_health", default_value="false", choices=["true", "false"]),
        Node(package="cleany_system_supervisor", executable="system_supervisor", output="screen",
             respawn=True, respawn_delay=2.0,
             parameters=[{"config_file": arg("supervisor_config"),
                          "tof_required": ParameterValue(arg("tof_required"), value_type=bool), **sim_time}]),
        Node(package="cleany_safety_adapter", executable="safety_adapter", output="screen",
             parameters=[{"estop_released_topic": arg("estop_released_topic"),
                          "protection_ready_topic": arg("protection_ready_topic"),
                          "odom_topic": arg("odom_topic"), **sim_time}]),
        Node(package="cleany_navigation", executable="nav2_lifecycle_recovery", output="screen",
             condition=IfCondition(arg("lifecycle_recovery")), parameters=[sim_time]),
        Node(package="cleany_skill_executor", executable="arm_health_monitor", output="screen",
             condition=IfCondition(arg("arm_health")), parameters=[sim_time]),
    ])
