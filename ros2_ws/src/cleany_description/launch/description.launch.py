from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    model_profile = LaunchConfiguration("model_profile")
    calibration_profile = LaunchConfiguration("calibration_profile")
    model = PathJoinSubstitution(
        [FindPackageShare("cleany_description"), "urdf", "cleany.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(["xacro ", model, " model_profile:=", model_profile, " calibration_profile:=", calibration_profile]),
        value_type=str,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_profile", default_value="cad_final",
                                  choices=["cad_final", "jetson_20261009"]),
            DeclareLaunchArgument(
                "calibration_profile",
                default_value="",
                choices=["", "jetson_20261009"],
                description="Optional provisional camera extrinsics; empty keeps nominal TF.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time}
                ],
                output="screen",
            ),
        ]
    )
