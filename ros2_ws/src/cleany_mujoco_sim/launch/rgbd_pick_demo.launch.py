from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration('headless')
    rgbd_rate_hz = LaunchConfiguration('rgbd_rate_hz')
    scene_path = PathJoinSubstitution(
        [
            FindPackageShare('cleany_mujoco_sim'),
            'scenes',
            'rgbd_pick_demo.xml.in',
        ]
    )

    node = Node(
        package='cleany_mujoco_sim',
        executable='mujoco_rgbd_sim_node',
        name='mujoco_rgbd_sim',
        parameters=[
            {
                'scene_path': scene_path,
                'headless': headless,
                'scan_enabled': False,
                'initial_joint_names': ['head_tilt_joint'],
                'initial_joint_positions': [1.0],
                'rgbd_rate_hz': rgbd_rate_hz,
                'ground_truth_geom_names': [
                    'pick_box_geom',
                    'pick_can_geom',
                ],
                'ground_truth_labels': ['box', 'can'],
            }
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('headless', default_value='true'),
            DeclareLaunchArgument('rgbd_rate_hz', default_value='5.0'),
            SetEnvironmentVariable('MUJOCO_GL', 'egl'),
            node,
        ]
    )
