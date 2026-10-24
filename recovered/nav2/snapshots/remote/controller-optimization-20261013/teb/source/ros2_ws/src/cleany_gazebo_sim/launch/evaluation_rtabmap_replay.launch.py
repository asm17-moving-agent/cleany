from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    database_arg = DeclareLaunchArgument(
        'database_path', default_value='/tmp/cleany_rtabmap.db'
    )
    odom_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_odom_tf_publisher',
        name='replay_odom_tf_publisher',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/odom',
            'publish_odometry': False,
        }],
    )
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'subscribe_scan': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'approx_sync': False,
            'database_path': LaunchConfiguration('database_path'),
            'Grid/FromDepth': 'false',
            'Grid/CellSize': '0.05',
            'Grid/RangeMax': '12.0',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'Rtabmap/DetectionRate': '0.0',
            'RGBD/LinearUpdate': '0.10',
            'RGBD/AngularUpdate': '0.10',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/ProximityBySpace': 'true',
        }],
        remappings=[('scan', '/scan'), ('odom', '/odom')],
        arguments=['-d'],
        output='screen',
    )
    return LaunchDescription([database_arg, odom_tf, rtabmap])
