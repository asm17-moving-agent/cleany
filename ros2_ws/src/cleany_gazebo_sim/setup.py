from glob import glob
from setuptools import find_packages, setup


package_name = 'cleany_gazebo_sim'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (f'share/{package_name}/config/behavior_trees', glob('config/behavior_trees/*.xml')),
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (
            f'share/{package_name}/config',
            glob('config/*.yaml'),
        ),
        (
            f'share/{package_name}/config/bridge',
            glob('config/bridge/*.yaml'),
        ),
        (
            f'share/{package_name}/config/study_cafe',
            glob('config/study_cafe/*.yaml'),
        ),
        (
            f'share/{package_name}/config/rviz',
            glob('config/rviz/*.rviz'),
        ),
        (
            f'share/{package_name}/maps',
            glob('maps/*.yaml') + glob('maps/*.pgm'),
        ),
        (f'share/{package_name}/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cleany Team',
    maintainer_email='team@example.com',
    description='Gazebo Fortress backend for Cleany mobile-base contract tests.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pan_motion_gate = cleany_gazebo_sim.pan_motion_gate_node:main',
            'voxel_guard = cleany_gazebo_sim.voxel_guard_node:main',
            'body_guard_observer = cleany_gazebo_sim.body_guard_observer:main',
            'obstacle_memory = cleany_gazebo_sim.obstacle_memory_node:main',
            'depth_clearance = cleany_gazebo_sim.depth_clearance_node:main',
            'gazebo_command_guard = cleany_gazebo_sim.command_guard:main',
            'gazebo_odom_tf_publisher = cleany_gazebo_sim.odom_tf_publisher:main',
            'gazebo_sensor_tf_publisher = cleany_gazebo_sim.sensor_tf_publisher:main',
            'simulated_encoder_node = '
            'cleany_gazebo_sim.simulated_encoder_node:main',
            'simulated_odometry_error_node = '
            'cleany_gazebo_sim.simulated_odometry_error_node:main',
            'ground_truth_route_follower = cleany_gazebo_sim.ground_truth_route_follower:main',
            'gazebo_slam_experiment = cleany_gazebo_sim.gazebo_slam_experiment:main',
            'occupancy_grid_marker = cleany_gazebo_sim.occupancy_grid_marker:main',
        ],
    },
)
