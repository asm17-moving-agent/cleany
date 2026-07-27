from glob import glob
from setuptools import find_packages, setup


package_name = 'cleany_gazebo_sim'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
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
            'gazebo_command_guard = cleany_gazebo_sim.command_guard:main',
            'gazebo_odom_tf_publisher = cleany_gazebo_sim.odom_tf_publisher:main',
        ],
    },
)
