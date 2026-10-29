from glob import glob
from setuptools import setup

setup(
    name='cleany_robot_interface', version='0.1.0',
    packages=['cleany_robot_interface'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cleany_robot_interface']),
        ('share/cleany_robot_interface', ['package.xml']),
        ('share/cleany_robot_interface/launch', glob('launch/*.launch.py')),
        ('share/cleany_robot_interface/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'feetech-servo-sdk==1.0.0', 'pyserial==3.5', 'PyYAML'],
    entry_points={'console_scripts': [
        'arm_state_reader = cleany_robot_interface.reader:main',
        'robot_state_reader = cleany_robot_interface.robot_state_reader:main',
        'rgbd_frame_bridge = cleany_robot_interface.rgbd_frame_bridge:main',
        'wrist_rgb_camera = cleany_robot_interface.wrist_rgb_camera:main',
        'control_state_guard = cleany_robot_interface.control_state_guard:main',
        'motor_telemetry_recorder = cleany_robot_interface.motor_telemetry:main',
    ]},
)
