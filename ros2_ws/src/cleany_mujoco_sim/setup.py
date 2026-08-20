import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cleany_mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json'),
        ),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scenes'), glob('scenes/*.xml.in')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이정현',
    maintainer_email='sw292ljh@gmail.com',
    description='MuJoCo simulation node for the Cleany XLeRobot platform.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_sim_node = cleany_mujoco_sim.sim_node:main',
            (
                'handeye_scene_preflight = '
                'cleany_mujoco_sim.scene_manifest:main'
            ),
            (
                'camera_contract_adapter = '
                'cleany_mujoco_sim.camera_contract_adapter:main'
            ),
            'mujoco_rgbd_sim_node = cleany_mujoco_sim.rgbd_sim_node:main',
        ],
    },
)
