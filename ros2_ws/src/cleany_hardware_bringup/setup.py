import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'cleany_hardware_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cleany Team',
    maintainer_email='team@example.com',
    description='Jetson hardware bringup for Cleany RGB-D perception.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
