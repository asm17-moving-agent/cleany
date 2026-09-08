from glob import glob

from setuptools import find_packages, setup


package_name = 'cleany_base_odometry'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cleany Team',
    maintainer_email='team@example.com',
    description='Joint-state based Mecanum wheel odometry for Cleany.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = '
            'cleany_base_odometry.wheel_odom_node:main',
        ],
    },
)
