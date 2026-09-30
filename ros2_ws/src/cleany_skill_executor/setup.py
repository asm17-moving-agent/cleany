import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cleany_skill_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이정현',
    maintainer_email='sw292ljh@gmail.com',
    description='Reachable grasp selection and nearest pre-grasp execution.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'manipulation_server = cleany_skill_executor.manipulation_node:main',
            'manipulation_test_client = cleany_skill_executor.manipulation_client:main',
            'grasp_selection_server = cleany_skill_executor.grasp_selection_node:main',
            'sorting_coordinator = cleany_skill_executor.sorting_coordinator:main',
            (
                'nearest_pregrasp_coordinator = '
                'cleany_skill_executor.nearest_pregrasp_coordinator:main'
            ),
        ],
    },
)
