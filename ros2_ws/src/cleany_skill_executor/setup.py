import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cleany_skill_executor'

setup(
    name=package_name,
    version='0.1.0',
    package_data={package_name: ['data/*.json']},
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json'),
        ),
        ('share/' + package_name + '/config', ['../../../configs/robot/gripper_contact_decision.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/docs', ['docs/groot2_table_cleanup.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이정현',
    maintainer_email='sw292ljh@gmail.com',
    description='Reachable grasp selection and nearest pre-grasp execution.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest', 'trimesh==4.5.3', 'scipy>=1.10,<1.14'],
        'monitor': ['pyzmq'],
    },
    entry_points={
        'console_scripts': [
            'collection_pipeline = cleany_skill_executor.collection_pipeline:main',
            'manipulation_evidence = cleany_skill_executor.manipulation_evidence_node:main',
            'hardware_pregrasp = cleany_skill_executor.hardware_pregrasp:main',
            'pregrasp_pipeline = cleany_skill_executor.pregrasp_pipeline:main',
            'pregrasp_sim_ready = cleany_skill_executor.pregrasp_sim_ready:main',
            'physical_grasp_selection_server = cleany_skill_executor.physical_grasp_selection_node:main',
            'validate_pick_server = cleany_skill_executor.validate_pick_node:main',

            'manipulation_server = cleany_skill_executor.manipulation_node:main',
            'manipulation_monitor = cleany_skill_executor.manipulation_monitor_node:main',
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
