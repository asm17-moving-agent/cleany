from glob import glob
from setuptools import find_packages, setup

package_name = 'cleany_system_supervisor'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['config/system_supervisor.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='Cleany Team', maintainer_email='team@example.com',
    description='Capability-based system availability, mission leases and recovery coordination.',
    license='MIT', extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'system_supervisor=cleany_system_supervisor.node:main',
        'supervisor_operator=cleany_system_supervisor.operator_cli:main',
    ]},
)
