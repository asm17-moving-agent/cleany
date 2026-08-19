from setuptools import find_packages, setup

package_name = 'cleany_skill_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/grasp_selection.yaml']),
        ('share/' + package_name + '/launch', ['launch/grasp_selection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이정현',
    maintainer_email='sw292ljh@gmail.com',
    description='Plan-only reachable grasp selection for Cleany.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp_selection_server = cleany_skill_executor.grasp_selection_node:main',
        ],
    },
)
