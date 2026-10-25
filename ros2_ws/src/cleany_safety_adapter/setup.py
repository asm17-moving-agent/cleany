from setuptools import find_packages, setup

package_name = 'cleany_safety_adapter'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Cleany Team', maintainer_email='team@example.com',
    description='Reports independent safety-layer evidence to the System Supervisor.',
    license='MIT', extras_require={'test': ['pytest']},
    entry_points={'console_scripts': ['safety_adapter=cleany_safety_adapter.node:main']},
)
