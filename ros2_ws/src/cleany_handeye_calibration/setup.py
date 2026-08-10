import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'cleany_handeye_calibration'


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
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='이정현',
    maintainer_email='sw292ljh@gmail.com',
    description='Cleany hand-eye calibration core and MoveIt adapters.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
