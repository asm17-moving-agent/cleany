from glob import glob
from setuptools import setup

setup(
    name='cleany_display',
    version='0.1.0',
    packages=['cleany_display'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cleany_display']),
        ('share/cleany_display', ['package.xml']),
        *[(f'share/cleany_display/{folder}', glob(f'{folder}/*'))
          for folder in ('config', 'web')],
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='이정현',
    maintainer_email='sw292ljh@gmail.com',
    license='Apache-2.0',
    description='Offline ROS face display',
    entry_points={'console_scripts': ['cleany_display = cleany_display.node:main']},
)
