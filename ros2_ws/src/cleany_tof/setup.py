from glob import glob
from setuptools import find_packages, setup

setup(name='cleany_tof', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/cleany_tof']),
                  ('share/cleany_tof', ['package.xml']),
                  ('share/cleany_tof/config', glob('config/*.yaml'))],
      install_requires=['setuptools', 'numpy', 'pyserial', 'PyYAML'],
      extras_require={'test': ['pytest']},
      entry_points={'console_scripts': [
          'tof_driver=cleany_tof.driver:main',
          'proximity_guard=cleany_tof.guard:main',
          'desk_approach=cleany_tof.approach:main']})
