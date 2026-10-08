from glob import glob
from setuptools import find_packages, setup

package_name = 'cleany_base_driver'
setup(name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + package_name + '/config',
                   glob('../../../configs/robot/base_*.yaml') + glob('config/*.perspective'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'base_driver_node=cleany_base_driver.driver_node:main',
          'mock_mcu_node=cleany_base_driver.mock_node:main',
          'odom_relay_node=cleany_base_driver.odom_relay:main']})
