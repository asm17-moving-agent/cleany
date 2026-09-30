from glob import glob
from setuptools import setup

setup(name="cleany_bringup", version="0.1.0", packages=[],
      data_files=[("share/ament_index/resource_index/packages", ["resource/cleany_bringup"]),
                  ("share/cleany_bringup", ["package.xml"]),
                  ("share/cleany_bringup/launch", glob("launch/*.launch.py"))],
      install_requires=["setuptools"], maintainer="Cleany Team",
      maintainer_email="team@example.com", license="MIT",
      description="Simulation composition for the Cleany mission runtime.")
