from glob import glob
from setuptools import setup, find_packages

setup(
    name="cleany_bringup",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cleany_bringup"]),
        ("share/cleany_bringup", ["package.xml"]),
        ("share/cleany_bringup/launch", glob("launch/*.launch.py")),
        ("share/cleany_bringup/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    license="MIT",
    description="Cleany simulation and hardware composition.",
    entry_points={
        "console_scripts": ["hardware_runtime=cleany_bringup.hardware_runtime:main"]
    },
)
