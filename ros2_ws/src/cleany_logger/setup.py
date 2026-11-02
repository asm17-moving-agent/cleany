from glob import glob

from setuptools import find_packages, setup

name = "cleany_logger"

setup(
    name=name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{name}"]),
        (f"share/{name}", ["package.xml"]),
        (f"share/{name}/config", glob("config/*.yaml")),
        (f"share/{name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    license="MIT",
    description="Reusable rosbag2 and SQLite observation recording engine",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["logger_node = cleany_logger.node:main", "logger = cleany_logger.cli:main"]},
)
