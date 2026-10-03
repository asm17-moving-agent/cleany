from glob import glob
from setuptools import find_packages, setup

package_name = "cleany_mission_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "py_trees>=2.2,<3", "PyYAML"],
    zip_safe=True,
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    description="Cleany mission lifecycle and FSM manager.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mission_runtime = cleany_mission_manager.node:main",
    ]},
)
