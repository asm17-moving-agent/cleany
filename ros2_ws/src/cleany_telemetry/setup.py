from setuptools import find_packages, setup

package_name = "cleany_telemetry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "websocket-client"],
    zip_safe=True,
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    description="Latest-only ROS odometry to WebSocket pose relay.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["telemetry_node = cleany_telemetry.node:main"]},
)
