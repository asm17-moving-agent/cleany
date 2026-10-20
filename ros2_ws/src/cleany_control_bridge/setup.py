from setuptools import find_packages, setup

setup(
    name="cleany_control_bridge", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/cleany_control_bridge"]),
                ("share/cleany_control_bridge", ["package.xml"])],
    install_requires=["setuptools", "websocket-client"],
    tests_require=["pytest"],
    maintainer="Cleany Team", maintainer_email="team@example.com", license="MIT",
    description="Durable Backend gateway to Mission Runtime ROS adapter.",
    entry_points={"console_scripts": ["control_bridge = cleany_control_bridge.node:main"]},
)
