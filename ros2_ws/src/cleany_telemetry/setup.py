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
    install_requires=["setuptools", "aiohttp>=3.8,<4", "Pillow>=9"],
    zip_safe=True,
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    description="Read-only robot pose observations and camera previews.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["pose_observer = cleany_telemetry.pose_node:main",
                                      "camera_preview = cleany_telemetry.camera_node:main"]},
)
