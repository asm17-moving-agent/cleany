from setuptools import find_packages, setup

package_name = "cleany_mission_manager"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    description="Cleany mission lifecycle and FSM manager.",
    license="MIT",
    extras_require={"test": ["pytest"]},
)
