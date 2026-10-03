from pathlib import Path

from setuptools import find_packages, setup

name = "cleany_dev_monitor"
assets = [
    (f"share/{name}/web/{path.parent.relative_to('web/dist')}", [str(path)])
    for path in Path("web/dist").rglob("*")
    if path.is_file()
]
setup(
    name=name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{name}"]),
        (f"share/{name}", ["package.xml"]),
        (f"share/{name}/config", ["config/monitor.yaml"]),
        *assets,
    ],
    install_requires=["setuptools", "aiohttp>=3.8,<4", "Pillow>=9"],
    zip_safe=True,
    maintainer="Cleany Team",
    maintainer_email="team@example.com",
    license="MIT",
    description="Read-only robot developer monitor",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["dev_monitor = cleany_dev_monitor.node:main"]},
)
