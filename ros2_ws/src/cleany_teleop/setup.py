from pathlib import Path
from glob import glob
from setuptools import find_packages, setup

assets = [
    (
        str(Path("share/cleany_teleop") / directory),
        [str(path) for path in Path(directory).glob("*") if path.is_file()],
    )
    for directory in ["ui/dist", "ui/dist/assets"]
]
setup(
    name="cleany_teleop",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/cleany_teleop"]),
        ("share/cleany_teleop", ["package.xml"]),
        ("share/cleany_teleop/launch", glob("launch/*.launch.py")),
        ("share/cleany_teleop/config", glob("config/*.yaml")),
    ]
    + assets,
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "phone_remote=cleany_teleop.web:main",
            "commission_wheels=cleany_teleop.command:main",
        ]
    },
)
