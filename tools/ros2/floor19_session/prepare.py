"""Prepare a pinned Floor 19 capture template from the deployed robot's profile."""

import argparse
import os
import shutil
import tempfile
from pathlib import Path

import yaml


def prepare(source: Path, release: Path, target: Path) -> None:
    """Copy the mapping inputs into a new template without changing the active run."""
    profile = yaml.safe_load(source.read_text())
    if not profile["lidar"]["serial_port"]:
        raise ValueError("The selected profile needs a LiDAR serial port")
    profile["scan_filter"]["half_angle_deg"] = 180
    release = release.resolve(strict=True)
    template = Path(__file__).resolve().parent
    slam = template.parents[2] / "ros2_ws/src/cleany_navigation/config/slam"
    if not target.name or target.exists() or target.is_symlink():
        raise FileExistsError(target)
    with tempfile.TemporaryDirectory(dir=target.parent) as root:
        stage = Path(root) / "template"
        shutil.copytree(
            template / "config",
            stage / "config",
            ignore=shutil.ignore_patterns("__pycache__"),
        )
        for name in ("session", "mapping.launch.py"):
            shutil.copy2(template / name, stage / name)
        for name in ("cartographer_2d.lua", "cartographer_hardware_2d.lua"):
            shutil.copy2(slam / name, stage / "config" / name)
        (stage / "config/profile.yaml").write_text(
            yaml.safe_dump(profile, sort_keys=False)
        )
        (stage / "release.txt").write_text(f"{release}\n")
        os.rename(stage, target)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "profile", type=Path, help="LiDAR and mount profile for this robot"
    )
    parser.add_argument(
        "target", type=Path, help="new template under /var/opt/cleany/runs/mapping"
    )
    parser.add_argument("--release", type=Path, default=Path("/opt/cleany/current"))
    args = parser.parse_args()
    prepare(args.profile, args.release, args.target)
