"""Build the existing ROS stack with Make and record its source identity."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess

SOURCE = Path(__file__).resolve().parents[3]


def source_digest(source: Path) -> str:
    names = subprocess.check_output(["git", "-C", str(source), "ls-files", "-z", "--cached", "--others",
                                     "--exclude-standard", "ros2_ws/src"], text=True).split("\0")
    digest = hashlib.sha256()
    for name in sorted(filter(None, names)):
        digest.update(name.encode()+b"\0")
        digest.update((source/name).read_bytes())
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-root", type=Path, required=True)
    parser.add_argument("--ros-image", default="localhost/cleany-mission-isaac-ros:humble")
    parser.add_argument("--cleanup", action="store_true", help="Build the sensor/MoveIt manipulation stack")
    args = parser.parse_args()
    build = args.build_root.resolve()
    build.mkdir(parents=True, exist_ok=True)
    image = subprocess.check_output(["podman", "image", "inspect", "--format", "{{.Id}}", args.ros_image], text=True).strip()
    command = ["podman", "run", "--rm", "--cpus", "4", "--memory", "12g", "--memory-swap", "12g",
               "-e", "CMAKE_BUILD_PARALLEL_LEVEL=2", "-e", "MAKEFLAGS=-j2 -l2",
               "--entrypoint", "bash", "-v", f"{SOURCE}:/main:ro",
               "-v", f"{build}:/build:rw", image, "-c",
               "make -C /main build CLEANY_BUILD_ROOT=/build COLCON_BUILD_ARGS='--packages-up-to "
               "cleany_navigation cleany_mission_manager cleany_control_bridge cleany_skill_executor "
               "cleany_moveit_config "+("cleany_manipulation_bt cleany_grasping " if args.cleanup else "")+
               "--parallel-workers 2 --cmake-args -DBUILD_TESTING=ON'"]
    with (build/".build.lock").open("a") as lock, (build/"build.log").open("w") as log:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        record = {"source_commit": subprocess.check_output(["git", "-C", str(SOURCE), "rev-parse", "HEAD"], text=True).strip(),
                  "source_digest": source_digest(SOURCE), "image": image, "command": command, "status": "running"}
        navigation = json.loads((Path(__file__).parent / "navigation_source.json").read_text())
        record["navigation_source"] = {key: navigation[key] for key in ("branch", "commit", "strategy")}
        manifest = build/"manifest.json"
        manifest.write_text(json.dumps(record, indent=2)+"\n")
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        record.update(status="passed" if result.returncode == 0 else "failed", returncode=result.returncode)
        manifest.write_text(json.dumps(record, indent=2)+"\n")
        print(f"{record['status']}: {manifest}")
        if result.returncode:
            raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
