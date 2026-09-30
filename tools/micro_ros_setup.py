#!/usr/bin/env python3
"""Fetch immutable micro-ROS sources and generate the ESP-IDF component."""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import venv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "motor_controller"
LOCK = CONTROLLER / "micro_ros.lock.json"
OUTPUT = CONTROLLER / "micro_ros"  # ignored bootstrap output
PIO_VENV = CONTROLLER / ".venv"


def component_fingerprint(project: Path, environment: str) -> str:
    """Include authoritative interfaces, metadata and ESP-IDF build identity."""
    digest = hashlib.sha256()
    digest.update(f"{project.resolve()}:{environment}".encode())
    files = [LOCK, CONTROLLER / "app-colcon.meta",
             CONTROLLER / "sdkconfig.microros.defaults", project / "platformio.ini"]
    interfaces = ROOT / "ros2_ws/src/cleany_base_interfaces"
    files += [interfaces / "CMakeLists.txt", interfaces / "package.xml"]
    files += sorted((interfaces / "msg").glob("*.msg"))
    for file in files:
        digest.update(str(file).encode())
        digest.update(file.read_bytes())
    return digest.hexdigest()


def refresh_component(project: Path, environment: str) -> None:
    """Regenerate derived type support when inputs or IDF build context change."""
    component = OUTPUT / "micro_ros_espidf_component"
    defaults = hashlib.sha256((CONTROLLER / "sdkconfig.microros.defaults").read_bytes()).hexdigest()
    defaults_stamp = project / ".pio/cleany-sdkconfig-defaults"
    if not defaults_stamp.exists() or defaults_stamp.read_text() != defaults:
        # sdkconfig.defaults only applies at initial generation, not to an
        # existing sdkconfig. Reset our derived config on reviewed defaults changes.
        config = project / f"sdkconfig.{environment}"
        if config.exists():
            config.unlink()
        cache = project / ".pio/build" / environment / "CMakeCache.txt"
        if cache.exists():
            cache.unlink()
        defaults_stamp.parent.mkdir(parents=True, exist_ok=True)
        defaults_stamp.write_text(defaults)
    fingerprint = component_fingerprint(project, environment)
    stamp = component / ".cleany-build-fingerprint"
    if stamp.exists() and stamp.read_text() == fingerprint:
        return
    # Upstream builds libmicroros/includes during CMake configuration. A
    # project/environment switch must rerun it even with unchanged sdkconfig.
    cache = project / ".pio/build" / environment / "CMakeCache.txt"
    if cache.exists():
        cache.unlink()
    for output in ("libmicroros.a", "include", "esp32_toolchain.cmake",
                   "micro_ros_src/build", "micro_ros_src/install", "micro_ros_src/log"):
        path = component / output
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()
    # Upstream only copies extra packages during the initial clone phase.
    source = component / "micro_ros_src/src"
    if source.exists():
        package = source / "extra_packages/cleany_base_interfaces"
        if package.exists():
            shutil.rmtree(package)
        shutil.copytree(ROOT / "ros2_ws/src/cleany_base_interfaces", package,
                        ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache"))
    stamp.write_text(fingerprint)


def run(*args: str, cwd: Path | None = None) -> None:
    subprocess.run(args, cwd=cwd, check=True)


def checkout(repository: str, revision: str, destination: Path) -> None:
    if destination.exists():
        current = subprocess.check_output(
            ["git", "-C", str(destination), "rev-parse", "HEAD"], text=True
        ).strip()
        if current == revision:
            return
        run("git", "-C", str(destination), "fetch", "--depth=1", "origin", revision)
        run("git", "-C", str(destination), "checkout", "--detach", "FETCH_HEAD")
    else:
        run("git", "clone", "--depth=1", repository, str(destination))
        run("git", "-C", str(destination), "fetch", "--depth=1", "origin", revision)
        run("git", "-C", str(destination), "checkout", "--detach", "FETCH_HEAD")
    actual = subprocess.check_output(
        ["git", "-C", str(destination), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual != revision:
        raise RuntimeError(f"{destination} resolved to {actual}, expected {revision}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="verify existing pinned checkouts")
    parser.add_argument("--agent-build", action="store_true")
    args = parser.parse_args()
    if not args.check and sys.version_info[:2] != (3, 10):
        raise RuntimeError(f"firmware setup requires Python 3.10, got {sys.version.split()[0]}")
    lock = json.loads(LOCK.read_text())
    if args.check:
        paths = {
            "component": OUTPUT / "micro_ros_espidf_component",
            "agent": OUTPUT / "agent",
            "micro_ros_msgs": OUTPUT / "agent" / "src" / "micro_ros_msgs",
        }
        for name in paths:
            spec = lock[name]
            path = paths[name]
            actual = subprocess.check_output(
                ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
            ).strip()
            if actual != spec["revision"]:
                raise SystemExit(f"{name}: expected {spec['revision']}, got {actual}")
        component = paths["component"]
        for area in ("micro_ros_dev", "micro_ros_src"):
            for path in (component / area / "src").glob("*"):
                if not (path / ".git").exists():
                    continue
                expected = lock["component_sources"][path.name][1]
                actual = subprocess.check_output(
                    ["/usr/bin/git", "-C", str(path), "rev-parse", "HEAD"], text=True
                ).strip()
                if actual != expected:
                    raise SystemExit(f"{path.name}: expected {expected}, got {actual}")
        return
    OUTPUT.mkdir(exist_ok=True)
    checkout(**lock["component"], destination=OUTPUT / "micro_ros_espidf_component")
    checkout(**lock["agent"], destination=OUTPUT / "agent")
    agent_msgs = OUTPUT / "agent" / "src" / "micro_ros_msgs"
    agent_msgs.parent.mkdir(parents=True, exist_ok=True)
    checkout(**lock["micro_ros_msgs"], destination=agent_msgs)
    git_bin = OUTPUT / "bin"
    git_bin.mkdir(exist_ok=True)
    wrapper = git_bin / "git"
    if not wrapper.exists():
        wrapper.symlink_to(ROOT / "tools/micro_ros_git.py")
    if not (PIO_VENV / "bin" / "python").exists():
        venv.EnvBuilder(with_pip=True).create(PIO_VENV)
    run(str(PIO_VENV / "bin" / "pip"), "install", "platformio==6.1.19",
        "catkin_pkg==1.1.1", "colcon-common-extensions==0.3.0",
        "colcon-core==0.21.3", "vcstool==0.3.0", "lark-parser==0.12.0",
        "empy==3.3.4")
    # The micro-ROS component's makefile copies these ROS source packages into
    # its private colcon workspace; keep the interface source authoritative.
    for project in (CONTROLLER, CONTROLLER / "microros_smoke"):
        package_dir = project / "extra_ros_packages" / "cleany_base_interfaces"
        package_dir.parent.mkdir(parents=True, exist_ok=True)
        if package_dir.is_symlink() or package_dir.exists():
            if package_dir.is_symlink() and package_dir.resolve() == (ROOT / "ros2_ws/src/cleany_base_interfaces").resolve():
                continue
            raise RuntimeError(f"unexpected existing generated package path: {package_dir}")
        package_dir.symlink_to(ROOT / "ros2_ws/src/cleany_base_interfaces", target_is_directory=True)
    for generated_link, target in (
        (CONTROLLER / "microros_smoke" / "app-colcon.meta", CONTROLLER / "app-colcon.meta"),
        (CONTROLLER / "microros_smoke" / "sdkconfig.defaults", CONTROLLER / "sdkconfig.microros.defaults"),
    ):
        if generated_link.is_symlink() and generated_link.resolve() == target.resolve():
            continue
        if generated_link.exists() or generated_link.is_symlink():
            raise RuntimeError(f"unexpected generated link path: {generated_link}")
        generated_link.symlink_to(target)
    if args.agent_build:
        agent = OUTPUT / "agent"
        run(str(PIO_VENV / "bin" / "colcon"), "build", "--packages-up-to",
            "micro_ros_agent", cwd=agent)
        return
    print(f"PlatformIO: {PIO_VENV / 'bin/platformio'} (6.1.19); run firmware targets via Make")


if __name__ == "__main__":
    main()
