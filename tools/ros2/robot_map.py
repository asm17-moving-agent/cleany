#!/usr/bin/python3
"""Install one verified navigation map on a Linux robot, independently of its Backend."""

import argparse
import ctypes
import fcntl
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path

import yaml

ROBOT = Path("/var/opt/cleany/robot")
FILES = {"source.yaml", "map.pgm", "metadata.json"}


@contextmanager
def directory_fd(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        yield fd
    finally:
        os.close(fd)


def sync_directory(path):
    with directory_fd(path) as fd:
        os.fsync(fd)


def write_file(path, data):
    with path.open("xb") as out:
        out.write(data)
        out.flush()
        os.fsync(out.fileno())


def check_owned_directory(path):
    if path.is_symlink() or not path.is_dir():
        raise ValueError(f"Expected a map directory: {path}")
    for item in path.iterdir():
        if item.name not in FILES or item.is_symlink() or not item.is_file():
            raise ValueError(f"Unexpected map file: {item}")


def exchange(a, b):
    rename = ctypes.CDLL(None, use_errno=True).renameat2
    if rename(-100, os.fsencode(a), -100, os.fsencode(b), 2) != 0:  # RENAME_EXCHANGE
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def prepare(source, destination, map_id, revision):
    """Accept a registered bundle or an operator-selected local YAML and PGM."""
    approved = source.is_dir()
    if approved:
        meta = json.loads((source / "metadata.json").read_text())
        if map_id is not None and map_id != meta["map_id"]:
            raise ValueError("Map ID differs from the registered map")
        if meta.get("frame_id") != "map":
            raise ValueError("Expected map frame")
        map_id = meta["map_id"]
        if revision is not None and meta.get("map_revision", revision) != revision:
            raise ValueError("Revision differs from the registered map")
        revision = meta.get("map_revision", revision)
        hashes = meta["source_sha256"]
        expected_yaml = hashes.get("source.yaml", hashes.get("map.yaml"))
        # Older Backend imports keep both a byte-exact YAML and a reserialized copy.
        yml = source / "source.yaml"
        if not yml.exists():
            yml = source / "map.yaml"
    else:
        yml = source
    if not isinstance(map_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", map_id):
        raise ValueError("A valid map ID is required")
    if type(revision) is not int or revision < 1:
        raise ValueError("A positive map revision is required")
    raw = yml.read_bytes()
    config = yaml.safe_load(raw)
    image = (yml.parent / config["image"]).resolve()
    if image.suffix.lower() != ".pgm" or (approved and image != (source / "map.pgm").resolve()):
        raise ValueError("Expected the registered PGM occupancy image")
    pixels = image.read_bytes()
    if pixels[:2] not in (b"P2", b"P5") or not pixels[2:3].isspace():
        raise ValueError("Expected a PGM occupancy image")
    pgm_hash = hashlib.sha256(pixels).hexdigest()
    if approved and (pgm_hash != hashes["map.pgm"] or hashlib.sha256(raw).hexdigest() != expected_yaml):
        raise ValueError("Registered map checksum mismatch")
    if config["image"] != "map.pgm":
        if approved:
            raise ValueError("Registered YAML must reference map.pgm")
        config["image"] = "map.pgm"
        raw = yaml.safe_dump(config, sort_keys=False).encode()
    result = {"map_id": map_id, "map_revision": revision, "frame_id": "map",
              "source_sha256": {"source.yaml": hashlib.sha256(raw).hexdigest(), "map.pgm": pgm_hash}}
    write_file(destination / "source.yaml", raw)
    write_file(destination / "map.pgm", pixels)
    return result


def read_bundle(path):
    check_owned_directory(path)
    meta = json.loads((path / "metadata.json").read_text())
    for name in ("source.yaml", "map.pgm"):
        if hashlib.sha256((path / name).read_bytes()).hexdigest() != meta["source_sha256"][name]:
            raise ValueError(f"Installed map checksum mismatch: {name}")
    return meta


def nav2_probe(path):
    # A private network namespace prevents the test map from reaching the robot's DDS domain.
    result = subprocess.run([
        "/usr/bin/bwrap", "--die-with-parent", "--unshare-net", "--unshare-pid", "--ro-bind", "/", "/",
        "--dev", "/dev", "--proc", "/proc", "--tmpfs", "/tmp", "--tmpfs", "/home",
        "--ro-bind", str(path.resolve()), "/tmp/candidate", "--ro-bind",
        str(Path(__file__).with_name("nav2_map_probe.py").resolve()), "/tmp/probe.py",
        "--clearenv", "--setenv", "HOME", "/tmp", "--setenv", "PATH", "/usr/bin:/bin",
        "--setenv", "ROS_LOCALHOST_ONLY", "1", "--setenv", "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp",
        "--chdir", "/", "--", "/bin/bash", "-ec",
        "source /opt/ros/humble/setup.bash; exec /usr/bin/python3 /tmp/probe.py",
    ], check=True, stdout=subprocess.PIPE, timeout=65, text=True)
    return json.loads(result.stdout.strip().splitlines()[-1])


def verify_live(path, probe):
    meta = read_bundle(path)
    if probe(path) != meta["nav2"]:
        raise ValueError("Nav2 grid differs from the staged map")
    return meta


def install(source, parent, map_id=None, revision=None, *, probe=nav2_probe):
    parent.mkdir(parents=True, exist_ok=True)
    sync_directory(parent.parent)
    active, staged = parent / "map", parent / "map.next"
    if source.resolve().is_relative_to(staged.resolve()):
        raise ValueError("map.next is reserved for updates")
    with directory_fd(parent) as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        existing = os.path.lexists(active)
        previous = read_bundle(active) if existing else None
        if os.path.lexists(staged):
            # A crash before or after exchange always leaves the active directory intact.
            check_owned_directory(staged)
            if existing:
                verify_live(active, probe)
            shutil.rmtree(staged)
        staged.mkdir(mode=0o700)
        try:
            candidate = prepare(source, staged, map_id, revision)
            if previous is not None and (candidate["map_id"], candidate["map_revision"]) == (
                    previous["map_id"], previous["map_revision"]):
                if candidate["source_sha256"] != previous["source_sha256"]:
                    raise ValueError("Map content changed without a new revision")
                verify_live(active, probe)
                shutil.rmtree(staged)
                sync_directory(parent)
                return previous
            candidate["nav2"] = probe(staged)
            write_file(staged / "metadata.json", (json.dumps(candidate, indent=2) + "\n").encode())
            sync_directory(staged)
            sync_directory(parent)
        except Exception:
            if staged.exists():
                shutil.rmtree(staged)
            raise
        if existing:
            exchange(staged, active)
        else:
            staged.rename(active)
        try:
            sync_directory(parent)
            verify_live(active, probe)
        except BaseException:
            if existing:
                exchange(staged, active)
            else:
                active.rename(staged)
            sync_directory(parent)
            shutil.rmtree(staged)
            sync_directory(parent)
            raise
        if existing:
            shutil.rmtree(staged)
        sync_directory(parent)
        return candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="approved map directory or a local SLAM YAML")
    parser.add_argument("--map-id")
    parser.add_argument("--revision", type=int)
    parser.add_argument("--robot-root", type=Path, default=ROBOT)
    args = parser.parse_args()
    os.umask(0o077)
    try:
        print(json.dumps(install(args.source, args.robot_root, args.map_id, args.revision)))
    except BlockingIOError:
        sys.exit("Map is in use by navigation, a field mission, or another update")
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError, yaml.YAMLError) as exc:
        sys.exit(str(exc))


if __name__ == "__main__":
    main()
