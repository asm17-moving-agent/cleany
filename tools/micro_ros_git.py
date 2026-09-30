#!/usr/bin/python3
"""Restrict the official component's branch-based clones to reviewed commits."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "motor_controller/micro_ros/micro_ros_espidf_component"


def clone_spec(args: list[str], sources: dict) -> tuple[str, str, str] | None:
    if not args or args[0] != "clone":
        return None
    # The pinned official libmicroros.mk uses exactly clone -b BRANCH URL DEST.
    if len(args) != 5 or args[1] != "-b":
        raise ValueError(f"unrecognized component clone invocation: {args}")
    url, destination = args[3], args[4]
    name = url.rstrip("/").rsplit("/", 1)[-1].removesuffix(".git")
    repository, revision = sources[name]  # unknown sources fail closed
    if url.removesuffix(".git") != repository.removesuffix(".git"):
        raise ValueError(f"repository mismatch: {url}")
    return repository, revision, destination


def main() -> None:
    args = sys.argv[1:]
    if COMPONENT.resolve() not in Path.cwd().resolve().parents:
        os.execv("/usr/bin/git", ["git", *args])
    sources = json.loads((ROOT / "motor_controller/micro_ros.lock.json").read_text())["component_sources"]
    spec = clone_spec(args, sources)
    if spec is None:
        os.execv("/usr/bin/git", ["git", *args])
    repository, revision, destination = spec
    subprocess.run(["/usr/bin/git", "clone", "--no-checkout", "--depth=1",
                    repository, destination], check=True)
    subprocess.run(["/usr/bin/git", "-C", destination, "fetch", "--depth=1",
                    "origin", revision], check=True)
    subprocess.run(["/usr/bin/git", "-C", destination, "checkout", "--detach",
                    revision], check=True)


if __name__ == "__main__":
    main()
