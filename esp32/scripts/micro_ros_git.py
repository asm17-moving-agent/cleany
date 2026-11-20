#!/usr/bin/python3
"""공식 컴포넌트의 브랜치 기반 clone을 검토한 커밋으로 제한합니다."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
COMPONENT = ROOT / "esp32/micro_ros/micro_ros_espidf_component"


def clone_spec(args: list[str], sources: dict) -> tuple[str, str, str] | None:
    if not args or args[0] != "clone":
        return None
    # 고정된 공식 libmicroros.mk는 clone -b BRANCH URL DEST 형식을 사용합니다.
    if len(args) != 5 or args[1] != "-b":
        raise ValueError(f"unrecognized component clone invocation: {args}")
    url, destination = args[3], args[4]
    name = url.rstrip("/").rsplit("/", 1)[-1].removesuffix(".git")
    repository, revision = sources[name]  # 알 수 없는 소스는 허용하지 않습니다.
    if url.removesuffix(".git") != repository.removesuffix(".git"):
        raise ValueError(f"repository mismatch: {url}")
    return repository, revision, destination


def main() -> None:
    args = sys.argv[1:]
    working_directory = Path.cwd().resolve()
    component = COMPONENT.resolve()
    if working_directory != component and component not in working_directory.parents:
        os.execv("/usr/bin/git", ["git", *args])
    sources = json.loads((ROOT / "esp32/micro_ros.lock.json").read_text())["component_sources"]
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
