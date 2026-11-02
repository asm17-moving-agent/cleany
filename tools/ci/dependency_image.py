#!/usr/bin/env python3
"""Prepare a dependency-only Docker context and its weekly image identity."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import re
import shutil


def dependency_files(root: Path) -> list[Path]:
    paths = [*root.glob('ros2_ws/src/**/package.xml'),
             *root.glob('ros2_ws/rosdep/*.yaml'),
             *root.glob('containers/ci/*'),
             root / 'tools/ci/dependency_image.py']
    return sorted(path for path in paths if path.is_file())


def dependency_key(root: Path, epoch: str, platform: str) -> str:
    """Include paths as well as bytes so package additions/removals invalidate."""
    digest = hashlib.sha256(f'{epoch}\0{platform}\0'.encode())
    for path in dependency_files(root):
        digest.update(path.relative_to(root).as_posix().encode() + b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def prepare_context(root: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=False)
    for path in dependency_files(root):
        target = destination / path.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--context', type=Path, required=True)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--platform', default='linux/amd64')
    parser.add_argument('--epoch', default=datetime.now(timezone.utc).strftime('%Gw%V'))
    args = parser.parse_args()
    repository = args.repository.lower()
    if not re.fullmatch(r'[a-z0-9_.-]+/[a-z0-9_.-]+', repository):
        parser.error('repository must be an owner/repository name')
    root = args.root.resolve()
    key = dependency_key(root, args.epoch, args.platform)
    prepare_context(root, args.context)
    print(f'key={key}')
    print(f'image=ghcr.io/{repository}-ci:humble-{key}')


if __name__ == '__main__':
    main()
