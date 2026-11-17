#!/usr/bin/env python3
"""Prepare a dependency-only Docker context and its content-based image identity."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil


def dependency_files(root: Path) -> list[Path]:
    paths = [*root.glob('ros2_ws/src/**/package.xml'),
             *root.glob('ros2_ws/rosdep/*.yaml'),
             *root.glob('containers/ci/*'),
             root / 'tools/ci/dependency_image.py',
             root / 'tools/ci/reuse-dependencies']
    return sorted(path for path in paths if path.is_file())


def file_key(root: Path, paths: list[Path], platform: str) -> str:
    digest = hashlib.sha256(f'{platform}\0'.encode())
    for path in paths:
        digest.update(path.relative_to(root).as_posix().encode() + b'\0')
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def dependency_key(root: Path, platform: str) -> str:
    """Include paths as well as bytes so package additions/removals invalidate."""
    return file_key(root, dependency_files(root), platform)


def provisioning_key(root: Path, platform: str) -> str:
    """Identify the installer, repository pins and rosdep rules of an image."""
    paths = [path for path in dependency_files(root)
             if path.relative_to(root).parts[:2] in (('containers', 'ci'), ('ros2_ws', 'rosdep'))
             and path.name != 'compatible-image.json']
    return file_key(root, paths, platform)


def compatible_image(root: Path, platform: str) -> str:
    """Select a pinned candidate; installed requirements still need validation."""
    path = root / 'containers/ci/compatible-image.json'
    if not path.is_file():
        return ''
    candidate = json.loads(path.read_text())
    if (candidate['platform'] != platform
            or candidate['provisioning_key'] != provisioning_key(root, platform)):
        return ''
    image = candidate['image']
    if not re.fullmatch(r'ghcr\.io/[a-z0-9_.-]+/[a-z0-9_.-]+@sha256:[a-f0-9]{64}', image):
        raise ValueError('Compatible dependency image must use an immutable GHCR digest')
    return image


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
    args = parser.parse_args()
    repository = args.repository.lower()
    if not re.fullmatch(r'[a-z0-9_.-]+/[a-z0-9_.-]+', repository):
        parser.error('repository must be an owner/repository name')
    root = args.root.resolve()
    key = dependency_key(root, args.platform)
    prepare_context(root, args.context)
    print(f'key={key}')
    print(f'image=ghcr.io/{repository}-ci:humble-{key}')
    print(f'compatible_image={compatible_image(root, args.platform)}')


if __name__ == '__main__':
    main()
