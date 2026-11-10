#!/usr/bin/env python3
"""의존성 전용 Docker 컨텍스트와 내용 기반 이미지 식별자를 준비합니다."""

from __future__ import annotations

import argparse
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


def dependency_key(root: Path, platform: str) -> str:
    """바이트뿐 아니라 경로도 포함해 패키지 추가와 삭제가 무효화를 유발하게 합니다."""
    digest = hashlib.sha256(f'{platform}\0'.encode())
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
    args = parser.parse_args()
    repository = args.repository.lower()
    if not re.fullmatch(r'[a-z0-9_.-]+/[a-z0-9_.-]+', repository):
        parser.error('repository must be an owner/repository name')
    root = args.root.resolve()
    key = dependency_key(root, args.platform)
    prepare_context(root, args.context)
    print(f'key={key}')
    print(f'image=ghcr.io/{repository}-ci:humble-{key}')


if __name__ == '__main__':
    main()
