#!/usr/bin/env python3
"""Check CI dependencies against the observed Jetson Python and ROS versions."""

from __future__ import annotations

import importlib.metadata
import json
from pathlib import Path
import subprocess


def version_errors(
    expected_pip: dict[str, str],
    actual_pip: dict[str, str],
    expected_ros: dict[str, str],
    actual_ros: dict[str, str],
) -> list[str]:
    """Compare Python pins and shared ROS upstream versions across architectures."""
    errors = []
    for name, expected in expected_pip.items():
        actual = actual_pip.get(name)
        if actual != expected:
            errors.append(f'{name}: expected {expected}, installed {actual or "missing"}')
    for name, actual in actual_ros.items():
        if name in expected_ros and actual.rsplit('-', 1)[0] != expected_ros[name]:
            errors.append(f'{name}: expected {expected_ros[name]}, installed {actual}')
    return errors


def main() -> None:
    root = Path(__file__).resolve().parent
    expected_pip = dict(
        line.split('==', 1)
        for line in (root / 'pip-constraints.txt').read_text().splitlines()
        if line and not line.startswith('#')
    )
    actual_pip = {}
    for name in expected_pip:
        try:
            actual_pip[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    expected_ros = json.loads((root / 'jetson-ros-versions.json').read_text())['packages']
    installed = subprocess.check_output(['dpkg-query', '-W'], text=True)
    actual_ros = dict(
        line.split('\t', 1) for line in installed.splitlines()
        if line.startswith('ros-humble-') and '\t' in line
    )
    errors = version_errors(expected_pip, actual_pip, expected_ros, actual_ros)
    if errors:
        raise SystemExit('\n'.join(errors))
    shared = len(expected_ros.keys() & actual_ros.keys())
    print(f'Jetson version check: {len(expected_pip)} Python, {shared} shared ROS packages matched')


if __name__ == '__main__':
    main()
