#!/usr/bin/env python3
"""전체 workspace 검사를 중복 없는 패키지 그룹으로 나누고 결과를 보존합니다."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess


def group_packages(names: list[str], group: str) -> list[str]:
    def owner(name):
        if name == 'cleany_navigation': return 'navigation'
        if name in ('cleany_manipulation_bt', 'cleany_skill_executor'): return 'manipulation'
        return 'workspace-rest'
    return [name for name in names if owner(name) == group]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('group', choices=['navigation', 'manipulation', 'workspace-rest'])
    args = parser.parse_args()
    root = Path(os.environ['CLEANY_BUILD_ROOT'])
    names = subprocess.check_output(['colcon', 'list', '--names-only'], text=True).splitlines()
    selected = group_packages(names, args.group)
    output = root/'checks'/args.group
    output.mkdir(parents=True, exist_ok=True)
    (output/'packages.json').write_text(json.dumps({'all': names, 'selected': selected}, indent=2))
    if not selected:
        raise RuntimeError(f'No packages selected for {args.group}')
    status = subprocess.run(['colcon', '--log-base', str(root/'log'/args.group), 'test',
        '--build-base', str(root/'build'), '--install-base', str(root/'install'),
        '--executor', 'sequential', '--return-code-on-test-failure', '--packages-select', *selected]).returncode
    for package in selected:
        code = subprocess.run(['colcon', 'test-result', '--test-result-base', str(root/'build'/package), '--verbose']).returncode
        status = status or code
    return status


if __name__ == '__main__':
    raise SystemExit(main())
