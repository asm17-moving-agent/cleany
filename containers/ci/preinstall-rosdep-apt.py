#!/usr/bin/env python3
"""Batch concrete APT requirements before the normal rosdep installation."""
from __future__ import annotations

import re
import shlex
import subprocess
import sys


BATCH_SIZE = 16
PACKAGE = re.compile(r'[a-z0-9][a-z0-9+.-]*(?::[a-z0-9-]+)?(?:=[A-Za-z0-9.+:~-]+)?')


def apt_packages(plan: str) -> list[str]:
    """Read plain APT commands only; leave alternatives and pip to rosdep."""
    packages = []
    for line in plan.splitlines():
        command = shlex.split(line)
        if command[:2] == ['sudo', '-H']:
            command = command[2:]
        if command[:3] != ['apt-get', 'install', '-y']:
            continue
        names = command[3:]
        # Do not interpret shell operators, options or virtual-package alternatives.
        # The unmodified rosdep pass below still handles anything not batched.
        if names and all(PACKAGE.fullmatch(name) for name in names):
            packages.extend(names)
    return list(dict.fromkeys(packages))


def main(arguments: list[str]) -> None:
    plan = subprocess.run(['rosdep', 'install', '--simulate', *arguments],
                          text=True, stdout=subprocess.PIPE, check=True)
    print(plan.stdout, end='', flush=True)
    packages = apt_packages(plan.stdout)
    # Bound each signed-index download plan by keeping dependency groups small.
    for offset in range(0, len(packages), BATCH_SIZE):
        batch = packages[offset:offset + BATCH_SIZE]
        print(f'Preinstalling {len(batch)} APT requirements in one transaction', flush=True)
        subprocess.run(['apt-get', 'install', '-y', *batch], check=True)


if __name__ == '__main__':
    main(sys.argv[1:])
