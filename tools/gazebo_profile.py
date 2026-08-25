#!/usr/bin/env python3
"""Validate the ROS 2 Humble / Gazebo Fortress environment."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
from typing import Callable, Mapping, Sequence


class ProfileError(RuntimeError):
    """Raised when the current ROS and Gazebo environment is unsupported."""


@dataclass(frozen=True)
class GazeboProfile:
    name: str
    ros_distro: str
    ros_setup: str
    gazebo_command: tuple[str, ...]
    gazebo_major: int
    launch_file: str
    build_base: str
    install_base: str
    log_base: str
    ubuntu_version: str
    python_version: str


PROFILES = {
    'fortress': GazeboProfile(
        name='fortress',
        ros_distro='humble',
        ros_setup='/opt/ros/humble/setup.bash',
        gazebo_command=('ign', 'gazebo', '--versions'),
        gazebo_major=6,
        launch_file='gazebo_fortress.launch.py',
        build_base='build',
        install_base='install',
        log_base='log',
        ubuntu_version='22.04',
        python_version='3.10',
    ),
}


def _setup_exists(path: str) -> bool:
    return Path(path).is_file()


def select_profile(
    environment: Mapping[str, str],
    setup_exists: Callable[[str], bool] = _setup_exists,
) -> GazeboProfile:
    override = environment.get('GAZEBO_PROFILE', '').strip().lower()
    ros_distro = environment.get('ROS_DISTRO', '').strip().lower()

    if override and override != 'fortress':
        raise ProfileError(
            f'unsupported GAZEBO_PROFILE={override!r}; choose fortress'
        )

    if ros_distro and ros_distro != 'humble':
        raise ProfileError(
            f'unsupported ROS_DISTRO={ros_distro!r}; choose humble'
        )
    profile = PROFILES['fortress']

    if not setup_exists(profile.ros_setup):
        raise ProfileError(f'ROS setup file not found: {profile.ros_setup}')
    return profile


def gazebo_major(
    profile: GazeboProfile,
    run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> int:
    executable = profile.gazebo_command[0]
    if shutil.which(executable) is None:
        raise ProfileError(f'Gazebo executable not found: {executable}')
    result = run(
        profile.gazebo_command,
        check=False,
        capture_output=True,
        text=True,
    )
    output = f'{result.stdout}\n{result.stderr}'
    if result.returncode != 0:
        raise ProfileError(
            f'{shlex.join(profile.gazebo_command)} failed: {output.strip()}'
        )
    match = re.search(r'\b(\d+)\.\d+(?:\.\d+)?\b', output)
    if match is None:
        raise ProfileError(
            f'could not parse Gazebo version from: {output.strip()!r}'
        )
    return int(match.group(1))


def validate_gazebo(
    profile: GazeboProfile,
    version_reader: Callable[[GazeboProfile], int] = gazebo_major,
) -> None:
    actual_major = version_reader(profile)
    if actual_major != profile.gazebo_major:
        raise ProfileError(
            f'{profile.name} requires Gazebo major {profile.gazebo_major}; '
            f'found {actual_major}'
        )


def shell_environment(profile: GazeboProfile) -> str:
    values = {
        'CLEANY_GAZEBO_PROFILE': profile.name,
        'CLEANY_ROS_DISTRO': profile.ros_distro,
        'CLEANY_ROS_SETUP': profile.ros_setup,
        'CLEANY_GAZEBO_LAUNCH': profile.launch_file,
        'CLEANY_BUILD_BASE': profile.build_base,
        'CLEANY_INSTALL_BASE': profile.install_base,
        'CLEANY_LOG_BASE': profile.log_base,
        'CLEANY_UBUNTU_VERSION': profile.ubuntu_version,
        'CLEANY_PYTHON_VERSION': profile.python_version,
    }
    return '\n'.join(
        f'{name}={shlex.quote(value)}' for name, value in values.items()
    )


def parse_args(arguments: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--shell',
        action='store_true',
        help='print shell assignments used by the Makefile',
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] | None = None) -> int:
    options = parse_args(arguments if arguments is not None else sys.argv[1:])
    try:
        profile = select_profile(os.environ)
        validate_gazebo(profile)
    except ProfileError as error:
        print(f'Gazebo environment error: {error}', file=sys.stderr)
        return 2
    print(shell_environment(profile) if options.shell else profile.name)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
