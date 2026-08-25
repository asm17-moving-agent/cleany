from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from gazebo_profile import (  # noqa: E402
    PROFILES,
    ProfileError,
    select_profile,
    shell_environment,
    validate_gazebo,
)


def _all_setups(_path: str) -> bool:
    return True


def test_humble_selects_fortress() -> None:
    profile = select_profile({'ROS_DISTRO': 'humble'}, _all_setups)
    assert profile == PROFILES['fortress']


def test_fortress_override_is_supported() -> None:
    profile = select_profile({'GAZEBO_PROFILE': 'fortress'}, _all_setups)
    assert profile == PROFILES['fortress']


def test_unsupported_ros_distro_fails() -> None:
    with pytest.raises(ProfileError, match='unsupported ROS_DISTRO'):
        select_profile({'ROS_DISTRO': 'unsupported'}, _all_setups)


def test_missing_ros_distro_uses_installed_humble() -> None:
    assert select_profile({}, _all_setups) == PROFILES['fortress']


def test_gazebo_major_must_match_profile() -> None:
    with pytest.raises(ProfileError, match='requires Gazebo major 6'):
        validate_gazebo(PROFILES['fortress'], lambda _profile: 7)


def test_shell_environment_uses_standard_build_outputs() -> None:
    output = shell_environment(PROFILES['fortress'])
    assert 'CLEANY_GAZEBO_PROFILE=fortress' in output
    assert 'CLEANY_BUILD_BASE=build' in output
    assert 'CLEANY_INSTALL_BASE=install' in output
