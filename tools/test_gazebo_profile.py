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


@pytest.mark.parametrize(
    ('ros_distro', 'expected'),
    [('humble', 'fortress'), ('jazzy', 'harmonic')],
)
def test_ros_distro_selects_matching_profile(
    ros_distro: str,
    expected: str,
) -> None:
    profile = select_profile({'ROS_DISTRO': ros_distro}, _all_setups)
    assert profile.name == expected


def test_override_selects_profile_when_ros_is_not_sourced() -> None:
    profile = select_profile({'GAZEBO_PROFILE': 'harmonic'}, _all_setups)
    assert profile == PROFILES['harmonic']


def test_conflicting_override_and_ros_distro_fail() -> None:
    with pytest.raises(ProfileError, match='conflicts'):
        select_profile(
            {'ROS_DISTRO': 'humble', 'GAZEBO_PROFILE': 'harmonic'},
            _all_setups,
        )


def test_ambiguous_installed_distributions_fail() -> None:
    with pytest.raises(ProfileError, match='ambiguous'):
        select_profile({}, _all_setups)


def test_gazebo_major_must_match_profile() -> None:
    with pytest.raises(ProfileError, match='requires Gazebo major 8'):
        validate_gazebo(PROFILES['harmonic'], lambda _profile: 7)


def test_shell_environment_keeps_build_outputs_isolated() -> None:
    output = shell_environment(PROFILES['harmonic'])
    assert 'CLEANY_GAZEBO_PROFILE=harmonic' in output
    assert 'CLEANY_BUILD_BASE=build-harmonic' in output
    assert 'CLEANY_INSTALL_BASE=install-harmonic' in output
