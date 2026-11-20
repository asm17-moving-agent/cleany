"""Verify that native workspace tools resolve the same build and install paths."""
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from cleany_paths import install_base


@pytest.fixture
def repository(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[2]
    repo = tmp_path / 'repository'
    (repo / 'tools/ros2').mkdir(parents=True)
    for name in ('Makefile', 'tools/ros2/cleany-install-env', 'tools/ros2/ros2-clean'):
        shutil.copyfile(source / name, repo / name)
    for name in ('CLEANY_DATA_ROOT', 'CLEANY_BUILD_ROOT', 'CLEANY_INSTALL_BASE',
                 'INSTALL_BASE', 'MAKEFLAGS', 'MAKEOVERRIDES', 'XDG_CACHE_HOME'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv('HOME', str(tmp_path / 'home'))
    return repo


@pytest.mark.parametrize(('marker', 'overrides', 'relative'), [
    (False, {}, 'home/.cache/cleany/builds/repository'),
    (False, {'XDG_CACHE_HOME': 'cache with spaces', 'CLEANY_DATA_ROOT': 'data'},
     'cache with spaces/cleany/builds/repository'),
    (True, {'XDG_CACHE_HOME': 'cache'}, 'repository/ros2_ws'),
    (True, {'CLEANY_BUILD_ROOT': 'explicit build'}, 'explicit build'),
], ids=['home-cache', 'xdg-cache', 'workspace-marker', 'explicit-build'])
def test_native_paths(repository, tmp_path, monkeypatch, marker, overrides, relative):
    if marker:
        (repository / '.env.workspace-build').touch()
        legacy = tmp_path / 'cleany-data/builds/repository'
        legacy.parent.mkdir(parents=True)
        old_build = tmp_path / 'old-build'
        old_build.mkdir()
        legacy.symlink_to(old_build, target_is_directory=True)
    for name, value in overrides.items():
        monkeypatch.setenv(name, str(tmp_path / value))

    build = (tmp_path / relative).resolve()
    expected = build / 'install'
    assert install_base(repository) == expected

    make = subprocess.check_output(['make', '-s', 'print-install-base'], cwd=repository, text=True)
    assert Path(make.strip()).resolve() == expected
    shell = subprocess.check_output(
        ['bash', '-c', 'source tools/ros2/cleany-install-env && printf "%s\\n" "$CLEANY_INSTALL_BASE"'],
        cwd=repository, text=True)
    assert Path(shell.strip()).resolve() == expected

    if sys.platform == 'linux':
        clean = subprocess.run(
            ['bash', 'tools/ros2/ros2-clean'], input='n\n', cwd=repository, text=True,
            capture_output=True, check=False)
        assert clean.returncode == 1
        assert f'Remove ROS 2 generated build outputs under {build}?' in clean.stderr
        assert 'aborted' in clean.stderr


def test_explicit_install_base(repository, tmp_path, monkeypatch):
    (repository / '.env.workspace-build').touch()
    build = tmp_path / 'build with spaces'
    expected = tmp_path / 'overlay'
    monkeypatch.setenv('CLEANY_BUILD_ROOT', str(build))
    monkeypatch.setenv('CLEANY_INSTALL_BASE', str(expected))
    assert install_base(repository) == expected.resolve()

    shell = subprocess.check_output(
        ['bash', '-c', 'source tools/ros2/cleany-install-env && printf "%s\\n" "$CLEANY_INSTALL_BASE"'],
        cwd=repository, text=True)
    assert Path(shell.strip()).resolve() == expected.resolve()


@pytest.mark.skipif(sys.platform != 'linux', reason='ros2-clean uses GNU realpath')
def test_clean_refuses_build_root_containing_checkout(repository, monkeypatch):
    monkeypatch.setenv('CLEANY_BUILD_ROOT', str(repository.parent))
    clean = subprocess.run(
        ['bash', 'tools/ros2/ros2-clean', '--yes'], cwd=repository, text=True,
        capture_output=True, check=False)
    assert clean.returncode == 2
    assert 'Refusing unsafe build root:' in clean.stderr
