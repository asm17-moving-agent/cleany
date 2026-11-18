"""Native overlay paths follow Makefile defaults and explicit overrides."""
from pathlib import Path
import os
import shutil
import subprocess

import pytest

from cleany_paths import install_base


@pytest.mark.parametrize('overrides,relative', [
    ({}, '../cleany-data/builds/repository/install'),
    ({'CLEANY_DATA_ROOT': 'data'}, 'data/builds/repository/install'),
    ({'CLEANY_DATA_ROOT': 'data', 'CLEANY_BUILD_ROOT': 'build'}, 'build/install'),
    ({'CLEANY_BUILD_ROOT': 'build', 'CLEANY_INSTALL_BASE': 'overlay'}, 'overlay'),
])
def test_native_paths(tmp_path, monkeypatch, overrides, relative):
    repository = tmp_path / 'repository'
    repository.mkdir()
    for key in ('CLEANY_DATA_ROOT', 'CLEANY_BUILD_ROOT', 'CLEANY_INSTALL_BASE'):
        monkeypatch.delenv(key, raising=False)
    for key, value in overrides.items():
        monkeypatch.setenv(key, str(tmp_path / value))
    expected = ((repository / relative) if relative.startswith('..') else (tmp_path / relative)).resolve()
    actual = install_base(repository)
    assert actual == expected



def test_shell_helper_matches_python(tmp_path, monkeypatch):
    repository = Path(__file__).resolve().parents[2]
    monkeypatch.delenv('CLEANY_INSTALL_BASE', raising=False)
    monkeypatch.setenv('CLEANY_BUILD_ROOT', str(tmp_path / 'build with spaces'))
    script = repository / 'tools/ros2/cleany-install-env'
    result = subprocess.run(['bash', '-c', 'source "$1"; printf "%s" "$CLEANY_INSTALL_BASE"',
                             'paths-test', str(script)], capture_output=True, text=True, check=True)
    assert Path(result.stdout).resolve() == install_base(repository)


@pytest.mark.parametrize('legacy_link', [False, True], ids=['no-link', 'different-target'])
def test_workspace_build_without_legacy_prefix(tmp_path, monkeypatch, legacy_link):
    source = Path(__file__).resolve().parents[2]
    repo = tmp_path / 'cleany'
    (repo / 'tools/ros2').mkdir(parents=True)
    for name in ('Makefile', 'tools/ros2/cleany-install-env'):
        shutil.copyfile(source / name, repo / name)
    (repo / '.env.workspace-build').touch()
    if legacy_link:
        alias = tmp_path / 'cleany-data/builds/cleany'
        alias.parent.mkdir(parents=True)
        old = tmp_path / 'old-build'
        old.mkdir()
        alias.symlink_to(old, target_is_directory=True)

    for key in ('CLEANY_DATA_ROOT', 'CLEANY_BUILD_ROOT', 'CLEANY_INSTALL_BASE',
                'INSTALL_BASE', 'MAKEFLAGS', 'MAKEOVERRIDES'):
        monkeypatch.delenv(key, raising=False)
    expected = repo / 'ros2_ws/install'
    assert install_base(repo) == expected
    make = subprocess.check_output(['make', '-s', 'print-install-base'], cwd=repo, text=True)
    assert Path(make.strip()) == expected
    shell = subprocess.check_output(
        ['bash', '-c', 'source tools/ros2/cleany-install-env && echo "$CLEANY_INSTALL_BASE"'],
        cwd=repo, text=True)
    assert Path(shell.strip()) == expected
