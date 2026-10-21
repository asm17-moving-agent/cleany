"""Native overlay paths follow Makefile defaults and explicit overrides."""
from pathlib import Path
import os
import subprocess

import pytest

from cleany_paths import install_base
from motion_container import Runtime, WORKSPACE


@pytest.mark.parametrize('overrides,relative', [
    ({}, '../cleany-data/builds/repository/install'),
    ({'CLEANY_DATA_ROOT': 'data'}, 'data/builds/repository/install'),
    ({'CLEANY_DATA_ROOT': 'data', 'CLEANY_BUILD_ROOT': 'build'}, 'build/install'),
    ({'CLEANY_BUILD_ROOT': 'build', 'CLEANY_INSTALL_BASE': 'overlay'}, 'overlay'),
])
def test_native_paths_and_container_translation(tmp_path, monkeypatch, overrides, relative):
    repository = tmp_path / 'repository'
    repository.mkdir()
    for key in ('CLEANY_DATA_ROOT', 'CLEANY_BUILD_ROOT', 'CLEANY_INSTALL_BASE'):
        monkeypatch.delenv(key, raising=False)
    for key, value in overrides.items():
        monkeypatch.setenv(key, str(tmp_path / value))
    expected = ((repository / relative) if relative.startswith('..') else (tmp_path / relative)).resolve()
    actual = install_base(repository)
    assert actual == expected
    runtime = Runtime(repository)
    assert runtime.translate(str(actual / 'package/share/model.xacro')) == WORKSPACE + '/install/package/share/model.xacro'
    assert runtime.translate(str(actual) + '-other/file') == str(actual) + '-other/file'


def test_shell_helper_matches_python(tmp_path, monkeypatch):
    repository = Path(__file__).resolve().parents[2]
    monkeypatch.delenv('CLEANY_INSTALL_BASE', raising=False)
    monkeypatch.setenv('CLEANY_BUILD_ROOT', str(tmp_path / 'build with spaces'))
    script = repository / 'tools/ros2/cleany-install-env'
    result = subprocess.run(['bash', '-c', 'source "$1"; printf "%s" "$CLEANY_INSTALL_BASE"',
                             'paths-test', str(script)], capture_output=True, text=True, check=True)
    assert Path(result.stdout).resolve() == install_base(repository)
