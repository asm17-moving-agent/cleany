"""Cleaning generated outputs must preserve run evidence and other worktrees."""
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[2]/'tools/ros2/ros2-clean'


@pytest.mark.parametrize('build_location', ['root', 'data_parent', 'data_alias'])
def test_clean_rejects_protected_ancestor_and_alias(tmp_path, build_location):
    data = tmp_path/'data'
    data.mkdir()
    sentinel = tmp_path/'build/keep.txt'
    sentinel.parent.mkdir()
    sentinel.write_text('preserve')
    alias = tmp_path/'alias'
    alias.symlink_to(data, target_is_directory=True)
    build = {'root': Path('/'), 'data_parent': tmp_path, 'data_alias': alias}[build_location]
    result = subprocess.run([str(SCRIPT), '--yes'], env={
        **os.environ, 'CLEANY_DATA_ROOT': str(data), 'CLEANY_BUILD_ROOT': str(build)},
        text=True, capture_output=True)
    assert result.returncode == 2
    assert 'Refusing unsafe build root' in result.stderr
    assert sentinel.read_text() == 'preserve'


def test_default_build_root_uses_repository_name(tmp_path):
    environment = {key: value for key, value in os.environ.items()
                   if key != 'CLEANY_BUILD_ROOT'}
    environment['CLEANY_DATA_ROOT'] = str(tmp_path)
    result = subprocess.run([str(SCRIPT)], env=environment, input='n\n',
                            text=True, capture_output=True)
    repository = Path(__file__).resolve().parents[2]
    assert result.returncode == 1
    assert str(tmp_path/'builds'/repository.name) in result.stderr


def test_clean_removes_only_selected_build_outputs(tmp_path):
    data = tmp_path/'data'
    build = data/'builds/worktree'
    generated = [build/name for name in ('build', 'install', 'log', 'build-scenario-runtime', 'install-navigation')]
    preserved = [data/'runs/result.json', data/'fixtures/world.sdf', data/'builds/other/build/cache', build/'notes.txt']
    for p in [*(d/'generated' for d in generated), *preserved]:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('preserve')
    subprocess.run([str(SCRIPT), '--yes'], env={**os.environ, 'CLEANY_DATA_ROOT': str(data), 'CLEANY_BUILD_ROOT': str(build)}, check=True)
    assert not any(d.exists() for d in generated)
    assert all(p.read_text() == 'preserve' for p in preserved)
    rejected = subprocess.run([str(SCRIPT), '--yes'], env={**os.environ, 'CLEANY_DATA_ROOT': str(data), 'CLEANY_BUILD_ROOT': str(data)}, capture_output=True)
    assert rejected.returncode != 0
    assert all(p.exists() for p in preserved)
