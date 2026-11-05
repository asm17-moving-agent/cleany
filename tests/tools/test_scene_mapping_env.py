"""Exercise clean builds and mandatory runtime overlays in isolated Bash shells."""
from __future__ import annotations

import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'tools/scene-mapping-env'


@pytest.fixture
def overlay_paths(tmp_path):
    paths = {
        'CLEANY_SCENE_RUNTIME_PREFIX': tmp_path / 'scene runtime',
        'CLEANY_MOVEIT_PATCH_PREFIX': tmp_path / 'moveit patch',
        'CLEANY_SCENE_DEPENDENCY_PREFIX': tmp_path / 'scene dependencies',
    }
    for path in paths.values():
        path.mkdir()
    return paths


def source_environment(paths, optional=True):
    environment = dict(os.environ, **{key: str(path) for key, path in paths.items()},
                       LD_LIBRARY_PATH='/system/libraries')
    environment.pop('CLEANY_TEST_OVERLAY_TRACE', None)
    return subprocess.run([
        'bash', '--noprofile', '--norc', '-c',
        'source "$1" "$2" && printf "%s\\n%s\\n" '
        '"${CLEANY_TEST_OVERLAY_TRACE:-system}" "$LD_LIBRARY_PATH"',
        'overlay-probe', str(SCRIPT), '--optional' if optional else '',
    ], env=environment, text=True, capture_output=True, check=False)


def install_fake_overlays(paths):
    (paths['CLEANY_SCENE_RUNTIME_PREFIX'] / 'local_setup.bash').write_text(
        'export CLEANY_TEST_OVERLAY_TRACE=scene\n')
    (paths['CLEANY_SCENE_RUNTIME_PREFIX'] / 'scene-runtime.receipt').write_text('test\n')
    (paths['CLEANY_MOVEIT_PATCH_PREFIX'] / 'local_setup.bash').write_text(
        'export CLEANY_TEST_OVERLAY_TRACE="${CLEANY_TEST_OVERLAY_TRACE}/patch"\n')
    (paths['CLEANY_MOVEIT_PATCH_PREFIX'] / 'octomap-read-lock.receipt').write_text('test\n')


def test_clean_build_preserves_system_library_environment(overlay_paths):
    result = source_environment(overlay_paths)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ['system', '/system/libraries']


def test_runtime_rejects_missing_overlays(overlay_paths):
    result = source_environment(overlay_paths, optional=False)
    assert result.returncode != 0
    assert 'Matching scene runtime is missing' in result.stderr


@pytest.mark.parametrize('prefix,filename', [
    ('CLEANY_SCENE_RUNTIME_PREFIX', 'local_setup.bash'),
    ('CLEANY_SCENE_RUNTIME_PREFIX', 'scene-runtime.receipt'),
    ('CLEANY_MOVEIT_PATCH_PREFIX', 'local_setup.bash'),
    ('CLEANY_MOVEIT_PATCH_PREFIX', 'octomap-read-lock.receipt'),
])
def test_partial_overlay_does_not_silently_use_system_packages(overlay_paths, prefix, filename):
    (overlay_paths[prefix] / filename).write_text('true\n')
    result = source_environment(overlay_paths)
    assert result.returncode != 0
    assert not result.stdout


@pytest.mark.parametrize('optional', [False, True])
def test_complete_overlays_load_in_order(overlay_paths, optional):
    install_fake_overlays(overlay_paths)
    result = source_environment(overlay_paths, optional)
    assert result.returncode == 0, result.stderr
    trace, libraries = result.stdout.splitlines()
    assert trace == 'scene/patch'
    assert libraries.startswith(str(overlay_paths['CLEANY_SCENE_DEPENDENCY_PREFIX']) + '/')
    assert libraries.endswith(':/system/libraries')


@pytest.mark.parametrize('prefix,filename', [
    ('CLEANY_SCENE_RUNTIME_PREFIX', 'local_setup.bash'),
    ('CLEANY_SCENE_RUNTIME_PREFIX', 'scene-runtime.receipt'),
    ('CLEANY_MOVEIT_PATCH_PREFIX', 'local_setup.bash'),
    ('CLEANY_MOVEIT_PATCH_PREFIX', 'octomap-read-lock.receipt'),
])
def test_missing_artifact_rejects_an_otherwise_complete_overlay(overlay_paths, prefix, filename):
    install_fake_overlays(overlay_paths)
    (overlay_paths[prefix] / filename).unlink()
    result = source_environment(overlay_paths)
    assert result.returncode != 0
    assert not result.stdout


@pytest.mark.parametrize('prefix', ['CLEANY_SCENE_RUNTIME_PREFIX', 'CLEANY_MOVEIT_PATCH_PREFIX'])
def test_failed_overlay_setup_stops_the_caller(overlay_paths, prefix):
    install_fake_overlays(overlay_paths)
    (overlay_paths[prefix] / 'local_setup.bash').write_text('return 7\n')
    result = source_environment(overlay_paths)
    assert result.returncode != 0
    assert not result.stdout


def test_make_build_reaches_colcon_without_local_overlays(overlay_paths, tmp_path):
    setup = tmp_path / 'ros setup.bash'
    setup.write_text('true\n')
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    colcon = binaries / 'colcon'
    colcon.write_text('#!/bin/bash\nprintf "COLCON %s\\n" "$*"\n')
    colcon.chmod(0o755)
    environment = dict(os.environ, **{key: str(path) for key, path in overlay_paths.items()},
                       PATH=str(binaries) + ':' + os.environ['PATH'])
    result = subprocess.run([
        'make', '--no-print-directory', 'build', f'ROS_SETUP={setup}',
        'COLCON_BUILD_ARGS=--packages-select cleany_mission_manager',
    ], cwd=ROOT, env=environment, text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert 'build --symlink-install' in result.stdout
    assert '--packages-select cleany_mission_manager' in result.stdout
    assert '--build-base' in result.stdout and '--install-base' in result.stdout


@pytest.mark.parametrize('override', [None, '', '/tmp/custom moveit cache'])
def test_geometry_cache_default_and_explicit_disable(overlay_paths, tmp_path, override):
    install_fake_overlays(overlay_paths)
    environment = dict(os.environ, **{key: str(path) for key, path in overlay_paths.items()},
                       XDG_CACHE_HOME=str(tmp_path / 'cache root'))
    environment.pop('CLEANY_MOVEIT_GEOMETRY_CACHE_DIR', None)
    if override is not None:
        environment['CLEANY_MOVEIT_GEOMETRY_CACHE_DIR'] = override
    result = subprocess.run([
        'bash', '--noprofile', '--norc', '-c',
        'source "$1" && printf "%s" "$CLEANY_MOVEIT_GEOMETRY_CACHE_DIR"',
        'cache-probe', str(SCRIPT)], env=environment, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    expected = str(tmp_path / 'cache root/cleany/moveit-geometry') if override is None else override
    assert result.stdout == expected
