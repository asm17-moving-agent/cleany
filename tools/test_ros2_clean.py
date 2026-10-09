"""Cleaning generated outputs must preserve run evidence and other worktrees."""
import os
from pathlib import Path
import subprocess

SCRIPT = Path(__file__).resolve().parent/'ros2-clean'


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
