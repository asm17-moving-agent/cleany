import subprocess
from pathlib import Path

import pytest

from paths import batch_directory, source_identity


def test_batch_cannot_escape_output_directory(tmp_path):
    for name in ('', '.', '..', '../evidence', '/tmp/evidence', 'nested/run'):
        with pytest.raises(ValueError):
            batch_directory(tmp_path, name)
    output = batch_directory(tmp_path, 'trial-1')
    output.mkdir()
    with pytest.raises(FileExistsError):
        output.mkdir(exist_ok=False)


def test_identity_tracks_real_commit_and_uncommitted_changes(tmp_path):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(tmp_path), *args], text=True).strip()
    git('init', '-b', 'trial')
    (tmp_path/'input').write_text('original')
    git('add', 'input')
    git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-m', 'input')
    before = source_identity(tmp_path)
    assert before == {'base_commit': git('rev-parse', 'HEAD'), 'branch': 'trial', 'source_status': ''}
    (tmp_path/'input').write_text('changed')
    after = source_identity(tmp_path)
    assert after['base_commit'] == before['base_commit']
    assert 'input' in after['source_status']
