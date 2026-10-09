import json
import sys
from types import SimpleNamespace

import pytest

import parallel


def test_worker_dying_before_barrier_is_not_a_success(tmp_path, monkeypatch):
    root = tmp_path/'repo'
    root.mkdir()
    data = tmp_path/'external data'
    (data/'fixtures/cad-fortress-command-hold').mkdir(parents=True)
    setup = data/'builds/repo/install-scenario-runtime/setup.bash'
    setup.parent.mkdir(parents=True)
    setup.write_text('')
    monkeypatch.delenv('CLEANY_BUILD_ROOT', raising=False)
    identity = {'base_commit': 'test-commit', 'branch': 'trial', 'source_status': ''}
    monkeypatch.setattr(parallel, 'source_identity', lambda root: identity)
    monkeypatch.setattr(sys, 'argv', ['parallel.py', '--root', str(root), '--data-root', str(data),
                                     '--count', '1', '--name', 'failed-start'])
    monkeypatch.setattr(parallel.subprocess, 'Popen', lambda *a, **k: SimpleNamespace(
        poll=lambda: 1, returncode=1))
    with pytest.raises(RuntimeError, match='Worker failed'):
        parallel.main()
    batch = data/'runs/scenarios/failed-start/batch.json'
    evidence = json.loads(batch.read_text())
    assert json.loads((batch.parent/'source.json').read_text()) == identity
    assert evidence['exit_codes'] == [1]
    assert evidence['mission_start_wall'] is None
    assert evidence['results'] == [{'error': 'missing result'}]
    assert not (root/'artifacts').exists()
