"""Check the console bootstrap in a fresh interpreter, before NumPy loads."""
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('setting,expected', [(None, '1'), ('1', '1'), ('2', '2')])
def test_budget_precedes_node_import(setting, expected):
    environment = dict(os.environ, OPENBLAS_NUM_THREADS='8')
    environment.pop('CLEANY_GRASP_BLAS_THREADS', None)
    if setting is not None:
        environment['CLEANY_GRASP_BLAS_THREADS'] = setting
    package = str(Path(__file__).resolve().parents[1])
    environment['PYTHONPATH'] = package + os.pathsep + environment.get('PYTHONPATH', '')
    code = """
import builtins
import os
import sys
from cleany_grasping import runtime
assert 'numpy' not in sys.modules
original = builtins.__import__
called = []
def guarded_import(name, *args, **kwargs):
    if name == 'cleany_grasping.grasp_node':
        assert 'numpy' not in sys.modules
        assert os.environ['OPENBLAS_NUM_THREADS'] == sys.argv[1]
        from types import SimpleNamespace
        return SimpleNamespace(main=lambda args: called.append(args))
    return original(name, *args, **kwargs)
builtins.__import__ = guarded_import
runtime.main(args=['--ros-args'])
assert called == [['--ros-args']]
"""
    subprocess.run([sys.executable, '-c', code, expected], env=environment, check=True)


@pytest.mark.parametrize('setting', ['0', '-1', '', 'no', '1.5'])
def test_invalid_budget_rejected_before_import(monkeypatch, setting):
    from cleany_grasping import runtime

    monkeypatch.setenv('CLEANY_GRASP_BLAS_THREADS', setting)
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '8')
    with pytest.raises(ValueError, match='positive integer'):
        runtime.main()
    assert os.environ['OPENBLAS_NUM_THREADS'] == '8'
