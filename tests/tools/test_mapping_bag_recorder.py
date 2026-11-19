"""Sensor-only selection, evidence preservation and graceful native shutdown."""
import importlib.util
from pathlib import Path
import signal
import subprocess
from unittest.mock import Mock

import pytest

SOURCE = Path(__file__).resolve().parents[2] / 'tools/validation/record_mapping_bag.py'
spec = importlib.util.spec_from_file_location('record_mapping_bag', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_native_recording_selects_inputs_and_preserves_existing_bag(tmp_path):
    output = tmp_path / 'bag'
    args = module.command(output)
    assert args[:3] == ['ros2', 'bag', 'record']
    assert args[args.index('--storage-preset-profile') + 1] == 'resilient'
    topics = {value for value in args if value.startswith('/') and value != str(output)}
    assert topics == {'/scan_raw', '/odom', '/tf', '/tf_static',
                      '/base/wheel_state', '/joint_states'}
    assert '/cmd_vel' not in args and '/base/wheel_command' not in args
    assert '--all' not in args and '--use-sim-time' not in args
    output.mkdir()
    evidence = output / 'old.db3'
    evidence.write_bytes(b'original')
    with pytest.raises(ValueError, match='overwrite'):
        module.command(output)
    assert evidence.read_bytes() == b'original'


@pytest.mark.parametrize('reason', [subprocess.TimeoutExpired('record', 1), KeyboardInterrupt()])
def test_timeout_and_interrupt_flush_only_owned_recorder(tmp_path, monkeypatch, reason):
    child = Mock(returncode=0)
    child.wait.side_effect = [reason, 0]
    popen = Mock(return_value=child)
    monkeypatch.setattr(module.subprocess, 'Popen', popen)
    assert module.record(tmp_path / 'bag', 1.) == 0
    child.send_signal.assert_called_once_with(signal.SIGINT)
    assert child.wait.call_args_list[-1].kwargs == {'timeout': 30}
    assert popen.call_count == 1
    assert popen.call_args.kwargs == {'stdin': subprocess.DEVNULL, 'start_new_session': True}


def test_native_failure_is_not_reported_as_success(tmp_path, monkeypatch):
    child = Mock(returncode=1)
    monkeypatch.setattr(module.subprocess, 'Popen', Mock(return_value=child))
    assert module.record(tmp_path / 'bag', 1.) == 1
    child.send_signal.assert_not_called()
