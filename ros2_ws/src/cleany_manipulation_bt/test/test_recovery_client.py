"""Recovery CLI initialization errors must report failure without an uncaught exception."""
from types import SimpleNamespace

import pytest
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

from cleany_manipulation_bt import recovery_client


@pytest.mark.parametrize('failure', ['init', 'node', 'client'])
def test_ros_initialization_failure_is_reported_and_cleaned_up(monkeypatch, capsys, failure):
    calls = []

    def init(**kwargs):
        calls.append('init')
        if failure == 'init':
            raise _rclpy.RCLError('domain unavailable')

    def create_client(*args):
        calls.append('client')
        raise _rclpy.RCLError('service unavailable')

    def create_node(name):
        calls.append('node')
        if failure == 'node':
            raise _rclpy.RCLError('domain unavailable')
        return SimpleNamespace(create_client=create_client,
                               destroy_node=lambda: calls.append('destroy'))

    monkeypatch.setattr(recovery_client.rclpy, 'init', init)
    monkeypatch.setattr(recovery_client.rclpy, 'create_node', create_node)
    monkeypatch.setattr(recovery_client.rclpy, 'try_shutdown', lambda: calls.append('shutdown'))
    assert recovery_client.main(['--status']) == 1
    output = capsys.readouterr()
    assert 'unavailable' in output.err and 'Traceback' not in output.err
    assert calls[-1] == 'shutdown'
    assert ('destroy' in calls) == (failure == 'client')
    if failure != 'client':
        assert 'make collection-start' in output.err
