"""CLI state guards and policy patching without ROS or a dashboard."""
import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest

from cleany_logger.cli import (
    LoggerRejected, LoggerUnavailable, TopicEdit, edit_topic, main, topic_rows,
)
from cleany_logger.recording import defaults


def view():
    return dict(state='IDLE', session_id='session-one', revision=7, boot_id='boot-one',
                stored_bytes=0, reason='', status={}, settings=defaults())


class Client:
    def __init__(self, value=None):
        self.value = value or view()
        self.calls = []
        self.closed = False

    def call(self, operation, **kwargs):
        self.calls.append((operation, kwargs))
        if operation == 'configure':
            self.value['settings'] = copy.deepcopy(kwargs['settings'])
        elif operation == 'start':
            self.value['state'] = 'RECORDING'
        elif operation == 'stop':
            if kwargs['session_id'] != self.value['session_id']:
                raise LoggerRejected('Session changed')
            self.value['state'] = 'IDLE'
        return copy.deepcopy(self.value)

    def close(self):
        self.closed = True


def run(args, client):
    return main(args, client_factory=lambda node, timeout: client)


def test_help_imports_no_ros_or_monitor():
    root = str(Path(__file__).resolve().parents[1])
    subprocess.run([sys.executable, '-I', '-S', '-c',
        "import sys; sys.path.insert(0, sys.argv[1]); "
        "from cleany_logger.cli import parser; parser().parse_args(['status']); "
        "assert not any(n in sys.modules for n in ('rclpy','aiohttp','cleany_dev_monitor'))", root], check=True)


def test_enabling_one_source_preserves_disabled_siblings():
    settings = defaults()
    settings['enabled']['paths'] = False
    changed = edit_topic(settings, TopicEdit('path_topic', ('/custom/plan',), True, '0.5'))
    rows = {row['id']: row for row in topic_rows(changed)}
    assert rows['path_topic']['enabled'] and rows['path_topic']['topic'] == '/custom/plan'
    assert not rows['local_path_topic']['enabled']
    assert not rows['controller_evaluation_topic']['enabled']
    assert not settings['enabled']['paths'] and settings['sources'] == {}


def test_indexed_and_full_velocity_list_edits_preserve_other_settings():
    settings = defaults()
    changed = edit_topic(settings, TopicEdit('velocity_topics:1', ('/custom/safe_vel',), False, 'all'))
    assert changed['sources']['velocity_topics'] == ['/nav2/cmd_vel', '/custom/safe_vel', '/cmd_vel']
    assert changed['source_hz']['velocity_topics:1'] is None
    assert not changed['source_enabled']['velocity_topics:1']
    assert changed['enabled'] == settings['enabled']
    changed = edit_topic(changed, TopicEdit('velocity_topics', ('/first', '/second'), True, '2'))
    rows = [row for row in topic_rows(changed) if row['id'].startswith('velocity_topics:')]
    assert [row['topic'] for row in rows] == ['/first', '/second']
    assert all(row['enabled'] and row['hz'] == 2 for row in rows)


@pytest.mark.parametrize('edit', [
    TopicEdit('camera', ('relative',)), TopicEdit('camera', ('/bad//topic',)),
    TopicEdit('camera', ('/first', '/second')), TopicEdit('camera', hz='all'),
    TopicEdit('path_topic', ('/map',)), TopicEdit('path_topic', hz='nan'),
    TopicEdit('path_topic', hz='0'), TopicEdit('path_topic', hz='1001'),
    TopicEdit('not_a_role', enabled=True), TopicEdit('path_topic'),
    TopicEdit('velocity_topics:7', enabled=False), TopicEdit('path_topic:0', enabled=False),
    TopicEdit('_computed_motion', ('/fake',)),
])
def test_invalid_edits_do_not_mutate_the_original(edit):
    settings = defaults()
    original = copy.deepcopy(settings)
    with pytest.raises(ValueError):
        edit_topic(settings, edit)
    assert settings == original


def test_topic_edit_uses_current_revision_and_json(capsys):
    client = Client()
    client.value['settings']['enabled']['camera'] = False
    assert run(['topics', 'set', 'camera', '--topic', '/custom/image', '--enable', '--hz', '2', '--json'], client) == 0
    assert client.closed
    operation, kwargs = client.calls[-1]
    assert operation == 'configure' and kwargs['revision'] == 7
    assert kwargs['settings']['topics']['camera'] == '/custom/image'
    assert not kwargs['settings']['active']
    output = capsys.readouterr()
    result = json.loads(output.out)
    assert next(row for row in result['topics'] if row['id'] == 'camera')['enabled']
    assert 'disk space' in json.loads(output.err)['warning']


def test_recording_rejects_edits_without_configure(capsys):
    client = Client()
    client.value['state'] = 'RECORDING'
    assert run(['topics', 'set', 'camera', '--enable'], client) == 4
    assert [operation for operation, _ in client.calls] == ['get_config']
    assert 'Stop recording' in capsys.readouterr().err
    assert client.closed


def test_stop_pins_session_and_revision_and_handles_stale_session(capsys):
    client = Client()
    client.value['state'] = 'RECORDING'
    assert run(['stop', '--json'], client) == 0
    assert client.calls[-1] == ('stop', dict(session_id='session-one', revision=7))
    assert json.loads(capsys.readouterr().out)['state'] == 'IDLE'
    assert run(['stop', '--session', 'old-session'], Client()) == 4
    assert 'Session changed' in capsys.readouterr().err


@pytest.mark.parametrize('command', ['topics', 'topics list', 'status', 'start'])
def test_read_start_and_options_on_either_side(command, capsys):
    calls = []
    client = Client()
    def factory(node, timeout):
        calls.append((node, timeout))
        return client
    args = ['--node', '/robot/logger', *command.split(), '--timeout', '0.5', '--json']
    assert main(args, client_factory=factory) == 0
    result = json.loads(capsys.readouterr().out)
    assert calls == [('/robot/logger', .5)] and client.closed
    if command.startswith('topics'):
        assert len(result['topics']) > 5
    if command == 'start':
        assert client.calls[-1] == ('start', dict(revision=7))


def test_offline_has_nonzero_exit_and_no_success_payload(capsys):
    class Offline(Client):
        def call(self, operation, **kwargs):
            raise LoggerUnavailable('service unavailable')
    client = Offline()
    assert run(['status', '--json'], client) == 3
    output = capsys.readouterr()
    assert output.out == '' and json.loads(output.err)['exit_code'] == 3
    assert client.closed


def test_failed_finalization_is_not_a_successful_stop(capsys):
    class Unclean(Client):
        def call(self, operation, **kwargs):
            result = super().call(operation, **kwargs)
            if operation == 'stop':
                result.update(state='ERROR', reason='Bag finalization failed')
            return result
    assert run(['stop', '--json'], Unclean()) == 4
    output = capsys.readouterr()
    assert not output.out
    assert json.loads(output.err)['error'] == 'Bag finalization failed'
