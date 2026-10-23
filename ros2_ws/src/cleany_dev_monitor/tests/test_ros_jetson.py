"""The production Jetson wrapper and monitor exchange evidence without motion Goals."""

from dataclasses import replace
import os
import socket
import time
from uuid import uuid4

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_RUN_ROS_TESTS') != '1', reason='ROS opt-in')


def test_preparation_file_is_observed_before_an_action_exists(tmp_path, monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from cleany_dev_monitor.node import MonitorNode
    from cleany_skill_executor.core.command_progress import CommandProgress

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    path = tmp_path / 'request.json'
    progress = CommandProgress(str(path), 'not-an-action-yet', os.getpid())
    progress.update('RUNTIME_PREPARING')
    progress.update('INSPECTING_SCENE')
    context = Context()
    rclpy.init(context=context)
    monitor = MonitorNode(context=context, parameter_overrides=[
        Parameter('manipulation_observation_mode', value='server'),
        Parameter('manipulation_request_file', value=str(path))])
    try:
        monitor.command_progress()
        request = monitor.hub.latest_data('manipulation_request', 'current')
        assert request['phase'] == 'INSPECTING_SCENE' and request['owner_alive']
        assert [item['phase'] for item in request['history']] == ['RUNTIME_PREPARING', 'INSPECTING_SCENE']
        assert not monitor.hub.latest_data('manipulation', 'executions')
        progress.update('COMMAND_FAILED', 'No classified target', state='FAILED')
        monitor.command_progress()
        request = monitor.hub.latest_data('manipulation_request', 'current')
        assert request['phase'] == 'COMMAND_FAILED' and request['state'] == 'FAILED'
        assert not (tmp_path / 'recordings').exists()
        published = monitor.get_publisher_names_and_types_by_node(monitor.get_name(), monitor.get_namespace())
        assert all(name in ('/rosout', '/parameter_events') for name, _ in published)
    finally:
        monitor.destroy_node()
        context.shutdown()


def test_jetson_heartbeat_reaches_monitor_without_mission_or_hardware_commands(tmp_path, monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from cleany_dev_monitor.node import MonitorNode
    from cleany_manipulation_bt.backend import WorkerBackend
    from cleany_manipulation_bt.jetson_node import JetsonManipulationNode
    from cleany_skill_executor.manipulation.models import Goal, Record, RecordState, Stage
    from std_srvs.srv import Trigger
    import json

    monkeypatch.setattr('cleany_dev_monitor.node.serve', lambda *args: None)
    context = Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    namespace = '/monitor_' + uuid4().hex[:8]
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]

    def forbidden_motion(*args):
        pytest.fail('Observation must never submit physical operations')

    server = monitor = None
    backend = WorkerBackend(forbidden_motion)
    try:
        server = JetsonManipulationNode(context=context, namespace=namespace, backend=backend,
            parameter_overrides=[Parameter('database_path', value=str(tmp_path / 'server.sqlite3')),
                                 Parameter('monitor_port', value=port)])
        monitor = MonitorNode(context=context, parameter_overrides=[
            Parameter('state_directory', value=str(tmp_path / 'monitor')),
            Parameter('manipulation_observation_mode', value='server'),
            Parameter('manipulation_events_topic', value=namespace + '/manipulation/execution_events')])
        for node in (server, monitor):
            executor.add_node(node)
        # Inject an in-memory observation only; do not accept a Goal or tick a tree.
        record = Record(Goal('m', 't', uuid4().hex, 'collect_trash', 's', 1, 'return_to_pick'),
            execution_profile='jetson', stage=Stage.GRASPING, substage='GraspObject',
            accepted_at_ns=time.time_ns(), updated_at_ns=time.time_ns())
        server.core.record = record
        state_client = monitor.create_client(Trigger, namespace + '/manipulation/control_state')
        assert state_client.wait_for_service(timeout_sec=3.)
        future = state_client.call_async(Trigger.Request())
        executor.spin_until_future_complete(future, timeout_sec=3.)
        state = json.loads(future.result().message)
        assert state['schema'] == 1 and state['active_execution_id'] == record.goal.execution_id
        assert state['backend_ready'] and not state['guard_fresh'] and not state['payload_attached']
        assert server.core.record is record and server.store.get(record.goal.execution_id) is None
        deadline = time.monotonic() + 9.
        snapshot = {}
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            snapshot = monitor.hub.latest_data('manipulation', 'executions')
            if snapshot.get('executions'):
                run = snapshot['executions'][0]
                if run['last_seen_at'] - run['received_at'] >= 1.:
                    break
        run = snapshot['executions'][0]
        assert run['record']['execution_profile'] == 'jetson'
        assert run['record']['substage'] == 'GraspObject'
        assert run['last_seen_at'] - run['received_at'] >= 1.
        assert len(run['intervals']) == 1 and run['record']['revision'] == 0
        assert server.core.record is record and server.store.get(record.goal.execution_id) is None
        published = monitor.get_publisher_names_and_types_by_node(monitor.get_name(), monitor.get_namespace())
        assert all(name in ('/rosout', '/parameter_events') for name, _ in published)
        seen = run['last_seen_at']
        server.core.record = replace(record, record_state=RecordState.INTERRUPTED)
        response = server._control_state(None, Trigger.Response())
        assert json.loads(response.message)['active_execution_id'] == ''
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert monitor.hub.latest_data('manipulation', 'executions')['executions'][0]['last_seen_at'] == seen
    finally:
        executor.shutdown()
        if monitor is not None:
            monitor.destroy_node()
        if server is not None:
            server.destroy_node()
        else:
            backend.close()
        context.shutdown()
