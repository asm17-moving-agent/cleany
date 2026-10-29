from collections import Counter
from types import SimpleNamespace as NS

import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue

import cleany_skill_executor.runtime_readiness as module


@pytest.fixture
def probe(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    nodes = [('perception_inspector', '/'), ('servers', '/')]
    services = [(name, ['type']) for name in module.REQUIRED_SERVICES]
    servers = [(name, ['type']) for name in module.REQUIRED_ACTIONS]
    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node',
                        lambda node, name, ns: servers if name == 'servers' else [])
    parameter = NS(values=[ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)])
    controllers = NS(controller=[NS(name=name, state='active') for name in
        ('left_arm_controller', 'left_gripper_controller')])
    future = lambda result: NS(done=lambda: True, result=lambda: result)
    node = NS(gui=False, get_node_names_and_namespaces=lambda: nodes,
        get_service_names_and_types=lambda: services,
        parameter_future=future(parameter), controllers_future=future(controllers),
        parameter_client=NS(service_is_ready=lambda: False),
        controllers_client=NS(service_is_ready=lambda: False),
        warmup_client=NS(service_is_ready=lambda: False), warmup_future=None,
        warmup_ready=True, warmup_poll_after=0.,
        guard='ready', received={name: 9. for name in module.FEEDBACK}, feedback_after=None,
        graph_ready_at=None)
    return node, clock, nodes, services, servers, parameter, controllers


def test_waits_for_live_feedback_after_graph_and_accepts_all_streams_together(probe):
    node, clock, *_ = probe
    assert set(module.ReadinessProbe.pending(node)) == set(module.FEEDBACK)
    node.received = {name: 11. for name in module.FEEDBACK}
    clock[0] = 11.
    assert module.ReadinessProbe.pending(node) == []


@pytest.mark.parametrize('topic', module.FEEDBACK)
def test_each_missing_feedback_stream_blocks_readiness(probe, topic):
    node, clock, *_ = probe
    module.ReadinessProbe.pending(node)
    node.received = {name: 11. for name in module.FEEDBACK if name != topic}
    clock[0] = 11.
    assert module.ReadinessProbe.pending(node) == [topic]


@pytest.mark.parametrize('condition', ['service', 'no_server', 'duplicate_server', 'duplicate_node',
    'missing_inspector', 'gui', 'execution_true', 'wrong_type', 'inactive', 'guard_fault', 'guard_missing'])
def test_required_checks_are_preserved(probe, condition):
    node, _, nodes, services, servers, parameter, controllers = probe
    if condition == 'service': services.pop()
    if condition == 'no_server': servers.clear()  # An action client is insufficient.
    if condition == 'duplicate_server': servers.append(servers[0])
    if condition == 'duplicate_node': nodes.append(nodes[0])
    if condition == 'missing_inspector': nodes.pop(0)
    if condition == 'gui': node.gui = True
    if condition == 'execution_true': parameter.values[0].bool_value = True
    if condition == 'wrong_type': parameter.values[0].type = ParameterType.PARAMETER_STRING
    if condition == 'inactive': controllers.controller[0].state = 'inactive'
    if condition == 'guard_fault': node.guard = 'fault'
    if condition == 'guard_missing': node.guard = None
    fatal = condition in ('duplicate_server', 'duplicate_node', 'execution_true', 'wrong_type', 'guard_fault')
    if fatal:
        with pytest.raises(RuntimeError): module.ReadinessProbe.pending(node)
    else:
        pending = module.ReadinessProbe.pending(node)
        assert any(item not in module.FEEDBACK for item in pending)


def test_server_discovery_can_catch_up_without_restarting_process(probe):
    node, clock, _, _, servers, *_ = probe
    saved = servers[:]
    servers.clear()
    assert any('(server)' in item for item in module.ReadinessProbe.pending(node))
    servers.extend(saved)
    assert set(module.ReadinessProbe.pending(node)) == set(module.FEEDBACK)
    clock[0] += 1.
    node.received = {name: clock[0] for name in module.FEEDBACK}
    assert module.ReadinessProbe.pending(node) == []


@pytest.mark.parametrize('failed_node', ['servers', 'perception_inspector'])
def test_action_discovery_error_retries_and_requires_new_feedback(probe, monkeypatch, failed_node):
    node, clock, *_ = probe
    module.ReadinessProbe.pending(node)
    clock[0] = 11.
    node.received = {name: clock[0] for name in module.FEEDBACK}
    assert module.ReadinessProbe.pending(node) == []
    original = module.get_action_server_names_and_types_by_node
    failures = [failed_node]

    def discover(probe_node, name, namespace):
        if failures and name == failures[0]:
            failures.pop()
            raise module._rclpy.RCLError('Failed to get action server names and type: error not set')
        return original(probe_node, name, namespace)

    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node', discover)
    pending = module.ReadinessProbe.pending(node)
    assert any(f'Action server discovery for /{failed_node}:' in item for item in pending)
    assert node.graph_ready_at is None
    assert node.feedback_after is None
    clock[0] = 12.
    assert set(module.ReadinessProbe.pending(node)) == set(module.FEEDBACK)
    clock[0] = 13.
    node.received = {name: clock[0] for name in module.FEEDBACK}
    assert module.ReadinessProbe.pending(node) == []


@pytest.mark.parametrize('condition', ['duplicate_node', 'duplicate_server', 'execution_true'])
def test_action_discovery_retry_does_not_hide_invalid_configuration(probe, monkeypatch, condition):
    node, _, nodes, _, servers, parameter, _ = probe
    original = module.get_action_server_names_and_types_by_node

    def discover(probe_node, name, namespace):
        if name == 'perception_inspector':
            raise module._rclpy.RCLError('discovery unavailable')
        return original(probe_node, name, namespace)

    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node', discover)
    if condition == 'duplicate_node':
        nodes.append(nodes[0])
    elif condition == 'duplicate_server':
        servers.append(servers[0])
    else:
        parameter.values[0].bool_value = True
    with pytest.raises(RuntimeError):
        module.ReadinessProbe.pending(node)


def test_persistent_action_discovery_error_fails_at_startup_deadline(probe, monkeypatch, capsys):
    node, clock, *_ = probe
    pending = module.ReadinessProbe.pending
    cleanup = []

    def discover(*args):
        raise module._rclpy.RCLError('discovery unavailable')

    def spin_once(*args, **kwargs):
        clock[0] += .25

    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node', discover)
    monkeypatch.setattr(module, 'ReadinessProbe', lambda **kwargs: NS(
        pending=lambda: pending(node), destroy_node=lambda: cleanup.append('destroy')))
    monkeypatch.setattr(module.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: cleanup.append('shutdown'))
    assert module.main(['--timeout', '1']) == 1
    assert clock[0] == 11.
    output = capsys.readouterr().out
    assert 'Required readiness unavailable:' in output
    assert 'Action server discovery for /servers: discovery unavailable' in output
    assert '"status": "ready"' not in output
    assert cleanup == ['destroy', 'shutdown']


def test_unexpected_action_discovery_error_is_not_retried(probe, monkeypatch):
    node, *_ = probe

    def discover(*args):
        raise ValueError('invalid graph query')

    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node', discover)
    with pytest.raises(ValueError, match='invalid graph query'):
        module.ReadinessProbe.pending(node)


@pytest.mark.parametrize('topic,timeout', module.FEEDBACK_TIMEOUTS.items())
def test_missing_feedback_keeps_original_short_timeout_not_graph_timeout(probe, topic, timeout):
    node, clock, *_ = probe
    module.ReadinessProbe.pending(node)
    node.received = {name: 11. for name in module.FEEDBACK if name != topic}
    clock[0] = 10. + timeout
    with pytest.raises(RuntimeError, match=topic): module.ReadinessProbe.pending(node)


def test_configuration_query_timeout_is_bounded_after_graph_ready(probe):
    node, clock, *_ = probe
    node.parameter_future = NS(done=lambda: False)
    assert 'MoveIt execution mode' in module.ReadinessProbe.pending(node)
    clock[0] += 10.
    with pytest.raises(RuntimeError, match='query timed out'): module.ReadinessProbe.pending(node)


def test_warmup_waits_beyond_configuration_timeout_then_requires_new_feedback(probe):
    node, clock, *_ = probe
    node.warmup_ready = False
    result = NS(success=False, message='IK warmup pending')
    polls = []
    node.warmup_client = NS(service_is_ready=lambda: True,
        call_async=lambda request: polls.append(request) or NS(done=lambda: True, result=lambda: result))
    assert 'Primary GPU IK warmup' in module.ReadinessProbe.pending(node)
    clock[0] += 20.
    assert 'Primary GPU IK warmup' in module.ReadinessProbe.pending(node)
    assert node.feedback_after is None
    # Repeated readiness checks do not flood the selector with requests.
    module.ReadinessProbe.pending(node)
    assert len(polls) == 2
    result.success = True
    clock[0] += 1.
    assert set(module.ReadinessProbe.pending(node)) == set(module.FEEDBACK)
    clock[0] += .1
    node.received = {name: clock[0] for name in module.FEEDBACK}
    assert module.ReadinessProbe.pending(node) == []


def test_failed_gpu_warmup_never_reports_collection_ready(probe):
    node, *_ = probe
    node.warmup_ready = False
    node.warmup_future = NS(done=lambda: True,
        result=lambda: NS(success=False, message='IK warmup failed: GPU unavailable'))
    with pytest.raises(RuntimeError, match='GPU unavailable'):
        module.ReadinessProbe.pending(node)
