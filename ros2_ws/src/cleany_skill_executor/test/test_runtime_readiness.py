from types import SimpleNamespace as NS
import pytest
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.task import Future

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
    node = NS(cold_controllers=(), gui=False, get_node_names_and_namespaces=lambda: nodes,
        get_service_names_and_types=lambda: services,
        parameter_future=future(parameter), parameter_retry=None, controllers_future=future(controllers),
        controllers_retry=None, controllers_query_at=0.,
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


def test_guard_waits_for_first_joint_sample_before_accepting_fresh_feedback(probe):
    node, clock, *_ = probe
    node.guard = 'waiting'
    assert '/real_control/status ready' in module.ReadinessProbe.pending(node)
    clock[0] += 3.
    node.guard = 'ready'
    assert set(module.ReadinessProbe.pending(node)) == set(module.FEEDBACK)
    clock[0] += .1
    node.received = {topic: clock[0] for topic in module.FEEDBACK}
    assert module.ReadinessProbe.pending(node) == []


def test_guard_that_keeps_waiting_hits_the_status_deadline(probe):
    node, clock, *_ = probe
    node.guard = 'waiting'
    module.ReadinessProbe.pending(node)
    clock[0] += 10.
    with pytest.raises(RuntimeError, match='/real_control/status ready'):
        module.ReadinessProbe.pending(node)


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
    monkeypatch.setattr(module.rclpy, 'try_shutdown', lambda: cleanup.append('shutdown'))
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


@pytest.mark.parametrize('reply_index', [0, 1, None])
def test_delayed_or_lost_controller_state_reply_requeries_within_startup_budget(probe, reply_index):
    node, clock, _, _, _, _, controllers = probe
    node.controllers_future = None
    queries = []

    def request(_request):
        future = Future()
        queries.append(future)
        return future

    node.controllers_client = NS(service_is_ready=lambda: True, call_async=request)
    assert 'Controller state' in module.ReadinessProbe.pending(node)
    clock[0] += 4.
    assert 'Controller state' in module.ReadinessProbe.pending(node)
    assert len(queries) == 1
    clock[0] += 1.
    assert 'Controller state' in module.ReadinessProbe.pending(node)
    assert len(queries) == 2
    if reply_index is not None:
        queries[reply_index].set_result(controllers)
        assert set(module.ReadinessProbe.pending(node)) == set(module.FEEDBACK)
        clock[0] += 1.
        node.received = {name: clock[0] for name in module.FEEDBACK}
        assert module.ReadinessProbe.pending(node) == []
    else:
        clock[0] += 5.
        with pytest.raises(RuntimeError, match='query timed out: Controller state'):
            module.ReadinessProbe.pending(node)
        assert len(queries) == 2


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

def test_gpu_startup_waits_for_scene_and_current_guard(monkeypatch):
    monkeypatch.setattr(module.time, 'monotonic', lambda: 20.)
    node = NS(scene_future=None, scene_ready_at=None, scene=NS(service_is_ready=lambda: False),
              guard_state='ready', guard_received=20.)
    pending = module.ControlStartupProbe.pending
    assert pending(node) == ['/get_planning_scene']
    node.scene_future = NS(done=lambda: True,
                           result=lambda: NS(scene=NS(robot_state=NS(joint_state=NS(name=['left_gripper_joint'])))))
    node.parameter_client = NS(service_is_ready=lambda: False)
    node.parameter_future = node.parameter_retry = None
    assert pending(node) == ['MoveIt execution mode']
    disabled = NS(values=[ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)])
    node.parameter_future = NS(done=lambda: True, result=lambda: disabled)
    node.guard_received = 10.
    assert pending(node) == ['/real_control/status ready']
    node.guard_received = 20.
    assert pending(node) == []
    node.guard_state = 'fault'
    with pytest.raises(RuntimeError, match='fault'):
        pending(node)



def test_lost_moveit_reply_retries_without_accepting_an_unsafe_mode(probe):
    node, clock, _, _, _, parameter, _ = probe
    first = NS(done=lambda: False)
    node.parameter_future = first
    sent = []
    def query(request):
        sent.append(request.names)
        return NS(done=lambda: True, result=lambda: NS(values=parameter.values))
    node.parameter_client = NS(service_is_ready=lambda: True, call_async=query)
    assert not module.moveit_execution_disabled(node, clock[0])
    clock[0] += 5.
    assert module.moveit_execution_disabled(node, clock[0] - 5.)
    assert sent == [['allow_trajectory_execution']] and node.parameter_future is first
    parameter.values[0].bool_value = True
    with pytest.raises(RuntimeError, match='allow_trajectory_execution=false'):
        module.moveit_execution_disabled(node, clock[0] - 5.)


@pytest.mark.parametrize('stage', module.PLANNING_STARTUP_STAGES)
def test_planning_inputs_require_fresh_guard_and_stage_dependencies(monkeypatch, stage):
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    services = []
    validation_ready = [False]
    node = NS(guard=None, guard_received=0., guard_ready_at=None, inputs_ready_at=None,
              stage=stage, validation=NS(server_is_ready=lambda: validation_ready[0])
              if stage == 'validation' else None,
              received={topic: 9. for topic in module.FEEDBACK},
              get_service_names_and_types=lambda: services)
    pending = module.PlanningInputsStartupProbe.pending
    guard = module.PlanningInputsStartupProbe._guard
    ready = NS(data='{"state": "ready"}')
    assert pending(node) == ['/real_control/status ready']
    guard(node, ready)
    expected = set(module.FEEDBACK)
    if stage != 'scene':
        expected.add('/grasp/plan')
    if stage == 'validation':
        expected.add('/grasp/validate_pick (server)')
    assert set(pending(node)) == expected
    services.append(('/grasp/plan', []))
    validation_ready[0] = True
    clock[0] = 11.
    node.received = {topic: clock[0] for topic in module.FEEDBACK}
    assert pending(node) == ['Fresh controller feedback after planning inputs']
    clock[0] = 12.
    guard(node, ready)
    node.received['/joint_states'] = clock[0]
    assert pending(node) == []
    guard(node, NS(data='{"state": "fault"}'))
    with pytest.raises(RuntimeError, match='guard is not ready: fault'):
        pending(node)


def test_load_admission_requires_consecutive_quiet_intervals(monkeypatch):
    from cleany_skill_executor import startup_cpu
    ticks = iter([(0, 0), (100, 25), (200, 30), (300, 60), (400, 85)])
    clock = [0.]
    monkeypatch.setenv('CLEANY_COLLECTION_START_CPU_PERCENT', '80')
    monkeypatch.setattr(startup_cpu, 'cpu_ticks', lambda: next(ticks))
    monkeypatch.setattr(startup_cpu.time, 'monotonic', lambda: clock[0])
    window = startup_cpu.StartupCpuWindow()
    for now in (.5, 1., 2., 3.):
        clock[0] = now
        assert not window.ready()
    clock[0] = 4.
    assert window.ready()


def test_action_start_requires_one_server_and_stable_feedback(monkeypatch, tmp_path):
    owners = []
    actions = []
    feedback = [False]
    marker = tmp_path / 'ready'
    node = NS(ready=lambda: feedback[0], get_publishers_info_by_topic=lambda _: owners,
              stable_since=None, startup_marker=marker, startup_cpu=None)
    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node', lambda *_: actions)
    monkeypatch.setattr(module, 'StartupCpuWindow', lambda: NS(ready=lambda: True))
    pending = lambda: module.ManipulationStartupProbe.pending(node)
    assert pending() and not marker.exists()
    feedback[0] = True
    assert pending() and marker.exists() and node.startup_marker is None
    assert pending() and node.stable_since is None
    owners.append(NS(node_name='server', node_namespace='/'))
    assert pending()
    actions.append(('/manipulation/execute_skill', ['cleany_interfaces/action/ExecuteManipulationSkill']))
    feedback[0] = False
    assert pending()
    feedback[0] = True
    assert not pending()
    owners.append(owners[0])
    with pytest.raises(RuntimeError, match='Multiple'):
        pending()


def test_one_probe_reuses_participant_through_warmup(monkeypatch, capsys):
    stages = []
    node = NS(stage='control', sensor_subscriptions=[], destroy_node=lambda: None,
              close_scene_monitor=lambda: None)
    def pending():
        stages.append(node.stage)
        return []
    node.pending = pending
    advance = module.PlanningSequenceProbe.advance
    node.advance = lambda: advance(node)
    monkeypatch.setattr(module, 'PlanningSequenceProbe', lambda _cold_startup_dir: node)
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *_, **kw: clock.__setitem__(0, clock[0] + .5))
    monkeypatch.setattr(module.rclpy, 'try_shutdown', lambda: None)
    assert module.main(['--planning-sequence', '--timeout', '5']) == 0
    assert stages == ['control', 'receipt', 'scene', 'grasp', 'validation', 'warmup']
    assert capsys.readouterr().out.count('CLEANY_START_STAGE ') == 5


def test_cold_preflight_requires_inactive_controllers_and_gpu_warmup(probe):
    node, clock, *_ = probe
    node.cold_controllers = ('left_arm_controller', 'left_gripper_controller')
    pending = module.ReadinessProbe.pending
    assert 'Inactive arm/gripper controllers' in pending(node)
    reply = NS(controller=[NS(name=name, state='inactive') for name in node.cold_controllers])
    node.controllers_future = NS(done=lambda: True, result=lambda: reply)
    node.warmup_ready = False
    node.warmup_client.service_is_ready = lambda: False
    assert set(pending(node)) == {'Primary GPU IK warmup', *module.FEEDBACK}
    node.warmup_ready = True
    assert set(pending(node)) == set(module.FEEDBACK)
    clock[0] += 1.
    node.received = {name: clock[0] for name in module.FEEDBACK}
    assert pending(node) == []
    node.guard = 'fault'
    with pytest.raises(RuntimeError, match='guard is not ready: fault'):
        pending(node)


def test_staged_selector_waits_for_the_recovered_pose_and_fresh_scene(tmp_path, monkeypatch, capsys):
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    from cleany_skill_executor import sensor_scene

    measured = NS(header=NS(stamp=NS(sec=9, nanosec=990_000_000)),
                  name=sorted(module.EXPECTED_JOINTS), position=[1.] * len(module.EXPECTED_JOINTS))
    mapped = NS(name=measured.name, position=[0.] * len(measured.name))
    results = NS(done=lambda: True, result=lambda: NS(scene=NS(robot_state=NS(joint_state=mapped))))
    capture = [9_990_000_000]
    calls = []

    class SceneMonitor:
        def __init__(self, node, *, required):
            assert required

        def check(self, *, after_stamp_ns):
            calls.append(after_stamp_ns)
            if capture[0] <= after_stamp_ns:
                raise InfrastructureError('Old depth capture')

        def close(self):
            pass

    monkeypatch.setattr(sensor_scene, 'SensorSceneMonitor', SceneMonitor)
    monkeypatch.setattr(module.PlanningInputsStartupProbe, 'pending', lambda self: [])
    probe = module.PlanningSequenceProbe.__new__(module.PlanningSequenceProbe)
    probe.stage = 'validation'
    probe.cold_startup_dir = tmp_path
    probe.inputs_announced = False
    probe.started_at = module.time.monotonic()
    probe.sensor_subscriptions = []
    probe.torque_ready_ns = None
    probe.scene_monitor = None
    probe.pose_future = None
    probe.pose_requested = 0.
    probe.joint_state = measured
    probe.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_080_000_000))
    probe.pose_client = NS(service_is_ready=lambda: True, call_async=lambda request: results)

    assert probe.pending() == ['Motor torque and joint limit return']
    assert probe.scene_monitor is None
    (tmp_path / 'torque-ready').write_text('10000000000')
    assert probe.pending() == ['Joint feedback after recovery']
    measured.header.stamp = NS(sec=10, nanosec=50_000_000)
    assert probe.pending() == ['Processed depth frame and populated MoveIt map after recovery']
    capture[0] = 10_050_000_000
    assert probe.pending() == ['MoveIt joint state after recovery']
    assert probe.pending() == ['MoveIt joint state after recovery']  # Cached pre-recovery joints.
    assert 'CLEANY_START_STAGE warmup' not in capsys.readouterr().out
    mapped.position = [1.] * len(measured.name)
    assert probe.pending() == []
    assert all(value == 10_000_000_000 for value in calls)
    probe.advance()
    assert probe.stage == 'warmup'
    assert 'CLEANY_START_STAGE warmup' in capsys.readouterr().out


@pytest.mark.parametrize('failure', [None, 'before_torque', 'after_torque'])
def test_staged_cold_start_splits_torque_and_controller_activation(tmp_path, monkeypatch, failure):
    not_requested = tmp_path / 'torque-not-requested'
    not_requested.touch()
    events = []

    class TorqueProbe:
        sample_stamp_ns = 103

        def destroy_node(self):
            pass

    class GraphProbe(TorqueProbe):
        def __init__(self, *, gui, cold_controllers):
            assert cold_controllers == ['left_arm_controller']
            assert (tmp_path / 'torque-ready').read_text().strip() == '103'

        def pending(self):
            events.append('warmup-verified')
            return []

    def activate(node, controllers, deadline, *, phase, before_torque_request=None):
        events.append(phase)
        if phase == 'torque':
            assert not (tmp_path / 'torque-ready').exists()
            if failure == 'before_torque':
                raise RuntimeError('Protected feedback unavailable')
            before_torque_request()
            if failure == 'after_torque':
                raise RuntimeError('Motor failed to return')
        else:
            assert events == ['torque', 'warmup-verified', 'controllers']

    monkeypatch.setattr(module, 'ControlFeedbackStartupProbe', TorqueProbe)
    monkeypatch.setattr(module, 'ReadinessProbe', GraphProbe)
    monkeypatch.setattr(module, 'activate_cold_controllers', activate)
    monkeypatch.setattr(module, 'require_stable_feedback', lambda *args: events.append('final-feedback'))
    monkeypatch.setattr(module.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(module.rclpy, 'try_shutdown', lambda: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *args, **kwargs: None)

    result = module.main(['--cold-startup-dir', str(tmp_path), '--activate-cold-controllers',
                          'left_arm_controller'])
    assert result == (0 if failure is None else 1)
    assert not_requested.exists() == (failure == 'before_torque')
    assert events == (['torque', 'warmup-verified', 'controllers', 'final-feedback']
                      if failure is None else ['torque'])
