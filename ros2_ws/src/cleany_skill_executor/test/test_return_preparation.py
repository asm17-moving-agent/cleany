from dataclasses import replace
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory, stopped_recorded_window
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
from rclpy.task import Future
from cleany_skill_executor.core.cartesian import CartesianPose
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.hardware_execution import HardwareExecution, ReturnEndpoint
from cleany_skill_executor.readonly_services import ReadOnlyBatch
from test_arrival_approach import command_pipeline

@pytest.fixture
def preparation(monkeypatch):
    from cleany_skill_executor import hardware_execution as module
    clock, pending, removed, events = [10.], [], [], []
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module, 'physical_limits', lambda *_: {'limit': 'fresh'})
    xml = '<robot name="same"/>'
    responses = [
        NS(component=[NS(class_type='cleany_feetech_hardware/FeetechSystem', state=NS(label='active'))]),
        NS(controller=[NS(name='right_'+part+'_controller', state='active') for part in ('arm', 'gripper')]),
        NS(values=[NS(string_value=xml)]),
        NS(values=[NS(string_value=xml), NS(bool_value=False)]),
    ]
    clients = []
    for index in range(4):
        def send(request, index=index):
            future = Future()
            pending.append((index, future))
            return future
        clients.append(NS(srv_name=f'/read_{index}', service_is_ready=lambda: True,
            wait_for_service=lambda **_: True, call_async=send,
            remove_pending_request=lambda f: removed.append(f)))
    def wait(future, _):
        for index, f in pending:
            if not f.done() and not f.cancelled(): f.set_result(deepcopy(responses[index]))
        return future.result()
    state = NS(name=[*ARM_JOINT_NAMES['right'], 'left_gripper_joint'], position=[0.]*6)
    real = object.__new__(HardwareExecution)
    real.node = NS(wait=wait, check_execution=lambda: None,
        event=lambda name, **fields: events.append((name, fields)),
        get_parameter=lambda _: NS(value=False),
        get_logger=lambda: NS(info=lambda _: None, warning=lambda _: None),
        motion=NS(set_robot_description=lambda _: None))
    real.hardware, real.controllers, real.parameters, real.moveit_parameters = clients
    real.share_return_preparation = real.parallel_model_queries = True
    real.monitor = lambda: deepcopy(state)
    real.motion_submissions, real.communication_recovery_count = 1, 0
    real._verified_models, real._physical_limits = (xml, xml), {'right': {'limit': 'fresh'}}
    real.value = {'hardware_guard_max_age_sec': .25, 'hardware_tcp_position_tolerance_m': .03,
                  'hardware_tcp_orientation_tolerance_deg': 20.}.__getitem__
    return NS(real=real, clock=clock, pending=pending, removed=removed,
              events=events, state=state, responses=responses, clients=clients)


def save_endpoint(fixture):
    real = fixture.real
    pose = CartesianPose((0., 0., .5), (0., 0., 0., 1.))
    receipt = ReturnEndpoint(pose, pose, tuple(sorted(zip(fixture.state.name, fixture.state.position))),
        real._verified_models, real.motion_submissions, real.communication_recovery_count, fixture.clock[0])
    real._return_preparation.endpoint = receipt
    return receipt


def test_return_preparation_overlaps_fresh_home_reads_and_consumes_once(preparation):
    f, real = preparation, preparation.real
    descriptions = []
    real.node.motion.set_robot_description = descriptions.append
    real._verified_models = None
    with real.return_preparation('right'):
        real.prepare_return_phase('right')
        assert len(f.pending) == 4 and all(not p.done() for _, p in f.pending)
        real.prepare_return_phase('right')
        f.clock[0] += .1  # Other preparation can proceed before response waits.
        assert real.verify('right') == {'limit': 'fresh'}
        assert len(f.pending) == 4
        assert sum(fields.get('outcome') == 'consumed' for _, fields in f.events) == 1
        real.verify('right')
        assert len(f.pending) == 8
    assert real._return_preparation is None and len(f.removed) == 8
    assert descriptions == [f.responses[2].values[0].string_value]*2


@pytest.mark.parametrize('invalidation', ['age', 'command', 'recovery', 'arm', 'late_age', 'late_command', 'late_recovery'])
def test_return_preparation_invalid_queries_fall_back_to_fresh_reads(preparation, invalidation):
    f, real = preparation, preparation.real
    def invalidate():
        if invalidation.endswith('age'): f.clock[0] += .3
        elif invalidation.endswith('command'): real.motion_submissions += 1
        elif invalidation.endswith('recovery'): real.communication_recovery_count += 1
        else: real._return_preparation.arm = 'left'
    with real.return_preparation('right'):
        real.prepare_return_phase('right')
        if invalidation.startswith('late_'):
            wait = real.node.wait
            def delayed(future, timeout):
                real.node.wait = wait
                invalidate()
                return wait(future, timeout)
            real.node.wait = delayed
        else:
            invalidate()
        real.verify('right')
        assert not any(fields.get('outcome') == 'consumed' for _, fields in f.events)
        assert len(f.pending) == len(f.removed) == 8


@pytest.mark.parametrize('failure', [False, True])
def test_return_preparation_scope_discards_unused_queries(preparation, failure):
    f, real = preparation, preparation.real
    try:
        with real.return_preparation('right'):
            real.prepare_return_phase('right')
            with pytest.raises(RuntimeError, match='exclusive'):
                with real.return_preparation('left'): pass
            if failure: raise RuntimeError('retreat failed')
    except RuntimeError:
        assert failure
    assert real._return_preparation is None
    assert len(f.removed) == 4 and all(p.cancelled() for _, p in f.pending)
    with real.return_preparation('right'):
        assert real._return_preparation.endpoint is None
        real.verify('right')
        assert len(f.pending) == 8


@pytest.mark.parametrize('change', ['', 'state', 'target', 'model', 'command', 'recovery', 'age', 'arm'])
def test_return_preparation_endpoint_reuse_requires_identical_fresh_full_state(preparation, change):
    f, real = preparation, preparation.real
    with real.return_preparation('right'):
        receipt = save_endpoint(f)
        target, arm = receipt.target, 'right'
        if change == 'state': f.state.position[-1] += .001  # Even another arm's jaw invalidates.
        elif change == 'target': target = CartesianPose((0., 0., .6), target.orientation)
        elif change == 'model': real._verified_models = ('new', 'new')
        elif change == 'command': real.motion_submissions += 1
        elif change == 'recovery': real.communication_recovery_count += 1
        elif change == 'age': f.clock[0] += .251
        elif change == 'arm': arm = 'left'
        result = real.reuse_release_retreat_endpoint(arm, target)
        assert result == (None if change else receipt.measured)
        if not change: assert real.reuse_release_retreat_endpoint(arm, target) is None


def test_return_preparation_model_change_blocks_home_even_when_both_models_agree(preparation):
    f, real = preparation, preparation.real
    with real.return_preparation('right'):
        save_endpoint(f)
        real.prepare_return_phase('right')
        f.responses[2].values[0].string_value = '<robot name="new"/>'
        f.responses[3].values[0].string_value = '<robot name="new"/>'
        with pytest.raises(RuntimeError, match='changed after release retreat'):
            real.verify('right')


@pytest.mark.parametrize('mode', ['strict', 'outside', 'ignored', 'no_orientation'])
def test_return_preparation_receipt_only_after_strict_measured_endpoint(preparation, mode):
    f, real = preparation, preparation.real
    real.arm_holds = {}
    real.wait_for_settle = lambda _: real.monitor()
    pose = CartesianPose((0., 0., .5), (0., 0., 0., 1.))
    real.endpoint_metrics = lambda *args, **kwargs: dict(
        actual_tcp_position_m=[0., 0., .4 if mode == 'outside' else .5],
        actual_tcp_orientation_xyzw=list(pose.orientation),
        target_tcp_position_m=list(pose.position), target_tcp_orientation_xyzw=list(pose.orientation))
    with real.return_preparation('right'):
        kwargs = dict(endpoint_pose=pose, check_orientation=mode != 'no_orientation',
                      allow_endpoint_error=mode == 'ignored')
        if mode == 'outside':
            with pytest.raises(RuntimeError, match='outside tolerance'):
                real.verify_arm_endpoint('right', (0.,)*5, {}, **kwargs)
        else:
            real.verify_arm_endpoint('right', (0.,)*5, {}, **kwargs)
        assert (real._return_preparation.endpoint is not None) == (mode == 'strict')
        assert len(f.pending) == (4 if mode == 'strict' else 0)


def test_return_preparation_unready_service_keeps_normal_verification(preparation):
    f, real = preparation, preparation.real
    f.clients[0].service_is_ready = lambda: False
    with real.return_preparation('right'):
        real.prepare_return_phase('right')
        assert not f.pending
        real.verify('right')
        assert len(f.pending) == 4
        assert not any(fields.get('outcome') == 'consumed' for _, fields in f.events)


def test_readonly_pending_batch_can_be_closed_without_waiting(preparation):
    f = preparation
    batch = ReadOnlyBatch(f.real.node, f.real._model_requests())
    batch.close()
    batch.close()
    assert len(f.removed) == 4
    with pytest.raises(RuntimeError, match='consumed or closed'): batch.result()


@pytest.mark.parametrize('shared', [False, True])
def test_return_preparation_linear_removes_only_duplicate_endpoint_fk(command_pipeline, shared):
    node, selected, state, limits, plans, sent, trace = command_pipeline
    real = node.real
    real.share_return_preparation = shared
    real._verified_models, real.communication_recovery_count = ('same', 'same'), 0
    value = real.value
    real.value = lambda name: .25 if name == 'hardware_guard_max_age_sec' else value(name)
    real.wait_for_settle = lambda _: real.monitor()
    real.verify_arm_endpoint = HardwareExecution.verify_arm_endpoint.__get__(real)
    client = NS(service_is_ready=lambda: False)
    real.hardware = real.controllers = real.parameters = real.moveit_parameters = client
    fk_calls, pose = [], node.pose
    def counted(arm):
        fk_calls.append(arm)
        return pose(arm)
    node.pose = counted
    with real.return_preparation('right'):
        node.linear('right', selected.fk_grasp_pose, 'retreat_after_release', check_collisions=False)
    assert len(sent) == 1
    assert len(fk_calls) == (0 if shared else 1)
    assert any(name == 'return_endpoint_handoff' and fields['reused'] for name, fields in trace) == shared
    assert real.last_execution_metrics['endpoint_within_tolerance']


def recorded_fixture(command_pipeline):
    node, _, state, limits, _, sent, trace = command_pipeline
    names = ARM_JOINT_NAMES['right']
    path = RecordedTrajectory(names, tuple(
        CommandPoint(seconds*10**9, (1., height, 0., 0., 0.), (0.,)*5)
        for seconds, height in [(0, .1), (2, .2), (4, .3)]))
    window = (10**9, 3*10**9)
    expected = stopped_recorded_window(path, *window, limits, velocity=.8, acceleration=1.2)
    state.position = [*expected.points[0].positions, .2]
    verifications = []
    node.real.verify = lambda arm: verifications.append(arm) or limits
    return node, path, window, expected, verifications, sent, trace, limits


@pytest.mark.parametrize('check_collisions', [True, False])
def test_return_window_verifies_once_and_preserves_exact_cropped_commands(
        command_pipeline, check_collisions):
    node, path, window, expected, verified, sent, trace, _ = recorded_fixture(command_pipeline)
    endpoint = CartesianPose((0., 0., .25), (0., 0., 0., 1.))
    endpoints = []
    node.real.verify_arm_endpoint = lambda arm, target, limits, actual, **kw: (
        endpoints.append(kw['endpoint_pose']) or actual, {})
    node.real.execute_return_segment('right', path, *window,
                                    check_collisions=check_collisions, endpoint_pose=endpoint)
    assert verified == ['right'] and endpoints == [endpoint]
    assert len(sent) == 1
    actual = sent[0].points
    assert [p.time_from_start.sec*10**9+p.time_from_start.nanosec for p in actual] == [
        p.time_ns for p in expected.points]
    assert [tuple(p.positions) for p in actual] == [p.positions for p in expected.points]
    assert [tuple(p.velocities) for p in actual] == [p.velocities for p in expected.points]
    assert sum(stage == 'return_replay_timing' for stage, _ in trace) == 1
    assert node.real.last_execution_metrics['collision_checks_enabled'] is check_collisions


@pytest.mark.parametrize('failure', ['stale_model', 'new_joint_limit', 'bad_window', 'collision'])
def test_return_window_fresh_verification_and_invalid_path_block_dispatch(command_pipeline, failure):
    node, path, window, _, verified, sent, _, limits = recorded_fixture(command_pipeline)
    if failure == 'stale_model':
        def reject(_):
            raise RuntimeError('model mismatch')
        node.real.verify = reject
    elif failure == 'new_joint_limit':
        name = path.joint_names[1]
        limits[name] = replace(limits[name], upper=.12)
    elif failure == 'bad_window':
        window = (3*10**9, 10**9)
    else:
        from types import SimpleNamespace
        node.call = lambda *_: SimpleNamespace(valid=False, contacts=[])
    with pytest.raises((RuntimeError, ValueError)):
        node.real.execute_return_segment('right', path, *window,
            endpoint_pose=CartesianPose((0., 0., .25), (0., 0., 0., 1.)))
    assert not sent
    assert len(verified) <= 1
