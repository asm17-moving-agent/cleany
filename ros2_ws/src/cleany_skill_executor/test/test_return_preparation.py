"""One retreat/home handoff retains live guards and measured endpoint acceptance."""
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
    with real.return_preparation('right'):
        real.prepare_return_phase('right')
        assert len(f.pending) == 4 and all(not p.done() for _, p in f.pending)
        real.prepare_return_phase('right')
        f.clock[0] += .1  # Other preparation can proceed before response waits.
        assert real.verify('right') == {'limit': 'fresh'}
        assert real._last_return_model_prefetch_used and len(f.pending) == 4
        real.verify('right')
        assert not real._last_return_model_prefetch_used and len(f.pending) == 8
    assert real._return_preparation is None and len(f.removed) == 8


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
        assert not real._last_return_model_prefetch_used
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
        assert not real._last_return_model_prefetch_used and len(f.pending) == 8


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
        assert len(f.pending) == 4 and not real._last_return_model_prefetch_used


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
