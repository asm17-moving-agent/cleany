"""Preparation may reduce copies, never samples, guard coverage or abort gates."""
from copy import deepcopy
from contextlib import contextmanager
from types import MethodType, SimpleNamespace as NS

import pytest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, REQUIRED_JOINT_NAMES
from cleany_skill_executor.hardware_execution import HardwareExecution


def fixture():
    names = [*REQUIRED_JOINT_NAMES, 'head_pan_joint', 'head_tilt_joint']
    measured = JointState(name=names, position=[.1]*len(names),
                          velocity=[.2]*len(names), effort=[.3]*len(names))
    measured.header.frame_id = 'base_link'
    measured.header.stamp.sec = 100
    expected = ARM_JOINT_NAMES['right']
    baseline = dict(zip(names, measured.position))
    samples = [NS(positions=(index*.001,)*5) for index in range(515)]
    return measured, expected, baseline, samples


def test_prepared_states_preserve_metadata_independence_and_every_clearance_check(monkeypatch):
    import cleany_skill_executor.hardware_execution as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    measured, expected, baseline, samples = fixture()
    checks, guards = [], []
    hw = NS(node=NS(), monitor=lambda: checks.append(True),
            _approach_clearance_guard=lambda q, index, phase: guards.append((q, index, phase)))
    actual = HardwareExecution._prepare_collision_states(
        hw, 'right', expected, samples, baseline, measured, hold_bridge_samples=3)
    reference = []
    for sample in samples:
        state = deepcopy(measured)
        q = {**baseline, **dict(zip(expected, sample.positions))}
        state.position = [q[name] for name in state.name]
        reference.append(RobotState(joint_state=state, is_diff=True))
    assert actual == reference and len(checks) == 10
    assert [row[1] for row in guards] == list(range(515))
    assert [row[2] for row in guards[:4]] == ['hold_bridge']*3+['command_path']
    actual[0].joint_state.header.stamp.sec = 200
    actual[0].joint_state.name[0] = 'changed'
    actual[0].joint_state.velocity[0] = 99.
    assert actual[1:] == reference[1:]
    assert measured.header.stamp.sec == 100 and measured.name[0] != 'changed'


def test_slow_preparation_checks_time_between_chunk_boundaries(monkeypatch):
    import cleany_skill_executor.hardware_execution as module
    now, index_seen, check_indices = [0.], [-1], []
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    measured, expected, baseline, samples = fixture()

    def guard(q, index, phase):
        index_seen[0] = index
        now[0] += .004

    hw = NS(node=NS(), monitor=lambda: check_indices.append(index_seen[0]),
            _approach_clearance_guard=guard)
    HardwareExecution._prepare_collision_states(hw, 'right', expected, samples, baseline, measured)
    assert len(check_indices) > 100
    assert max(b-a for a,b in zip(check_indices, check_indices[1:])) <= 3
    assert check_indices[-1] == 514


@pytest.mark.parametrize('abort_check', [1, 2, 10])
def test_abort_at_entry_chunk_or_final_gate_prevents_every_rpc(monkeypatch, abort_check):
    import cleany_skill_executor.hardware_execution as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    measured, expected, baseline, samples = fixture()
    checks, calls = [], []

    def monitor():
        checks.append(True)
        if len(checks) == abort_check:
            raise InterruptedError('test abort or fault')

    hw = NS(node=NS(call=lambda *_: calls.append(True)), monitor=monitor, validity=object())
    with pytest.raises(InterruptedError):
        HardwareExecution.validate_collision_samples(hw, 'right', expected, samples, baseline, measured)
    assert not calls


def test_clearance_failure_in_nonmonitor_sample_prevents_every_rpc(monkeypatch):
    import cleany_skill_executor.hardware_execution as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 0.)
    measured, expected, baseline, samples = fixture()
    calls = []

    def guard(q, index, phase):
        if index == 31:
            raise ValueError('clearance failed')

    hw = NS(node=NS(call=lambda *_: calls.append(True)), monitor=lambda: None,
            _approach_clearance_guard=guard, validity=object())
    with pytest.raises(ValueError, match='clearance failed'):
        HardwareExecution.validate_collision_samples(hw, 'right', expected, samples, baseline, measured)
    assert not calls


def test_malformed_collision_sample_rejected_before_rpc():
    measured, expected, baseline, samples = fixture()
    samples[50].positions = (.1,)
    hw = NS(node=NS(), monitor=lambda: None)
    with pytest.raises(ValueError, match='joint count'):
        HardwareExecution._prepare_collision_states(hw, 'right', expected, samples, baseline, measured)


def test_partial_synchronized_client_acquisition_releases_the_first_lease():
    released = []

    def acquire(kind, name):
        if 'gripper' in name:
            raise RuntimeError('previous gripper lease unresolved')
        return NS(destroy=lambda: released.append(name))

    hw = NS(node=NS(create_motion_client=acquire))
    with pytest.raises(RuntimeError, match='unresolved'):
        HardwareExecution.motion_clients(hw, 'right', ('arm', 'gripper'))
    assert released == ['/right_arm_controller/follow_joint_trajectory']


@pytest.mark.parametrize('defer', [False, True])
@pytest.mark.parametrize('stale', [False, True])
def test_scene_wait_consolidation_keeps_real_dispatch_freshness_and_cleanup(defer, stale):
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    from cleany_skill_executor.core.physical_grasp_selection import PlannedPath
    state, names, _, _ = fixture()
    trace = []
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=[q]*5, velocities=[0.]*5) for q in (.1, .2)])
    selected = NS(selected_arm='right', planning_start_state=state,
                  pregrasp_trajectory=trajectory, gripper_joint_position=.3,
                  selected_candidate=NS(required_opening_m=.04))

    def scene_wait(**kwargs):
        if 'after_stamp_ns' in kwargs:
            trace.append('final_wait')
            if stale:
                raise RuntimeError('final scene is stale')
        else:
            trace.append('preliminary_wait')

    node = NS(verify_simulator=lambda _: None, measured=lambda: state,
        scene_monitor=NS(wait=scene_wait),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)),
        get_logger=lambda: NS(info=lambda _: None),
        scene=NS(begin=lambda *_: None, restore=lambda: trace.append('restore')),
        motion=NS(set_current_state=lambda _: None, set_gripper_position=lambda *_: None,
                  gripper_sweep_is_valid=lambda *_: True,
                  _validated_cubic_path=lambda *_: PlannedPath(names, ((.1,)*5, (.2,)*5),
                                                              (0., 1.), ((0.,)*5, (0.,)*5))),
        config={'pregrasp_opening_margin_m': .01}, _defer_pregrasp_validation=defer,
        arm_config=NS(opening=lambda width: width*10, width=.1, opened=1., closed=0.),
        release_grasp_scene_freeze=lambda: None)
    hw = NS(node=node, monitor=lambda: state, fault='', value=lambda _: .03,
            positive_integer=lambda _: 1, capture_scene_barrier=lambda _: None,
            _execute_pregrasp_parallel=lambda *_args, **_kwargs: trace.append('submit') or state)

    @contextmanager
    def fresh_scene(**kwargs):
        HardwareExecution.wait_for_fresh_sensor_scene(hw, 'test', **kwargs)
        yield

    hw.frozen_sensor_scene = fresh_scene
    hw.execute_pregrasp_parallel = MethodType(HardwareExecution.execute_pregrasp_parallel, hw)
    node.real = hw
    if stale:
        with pytest.raises(RuntimeError, match='final scene is stale'):
            PregraspPipeline.execute_pregrasp(node, selected)
    else:
        PregraspPipeline.execute_pregrasp(node, selected)
    assert trace.count('final_wait') == 1
    assert trace.count('preliminary_wait') == (0 if defer else 1)
    assert ('submit' in trace) is not stale
    assert trace[-1] == 'restore'
