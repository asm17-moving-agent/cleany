"""Remove redundant work while preserving acquisition and command gates."""
from copy import deepcopy
import json
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.hardware_execution import (
    PhysicalJointLimit, bounded_path,
)
from cleany_skill_executor.core.joint_path import CubicJointPath
from cleany_skill_executor.core.joint_stability import JointStabilityHistory
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_skill_executor.state_validity_batch import StateValidityBatch


def acquisition_history():
    history = JointStabilityHistory()
    positions = {'left': .1, 'right': .2, 'head': .3}
    for stamp in range(1_000_000_000, 1_500_000_001, 100_000_000):
        history.observe(stamp, positions, 0)
    return history, positions


@pytest.mark.parametrize('change', [
    'none', 'current', 'other_arm', 'head', 'missing', 'stale', 'future',
    'gap', 'recovery', 'reset', 'clock_reset', 'nan',
])
def test_exact_scene_window_rejects_movement_and_invalid_acquisition(change):
    history, positions = acquisition_history()
    now = 1_600_000_000
    if change == 'current':
        positions['left'] += .00001
    if change in ('other_arm', 'head'):
        positions['right' if change == 'other_arm' else 'head'] += .00001
        history.observe(1_600_000_000, positions, 0)
    if change == 'missing':
        positions.pop('head')
    if change == 'stale':
        now += 200_000_000
    if change == 'future':
        now = 1_400_000_000
    if change == 'gap':
        now = 2_000_000_000
        history.observe(now, positions, 0)
    if change == 'recovery':
        history.observe(now, positions, 1)
    if change == 'reset':
        history.clear()
    if change == 'clock_reset':
        history.observe(1_200_000_000, positions, 0)
    if change == 'nan':
        positions['head'] = float('nan')
    stamp = history.unchanged_since(positions, now_ns=now,
                                  duration_sec=.3, maximum_age_sec=.25)
    assert stamp == (1_000_000_000 if change == 'none' else None)


@pytest.mark.parametrize('value', [0., -.1, float('nan'), float('inf')])
def test_exact_scene_window_rejects_invalid_duration(value):
    history, positions = acquisition_history()
    assert history.unchanged_since(positions, now_ns=1_600_000_000,
                                  duration_sec=value, maximum_age_sec=.25) is None


@pytest.mark.parametrize('moving', [False, True])
def test_scene_barrier_reuses_only_exact_stationary_history(moving):
    history, positions = acquisition_history()
    if moving:
        positions['head'] += .00001
    state = JointState(name=list(positions), position=list(positions.values()))
    fake = NS(reuse_scene_barrier=True, wait_for_settle=lambda _: state,
              node=NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_600_000_000))),
              _stability_history=history, motion_submissions=3,
              communication_recovery_count=2, value=lambda _: .3)
    barrier = HardwareExecution.capture_scene_barrier(fake, 'left')
    assert barrier == dict(stamp_ns=1_600_000_000 if moving else 1_000_000_000,
                          positions=positions, submissions=3, generation=2)


def gripper_fixture(monkeypatch, positions):
    import cleany_skill_executor.hardware_execution as module
    name = 'left_gripper_joint'
    state = JointState(name=[name, 'head_pan_joint'], position=[positions[0], 0.])
    limit = PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
    limits = {name: limit}
    sent, checked, logs, verified = [], [], [], []
    monkeypatch.setattr(module, 'ActionClient', lambda *_: NS(destroy=lambda: None))
    monkeypatch.setattr(module.time, 'sleep', lambda _: None)
    values = dict(hardware_velocity_rad_s=.8, hardware_acceleration_rad_s2=1.2,
                  hardware_path_step_rad=.01, hardware_path_simplification_tolerance_rad=.002,
                  hardware_start_tolerance_rad=.005, hardware_start_blend_max_rad=.03,
                  hardware_arrival_tolerance_rad=.015,
                  hardware_uncommanded_tolerance_rad=.03, hardware_settle_time_sec=.5)

    def call(_client, request):
        checked.append(dict(zip(request.robot_state.joint_state.name,
                                request.robot_state.joint_state.position))[name])
        return NS(valid=True)

    def run_action(_client, goal, _timeout, **_kwargs):
        sent.append(goal.trajectory)
        state.position[0] = goal.trajectory.points[-1].positions[0]
        return NS(error_code=0, SUCCESSFUL=0)

    node = NS(call=call, run_action=run_action,
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)),
              get_logger=lambda: NS(info=logs.append, warning=logs.append))
    hw = NS(node=node, verify=lambda _: verified.append(True) or limits,
            monitor=lambda: deepcopy(state), value=values.__getitem__, validity=object(),
            fail_stop=lambda: None, uncommanded=None, arm_holds={}, last_execution_metrics={},
            contact_criteria={'left': NS(closed_goal_raw=1800)}, _physical_limits={'left': limits})
    hw.closed_contact_position = MethodType(HardwareExecution.closed_contact_position, hw)
    trajectory = JointTrajectory(joint_names=[name], points=[
        JointTrajectoryPoint(positions=[q]) for q in positions])
    return hw, trajectory, limits, sent, checked, logs, verified


@pytest.mark.parametrize('positions', [(.8848652492263254, -.35459122738527254),
                                        (.1500884518291034, 1.74532919957)])
def test_two_endpoint_gripper_keeps_curve_bounds_duration_and_dense_validation(monkeypatch, positions):
    hw, trajectory, limits, sent, checked, logs, verified = gripper_fixture(monkeypatch, positions)
    samples, duration = bounded_path(tuple(trajectory.joint_names), [(q,) for q in positions],
                                     limits, velocity=.8, acceleration=1.2)
    HardwareExecution._execute(hw, 'left', trajectory, 'gripper')
    assert len(verified) == 1 and len(sent) == 1 and len(sent[0].points) == 2
    assert len(checked) == len(samples) > 100
    assert checked == pytest.approx([sample.positions[0] for sample in samples])
    command = RecordedTrajectory(tuple(sent[0].joint_names), tuple(
        CommandPoint(p.time_from_start.sec*10**9+p.time_from_start.nanosec,
                     tuple(p.positions), tuple(p.velocities)) for p in sent[0].points))
    path = CubicJointPath([(q,) for q in positions])
    for t in np.linspace(0, command.points[-1].time_ns, 301).astype(int):
        fraction = t/command.points[-1].time_ns
        expected = path.sample(0, fraction)
        actual = command.sample_at(int(t))
        assert actual.positions == pytest.approx(expected.positions, abs=1e-12)
        assert actual.velocities == pytest.approx([v/duration for v in expected.derivatives], abs=1e-9)
    assert command.points[-1].time_ns == round(duration*1e9)
    assert all(not p.accelerations for p in sent[0].points)
    prepared = next(json.loads(line) for line in logs if 'hardware_trajectory_prepared' in line)
    assert prepared['controller_points'] == 2 and prepared['command_samples'] == len(samples)


def test_multisegment_gripper_does_not_remove_turnaround(monkeypatch):
    hw, trajectory, _, sent, checked, _, _ = gripper_fixture(monkeypatch, (0., 1., .2))
    HardwareExecution._execute(hw, 'left', trajectory, 'gripper')
    assert len(sent[0].points) > 2
    assert max(p.positions[0] for p in sent[0].points) == pytest.approx(1.)
    assert max(checked) == pytest.approx(1.)


def test_invalid_interior_gripper_waypoint_cannot_be_compacted_away(monkeypatch):
    hw, trajectory, _, sent, _, _, _ = gripper_fixture(monkeypatch, (0., 2.1, .2))
    with pytest.raises(ValueError, match='physical joint/calibration range'):
        HardwareExecution._execute(hw, 'left', trajectory, 'gripper')
    assert not sent


def test_cached_close_proposal_is_rejected_if_fresh_calibration_changes(monkeypatch):
    hw, trajectory, limits, sent, _, _, verified = gripper_fixture(monkeypatch, (.8, 0.))
    target = hw.closed_contact_position('left')
    assert verified == []
    trajectory.points[-1].positions = [target]
    old = limits['left_gripper_joint']
    fresh = PhysicalJointLimit(old.lower, old.upper, old.velocity, old.acceleration,
                               old.zero+1, old.direction, old.offset, old.raw_min, old.raw_max)
    hw.verify = lambda _: {'left_gripper_joint': fresh}
    with pytest.raises(RuntimeError, match='calibrated final gripper goal'):
        HardwareExecution._execute(hw, 'left', trajectory, 'gripper', contact_close=True)
    assert not sent


@pytest.mark.parametrize('cached', [False, True])
def test_close_proposal_only_queries_model_if_cache_missing(monkeypatch, cached):
    hw, _, _, _, _, _, verified = gripper_fixture(monkeypatch, (.8, 0.))
    if not cached:
        hw._physical_limits.clear()
    assert np.isfinite(hw.closed_contact_position('left'))
    assert len(verified) == (0 if cached else 1)


@pytest.mark.parametrize('available', [False, True])
@pytest.mark.parametrize('invalid', [None, 63, 64, 514])
def test_larger_collision_batches_keep_all_states_and_global_failure_index(available, invalid):
    client = NS(service_is_ready=lambda: available)
    batch = StateValidityBatch(object(), batch_size=64, client=client)
    calls, seen = [], []
    states = [RobotState(joint_state=JointState(name=['q'], position=[float(i)])) for i in range(515)]

    def call(actual_client, request, **_kwargs):
        chunk = request.robot_states if actual_client is client else [request.robot_state]
        calls.append(len(chunk))
        valid = []
        for state in chunk:
            index = int(state.joint_state.position[0])
            seen.append(index)
            valid.append(index != invalid)
            if not valid[-1]:
                break
        return (NS(success=True, message='', valid=valid, contacts=[],
                   first_invalid_index=-1 if all(valid) else len(valid)-1)
                if actual_client is client else NS(valid=valid[0]))

    hw = NS(_batch_validity=batch, monitor=lambda: None, node=NS(call=call), validity=object())
    result = list(HardwareExecution._collision_responses(hw, states))
    count = 515 if invalid is None else invalid+1
    assert seen == list(range(count))
    assert [index for index, _ in result] == seen
    assert result[-1][1].valid is (invalid is None)
    if invalid is None:
        assert len(calls) == (9 if available else 515)
