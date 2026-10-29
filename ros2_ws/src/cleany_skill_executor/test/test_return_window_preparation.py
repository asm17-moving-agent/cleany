"""Recorded windows share one fresh verification; all hardware ports are mocked."""
from dataclasses import replace

import pytest

from cleany_skill_executor.core.cartesian import CartesianPose
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.core.recorded_trajectory import (
    CommandPoint, RecordedTrajectory, stopped_recorded_window,
)
from test_arrival_approach import command_pipeline


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
