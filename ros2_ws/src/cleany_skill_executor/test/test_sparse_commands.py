import numpy as np
import pytest
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit, prepare_bounded_path
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory
from cleany_skill_executor.trajectory_io import command_samples_to_message


def recorded(message):
    return RecordedTrajectory(tuple(message.joint_names), tuple(CommandPoint(
        p.time_from_start.sec*10**9+p.time_from_start.nanosec,
        tuple(p.positions), tuple(p.velocities)) for p in message.points))


@pytest.mark.parametrize('positions', [
    [(0., 0.), (1., -.5)],
    [(0., 0.), (.0009, -.0082), (-1., -.6), (-1.5, -.3)],
    [(0., 0.), (.2, .3), (-.3, .2), (.1, -.4)],
    [(.1, .2), (.1, .2)],
])
@pytest.mark.parametrize('delay_ns', [0, 300_000_000])
def test_sparse_commands_reconstruct_dense_cubic_at_knots_and_interiors(positions, delay_ns):
    names = ('a', 'b')
    limits = {n: PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.) for n in names}
    path = prepare_bounded_path(names, positions, limits, velocity=.8, acceleration=1.2)
    dense = command_samples_to_message(names, path.samples, path.duration, delay_ns=delay_ns)
    sparse = command_samples_to_message(names, path.command_knots, path.duration, delay_ns=delay_ns)
    assert len(sparse.points) < len(dense.points)
    assert sparse.points[-1].time_from_start == dense.points[-1].time_from_start
    assert all(not p.accelerations for p in sparse.points)
    before, after = recorded(dense), recorded(sparse)
    probes = {int(t) for t in np.linspace(0, before.points[-1].time_ns, 1001)}
    for point in after.points:
        probes.update(t for t in (point.time_ns-1, point.time_ns, point.time_ns+1)
                      if 0 <= t <= before.points[-1].time_ns)
    for ns in probes:
        a, b = before.sample_at(ns), after.sample_at(ns)
        assert a.positions == pytest.approx(b.positions, abs=1e-8)
        assert a.velocities == pytest.approx(b.velocities, abs=1e-6)


def test_sparse_command_rejects_timestamp_collapse():
    from cleany_skill_executor.core.joint_path import JointPathSample
    samples = [JointPathSample(p, (0.,), (0.,)) for p in (0., 1e-14, 1.)]
    with pytest.raises(ValueError, match='nanosecond rounding'):
        command_samples_to_message(('joint',), samples, 1.)
