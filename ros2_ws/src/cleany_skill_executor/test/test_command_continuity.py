from copy import deepcopy
from contextlib import contextmanager
from types import SimpleNamespace as NS
import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.core.hardware_execution import ArmHold, CollisionValidationError, PhysicalJointLimit, bounded_path, continue_joint_path
import cleany_skill_executor.hardware_execution as transport


def test_collision_opt_out_does_not_wait_for_a_map_or_change_the_next_motion():
    trace = []
    @contextmanager
    def frozen():
        trace.append('fresh_scene')
        yield
    fake = NS(frozen_sensor_scene=frozen,
              _execute_with_recovery=lambda *args, **options:
                  trace.append(options['check_collisions']))
    transport.HardwareExecution.execute(fake, 'right', object(), 'arm', check_collisions=False)
    transport.HardwareExecution.execute(fake, 'right', object(), 'arm')
    assert trace == [False, 'fresh_scene', True]


def test_continuation_preserves_held_start_without_changing_the_new_destination():
    hold = ArmHold((.1, .2), (.14, .18))
    path = continue_joint_path([(.098, .203), (.12, .22), (.15, .25)], hold, .06)
    assert path[0] == hold.command
    assert path[1:] == [(.12, .22), (.15, .25)]
    assert path[0] != (.098, .203)
    # The small measured/nominal entry residual is blended away, not integrated
    # into a larger load bias on every new segment.
    next_path = continue_joint_path(
        [(.148, .253), (.16, .26)], ArmHold((.15, .25), path[-1]), .06)
    assert next_path[0] == path[-1]
    assert next_path[-1] == pytest.approx((.16, .26))


@pytest.mark.parametrize('hold', [ArmHold((0.,), (.061,)),
                                ArmHold((0.,), (float('nan'),)),
                                ArmHold((0., 0.), (0.,))])
def test_invalid_or_excessive_bias_is_rejected(hold):
    with pytest.raises(ValueError):
        continue_joint_path([(0.,), (.1,)], hold, .06)


def test_continuous_start_still_obeys_physical_limits():
    limit = PhysicalJointLimit(-1., 1., 1., 2., 2048., 1., 0., 0., 4095.)
    path = continue_joint_path([(.95,), (1.01,)], ArmHold((.95,), (.99,)), .06)
    with pytest.raises(ValueError, match='range'):
        bounded_path(('joint',), path, {'joint': limit}, velocity=.8, acceleration=1.2)


@pytest.mark.parametrize('collision', [False, True, 'held_endpoint', 'later_path'])
@pytest.mark.parametrize('check_collisions', [True, False])
def test_next_measured_cartesian_path_carries_bias_and_checks_nominal_endpoint(
        monkeypatch, collision, check_collisions):
    names = ARM_JOINT_NAMES['left']
    nominal = (.1, 0., 0., 0., 0.)
    held = (.14, 0., 0., 0., 0.)
    target = (.12, 0., 0., 0., 0.)
    state = JointState(name=list(names), position=list(nominal))
    limits = {n: PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 0., 4095.)
              for n in names}
    values = {
        'hardware_velocity_rad_s': .8, 'hardware_acceleration_rad_s2': 1.2,
        'hardware_path_step_rad': .01, 'hardware_path_simplification_tolerance_rad': .002,
        'hardware_start_tolerance_rad': .005, 'hardware_arrival_tolerance_rad': .07,
        'hardware_start_blend_max_rad': .03,
        'hardware_uncommanded_tolerance_rad': .03, 'hardware_settle_time_sec': .5,
        'hardware_command_continuity_max_offset_rad': .06,
    }
    sent, checked, endpoints, passed_hold = [], [], [], []
    monkeypatch.setattr(transport.time, 'sleep', lambda _: None)
    monkeypatch.setattr(transport, 'ActionClient', lambda *_: NS(destroy=lambda: None))

    def call(_client, request):
        q = tuple(request.robot_state.joint_state.position)
        checked.append(q)
        later_collision = bool(passed_hold) and q[0] < held[0]-1e-8
        if q == pytest.approx(held):
            passed_hold.append(True)
        blocked = (collision is True
                   or collision == 'held_endpoint' and q == pytest.approx(held)
                   or collision == 'later_path' and later_collision)
        return NS(valid=not blocked, contacts=[])

    def run_action(_client, goal, _timeout, monitor):
        assert monitor
        sent.append(goal.trajectory)
        state.position = list(goal.trajectory.points[-1].positions)
        return NS(error_code=0, SUCCESSFUL=0)

    def verify_endpoint(_arm, desired, _limits, actual, *, check_orientation=True):
        assert check_orientation
        endpoints.append(desired)
        return actual, {}

    fake = NS(
        arm_holds={'left': ArmHold(nominal, held)},
        verify=lambda _: limits, monitor=lambda: deepcopy(state), value=values.__getitem__,
        validity=object(), uncommanded=None, verify_arm_endpoint=verify_endpoint, sensor_scene_frozen=True,
        last_execution_metrics={},
        fail_stop=lambda: None,
        node=NS(call=call, run_action=run_action,
                get_logger=lambda: NS(info=lambda _: None, error=lambda _: None, warning=lambda _: None)),
    )
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=list(nominal)),
        JointTrajectoryPoint(positions=list(target)),
    ])
    if collision and check_collisions:
        with pytest.raises(CollisionValidationError, match='collision') as rejected:
            transport.HardwareExecution._execute(fake, 'left', trajectory, 'arm')
        expected_phase = 'command_path' if collision == 'later_path' else 'hold_bridge'
        assert rejected.value.phase == expected_phase
        assert rejected.value.entry_blocked is (expected_phase == 'hold_bridge')
        assert sent == []
        return
    transport.HardwareExecution._execute(
        fake, 'left', trajectory, 'arm', check_collisions=check_collisions)
    assert sent[0].header.stamp.sec == sent[0].header.stamp.nanosec == 0
    assert tuple(sent[0].points[0].positions) == held
    assert tuple(sent[0].points[-1].positions) == target
    assert endpoints == [target]  # never validate against the biased command
    if check_collisions:
        assert checked[0] == nominal  # real-state bridge remains collision-checked
        assert any(q == pytest.approx(held) for q in checked)
    else:
        assert checked == []
    assert fake.last_execution_metrics['collision_checks_enabled'] is check_collisions
    assert fake.arm_holds['left'].nominal == target
