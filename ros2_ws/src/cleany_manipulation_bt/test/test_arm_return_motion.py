"""Smooth return geometry and synchronized closing without physical access."""
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState

from cleany_manipulation_bt.arm_return_motion import move_arm_return, shorten_return_path
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, PlannedPath, InfrastructureError
from cleany_skill_executor.core.joint_path import executable_joint_path


def test_logged_return_start_uses_one_smooth_connection_and_closes_on_that_path():
    names = ARM_JOINT_NAMES['right']
    start = (1.7629816532140783, .29004267130991535, .1918647662343087,
             1.5006386753174614, .47045580821563315)
    home = (1.5370487494614125, 3.451456772742693, 3.1354567304382504,
            -1.7778837331594581, 1.7410681942502026)
    state = JointState(name=[*names, 'right_gripper_joint'], position=[*start, 1.74532919957])
    dispatched = []
    ops = NS(check_execution=lambda: None, event=lambda *args, **kwargs: None,
        motion=NS(set_current_state=lambda _: None, set_gripper_position=lambda *_: None,
                  _validated_cubic_path=lambda arm, path: path,
                  plan=lambda *_: pytest.fail('A valid direct connection must bypass RRT')),
        real=NS(monitor=lambda: state,
                execute_arm_gripper=lambda *args: dispatched.append(args)))
    move_arm_return(ops, 'right', home, -.374532977628)
    arm, trajectory, jaw = dispatched[0]
    assert arm == 'right' and len(trajectory.points) == 2
    assert tuple(trajectory.points[0].positions) == start
    assert tuple(trajectory.points[-1].positions) == home
    assert jaw.points[0].positions == pytest.approx([1.74532919957])
    assert jaw.points[-1].positions == pytest.approx([-.374532977628])
    # Geometry alone can cover the logged 51.8s move in about 6.5s at the
    # existing caps. This is a timing comparison, not physical collision proof.
    cubic = executable_joint_path([start, home])
    assert cubic.minimum_duration([.8]*5, [1.2]*5)*1.05 < 6.5
    assert cubic.sample(0, 0.).derivatives == pytest.approx((0.,)*5)
    assert cubic.sample(0, 1.).derivatives == pytest.approx((0.,)*5)


def test_only_faster_fully_validated_shortcuts_replace_a_detour():
    names = ARM_JOINT_NAMES['right']
    path = PlannedPath(names, ((0.,)*5, (1.,)*5, (-.5,)*5, (.4,)*5))
    checked = []
    def validate(candidate):
        checked.append(candidate)
        return candidate
    result = shorten_return_path(path, validate, (.8,)*5, (1.2,)*5, 8)
    assert len(result.points) == 2
    assert result.points[0] == path.points[0] and result.points[-1] == path.points[-1]
    assert result in checked


def test_colliding_shortcuts_keep_the_valid_original_path_and_respect_budget():
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, tuple((i/10., (-1.)**i*.2, 0., 0., 0.) for i in range(12)))
    checked = []
    result = shorten_return_path(path, lambda candidate: checked.append(candidate),
                                 (.8,)*5, (1.2,)*5, 3)
    assert result is path and len(checked) == 3


def test_known_rejected_direct_connection_is_not_checked_again():
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, ((0.,)*5, (1.,)*5, (-.5,)*5, (.4,)*5))
    direct = PlannedPath(names, (path.points[0], path.points[-1]))
    checked = []
    result = shorten_return_path(path, lambda candidate: checked.append(candidate),
        (.8,)*5, (1.2,)*5, 8, known_invalid=(direct,))
    assert result is path and checked
    assert all(candidate.points != direct.points for candidate in checked)


def test_infrastructure_failure_during_shortcut_never_authorizes_a_path():
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, ((0.,)*5, (1.,)*5, (.4,)*5))
    def unavailable(_):
        raise InfrastructureError('Scene unavailable')
    with pytest.raises(InfrastructureError):
        shorten_return_path(path, unavailable, (.8,)*5, (1.2,)*5, 8)


def test_blocked_direct_connection_uses_and_checks_rrt_before_dispatch():
    names = ARM_JOINT_NAMES['left']
    state = JointState(name=[*names, 'left_gripper_joint'], position=[0.]*5 + [1.])
    home = (.4,)*5
    detour = PlannedPath(names, ((0.,)*5, (1.,)*5, home))
    calls, events = [], []
    def plan(arm, goal, start, *, planner_id, trajectory_scaling):
        assert planner_id == 'RRTConnectReturnkConfigDefault'
        assert trajectory_scaling == 1.
        calls.append('rrt')
        return detour
    ops = NS(check_execution=lambda: None, event=lambda event, **fields: events.append((event, fields)),
        value=lambda name: {'bt_arm_return_shortcut_attempts': 8,
                            'bt_arm_return_optimization_budget_sec': .5,
                            'bt_arm_return_planning_trajectory_scaling': 1.,
                            'bt_arm_return_planner_id': 'RRTConnectReturnkConfigDefault'}[name],
        motion=NS(set_current_state=lambda _: None, set_gripper_position=lambda *_: None,
                  _validated_cubic_path=lambda arm, path, **_: calls.append('validate'),
                  plan=plan),
        real=NS(monitor=lambda: state, value=lambda _: .8,
                verify=lambda _: {n: NS(velocity=1., acceleration=2.) for n in names},
                execute_arm_gripper=lambda arm, trajectory, jaw: calls.append(trajectory)))
    move_arm_return(ops, 'left', home, -.37)
    assert calls[0:2] == ['validate', 'rrt']
    assert [tuple(p.positions) for p in calls[-1].points] == list(detour.points)
    assert [fields['phase'] for event, fields in events if event == 'arm_return_planning_phase'] == [
        'direct_validation', 'rrt', 'shortcuts']
    planned = events[-1][1]
    assert planned['planning_wall_sec'] >= sum(planned[key] for key in (
        'direct_validation_wall_sec', 'rrt_wall_sec', 'shortcuts_wall_sec'))


def test_three_knot_detour_is_moved_closer_even_when_direct_connection_is_blocked():
    from math import dist
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, ((0.,)*5, (2., 1., 0., 0., 0.), (0., 2., 0., 0., 0.)))
    checked, report = [], {}
    def validate(candidate):
        checked.append(candidate)
        cubic = executable_joint_path(list(candidate.points)).retimed((.8,)*5, (1.2,)*5)
        # A synthetic obstacle blocks the direct connection. Check the whole
        # resulting spline, including neighboring junctions after knot motion.
        return (candidate if all(not .8 < q.positions[1] < 1.2 or q.positions[0] >= .4
                for q in cubic.samples(.01, 2)) else None)
    direct = PlannedPath(names, (path.points[0], path.points[-1]))
    result = shorten_return_path(path, validate, (.8,)*5, (1.2,)*5, 8,
                                 known_invalid=(direct,), report=report)
    assert result in checked and len(result.points) == 3
    assert result.points[0] == path.points[0] and result.points[-1] == path.points[-1]
    assert .4 <= result.points[1][0] <= .6
    assert sum(dist(a,b) for a,b in zip(result.points,result.points[1:])) < sum(
        dist(a,b) for a,b in zip(path.points,path.points[1:]))
    assert report['final_arm_duration_sec'] <= report['initial_arm_duration_sec']
    assert report['accepted_improvements'] >= 1 and report['candidate_checks'] <= 8


def test_optional_search_stops_at_time_budget_and_keeps_valid_original():
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, ((0.,)*5, (2., 1., 0., 0., 0.), (0., 2., 0., 0., 0.)))
    clock, checked, report = [0.], [], {}
    def reject(candidate):
        checked.append(candidate)
        clock[0] += .2
    result = shorten_return_path(path, reject, (.8,)*5, (1.2,)*5, 8,
        deadline_monotonic=.5, monotonic=lambda: clock[0], report=report)
    assert result is path and len(checked) == 3
    assert report['budget_exhausted'] and report['candidate_checks'] == 3


def test_budget_expired_before_search_performs_no_validation():
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, ((0.,)*5, (1.,)*5, (.4,)*5))
    result = shorten_return_path(path, lambda _: pytest.fail('Expired search issued a request'),
        (.8,)*5, (1.2,)*5, 8, deadline_monotonic=1., monotonic=lambda: 2.)
    assert result is path


def test_validator_cannot_change_return_target():
    names = ARM_JOINT_NAMES['left']
    path = PlannedPath(names, ((0.,)*5, (2.,)*5, (.4,)*5))
    with pytest.raises(RuntimeError, match='endpoints'):
        shorten_return_path(path, lambda _: PlannedPath(names, ((0.,)*5, (.3,)*5)),
                            (.8,)*5, (1.2,)*5, 8)
