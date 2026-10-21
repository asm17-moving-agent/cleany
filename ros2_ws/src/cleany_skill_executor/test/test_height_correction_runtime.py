"""Retreat/lift correction through synthetic motion ports; never opens motors."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import configuration_issues
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.core.hardware_execution import ArmHold, ExecutionCanceled
from cleany_skill_executor.core.height_correction import HeightCorrectionError
from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory
from cleany_skill_executor.hardware_execution import HardwareExecution
from test_arrival_approach import command_pipeline  # noqa: F401


def pose(z, x=.3):
    result = Pose()
    result.position.x, result.position.z, result.orientation.w = x, z, 1.
    return result


def plant(arm='right', goal=.5743888152472258, sag=.0185967956680563):
    node = object.__new__(CollectionPipeline)
    node.config = {'lift_height_correction_enabled': True, 'cartesian_tolerance_m': .005,
                   'lift_height_tolerance_m': .010}
    state = JointState(name=list(ARM_JOINT_NAMES[arm]), position=[goal-sag, 0., 0., 0., .2])
    actual, commands, events = pose(goal-sag), [], []
    node.pose = lambda _: deepcopy(actual)
    node.event = lambda stage, **fields: events.append((stage, fields))
    values = {'hardware_tcp_position_tolerance_m': .03,
              'hardware_tcp_orientation_tolerance_deg': 20.}
    # Synthetic FK maps the first joint to height, retaining the other ports.
    node.real = NS(value=values.__getitem__, monitor=lambda: state,
        wait_for_settle=lambda _: state,
        arm_holds={arm: ArmHold((goal, 0., 0., 0., .2), (goal, 0., 0., 0., .2))})
    node.motion = NS(set_current_state=lambda _: None,
        forward_kinematics=lambda _, q: NS(position=(.3, 0., q.positions[0]),
                                          orientation_xyzw=(0., 0., 0., 1.)))

    def linear(selected_arm, target, stage, **kwargs):
        assert selected_arm == arm
        motor = node.pose_message(kwargs['command_target']) if 'command_target' in kwargs else target
        commands.append((deepcopy(motor), stage, kwargs))
        actual.position = deepcopy(motor.position)
        actual.position.z -= sag
        q = (motor.position.z, 0., 0., 0., .2)
        state.position[0] = actual.position.z
        node.real.arm_holds[arm] = ArmHold(q, q)
        return state

    node.linear = linear
    return node, actual, commands, events


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('collisions', [False, True])
def test_loaded_endpoint_converges_with_original_goal_and_existing_motion_checks(arm, collisions):
    node, actual, commands, events = plant(arm)
    target, report = pose(.5743888152472258), {}
    original = deepcopy(target)
    node.correct_height_endpoint(arm, target, 'lift', report, check_collisions=collisions)
    assert target == original
    assert abs(target.position.z-actual.position.z) <= .010
    assert len(commands) == 1
    for command, stage, options in commands:
        assert command.position.x == target.position.x
        assert command.position.y == target.position.y
        assert command.orientation == target.orientation
        assert stage == 'lift_height_correction'
        assert options == dict(check_orientation=True, check_collisions=collisions,
                               command_target=node.cartesian_pose(command), validate_command_path=True)
    summary = report['height_corrections']['lift']
    assert summary['status'] == 'reached'
    assert summary['target_z_m'] == target.position.z
    assert summary['attempt'] == 1
    assert summary['tolerance_m'] == .010
    assert events[-1][0] == 'height_correction_completed'


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('shortfall,step', [
    (.011, .015), (.0351138972905688, .040), (.043382882998641126, .045), (.075, .075)])
def test_full_rounded_shortfall_is_sent_and_then_measured_again(arm, shortfall, step):
    node, actual, commands, events = plant(arm, goal=.5, sag=shortfall)
    node.config.update(lift_height_correction_max_attempts=3)
    report = {}
    node.correct_height_endpoint(arm, pose(.5), 'lift', report)
    assert len(commands) == 1
    assert commands[0][0].position.z == pytest.approx(.5+step)
    correction_events = [fields for stage, fields in events if stage == 'height_correction_command']
    assert len(correction_events) == 1
    assert correction_events[0]['correction_step_m'] == pytest.approx(step)
    assert report['height_corrections']['lift']['attempt'] == 1
    assert abs(actual.position.z-.5) <= .010


@pytest.mark.parametrize('sag', [.0319983857472429, .043382882998641126, .075])
def test_logged_shortfall_passes_real_transport_then_converges_against_fixed_goal(command_pipeline, sag):
    node, selected, state, _, plans, sent, _ = command_pipeline
    node.config.update(lift_height_correction_enabled=True,
                       lift_height_correction_max_attempts=3)
    action = node.run_action
    def loaded_action(*args, **options):
        result = action(*args, **options)
        state.position[1] -= sag
        return result
    node.run_action = loaded_action
    node.real.wait_for_settle = lambda _: node.real.monitor()
    node.real.verify_arm_endpoint = lambda *args, **options: HardwareExecution.verify_arm_endpoint(
        node.real, *args, **options)
    measured = tuple(state.position[:5])
    goal = deepcopy(selected.fk_grasp_pose)
    node.linear('right', goal, 'lift', check_collisions=False)
    assert plans[0] == measured
    assert node.real.last_execution_metrics['endpoint_height_shortfall_deferred']
    assert not node.real.last_execution_metrics['endpoint_within_tolerance']
    node.correct_height_endpoint('right', goal, 'lift', {}, check_collisions=False)
    assert len(sent) == 2
    assert abs(goal.position.z-node.pose('right').position.z) <= .010
    assert goal == selected.fk_grasp_pose
    assert sent[-1].points[-1].positions[1] > goal.position.z+sag-1e-9


@pytest.mark.parametrize('stage,duration', [('lift_height_correction', .3), ('lift', 1.)])
def test_short_floor_reaches_controller_only_for_height_correction(command_pipeline, stage, duration):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    node, selected, state, limits, plans, sent, _ = command_pipeline
    state.position = [*node.real.arm_holds['right'].command, .2]
    names = ARM_JOINT_NAMES['right']
    def call(client, request, *_):
        assert client is node.cartesian
        start = list(request.start_state.joint_state.position[:5])
        end = list(start)
        end[1] = request.waypoints[-1].position.z
        trajectory = JointTrajectory(joint_names=list(names), points=[
            JointTrajectoryPoint(positions=start), JointTrajectoryPoint(positions=end)])
        return NS(fraction=1.,error_code=NS(val=1),solution=NS(joint_trajectory=trajectory))
    node.call = call
    target = pose(.005, x=0.)
    node.linear('right', target, stage, check_collisions=False, validate_command_path=True)
    assert len(sent) == 1
    end = sent[0].points[-1]
    assert end.positions[1] == pytest.approx(.005)
    assert end.time_from_start.sec+end.time_from_start.nanosec/1e9 == pytest.approx(duration)
    assert list(end.velocities) == pytest.approx([0.]*5, abs=1e-12)


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('sag', [.0319983857472429, .075])
def test_retreat_defers_logged_shortfall_for_subsequent_lift(reverse, sag):
    target = .5408224849074599
    node, actual, commands, _ = plant(goal=target, sag=sag)
    node.config.update(retreat_reverse_approach_enabled=reverse,
                       lift_height_correction_max_attempts=3)
    node._approach_command_arm = 'right'
    node._approach_command_paths = (RecordedTrajectory(ARM_JOINT_NAMES['right'], (
        CommandPoint(0, (target,0.,0.,0.,.2), (0.,)*5),
        CommandPoint(10**9, (.46,0.,0.,0.,.2), (0.,)*5))),)
    actual.position.z = .46-sag
    node.real.monitor().position[0] = actual.position.z
    options = []
    def replay(*args, **kwargs):
        options.append(kwargs)
        actual.position.z = target-sag
        node.real.monitor().position[0] = actual.position.z
        return node.real.monitor()
    node.real.execute_recorded_trajectory = replay
    node.retreat_to_pregrasp('right', pose(target), {}, check_collisions=False)
    assert actual.position.z == pytest.approx(target-sag)
    assert len(commands) == (0 if reverse else 1)
    if reverse:
        assert options == [dict(check_collisions=False, defer_height_shortfall=True,
                                endpoint_pose=node.cartesian_pose(pose(target)))]


def test_large_cumulative_correction_still_stops_after_three_attempts():
    node, actual, commands, _ = plant(goal=.5, sag=.1)
    def improve_slightly(arm, target, stage, **options):
        commands.append(node.pose_message(options['command_target']))
        actual.position.z += .002
    node.linear = improve_slightly
    report = {}
    with pytest.raises(HeightCorrectionError, match='attempt limit'):
        node.correct_height_endpoint('right', pose(.5), 'lift', report)
    assert len(commands) == 3
    assert commands[-1].position.z == pytest.approx(.8)
    assert report['height_corrections']['lift']['attempt'] == 3


def test_stalled_endpoint_never_keeps_raising_the_command():
    node, actual, commands, events = plant()
    node.linear = lambda *args, **kwargs: commands.append(args)
    report = {}
    with pytest.raises(HeightCorrectionError, match='progress'):
        node.correct_height_endpoint('right', pose(.5743888152472258), 'lift', report)
    assert len(commands) == 1
    assert report['height_corrections']['lift']['status'] == 'failed'
    assert events[-1][0] == 'height_correction_failed'


@pytest.mark.parametrize('fault', ['stale', 'contact', 'cancel', 'collision'])
def test_feedback_contact_cancel_and_motion_rejection_propagate_without_retry(fault):
    node, _, commands, events = plant()
    error = ExecutionCanceled('canceled') if fault == 'cancel' else RuntimeError(fault)
    def fail(*args, **kwargs):
        raise error
    if fault in ('stale', 'contact', 'cancel'):
        node.real.wait_for_settle = fail
    else:
        node.linear = fail
    with pytest.raises(type(error)) as caught:
        node.correct_height_endpoint('right', pose(.5743888152472258), 'lift', {})
    assert caught.value is error
    assert commands == []
    assert events[-1][0] == 'height_correction_failed'


@pytest.mark.parametrize('bad', ['xy', 'orientation', 'nonfinite', 'missing_hold', 'overshoot'])
def test_invalid_endpoint_cannot_authorize_vertical_motion(bad):
    node, actual, commands, _ = plant()
    if bad == 'xy':
        actual.position.x += .04
    elif bad == 'orientation':
        actual.orientation.w, actual.orientation.x = 0., 1.
    elif bad == 'nonfinite':
        actual.position.z = float('nan')
    elif bad == 'overshoot':
        actual.position.z = .5743888152472258 + .031
    else:
        node.real.arm_holds.clear()
    with pytest.raises((ValueError, RuntimeError)):
        node.correct_height_endpoint('right', pose(.5743888152472258), 'lift')
    assert commands == []


def test_disabled_correction_never_reads_or_commands_hardware():
    node = object.__new__(CollectionPipeline)
    node.config = {}
    node.correct_height_endpoint('right', pose(.5), 'lift')


@pytest.mark.parametrize('reverse', [False, True])
def test_retreat_accepts_existing_thirty_mm_tolerance_without_height_correction(reverse):
    goal, sent, measured = .5318423315105774, .5419910580592112, .5243888133737824
    node, actual, commands, events = plant(goal=sent, sag=sent-measured)
    node.config['retreat_reverse_approach_enabled'] = reverse
    node._approach_command_arm = 'right'
    node._approach_command_paths = (RecordedTrajectory(ARM_JOINT_NAMES['right'], (
        CommandPoint(0, (sent, 0., 0., 0., .2), (0.,)*5),
        CommandPoint(10**9, (.46, 0., 0., 0., .2), (0.,)*5))),)
    if reverse:
        actual.position.z = .46-(sent-measured)
        node.real.monitor().position[0] = actual.position.z
    def replay(*args, **kwargs):
        actual.position.z = measured
        node.real.monitor().position[0] = measured
        return node.real.monitor()
    node.real.execute_recorded_trajectory = replay
    node.correct_height_endpoint = lambda *args, **kwargs: pytest.fail('Retreat uses original tolerance')
    report = {}
    node.retreat_to_pregrasp('right', pose(goal), report, check_collisions=False)
    assert .005 < abs(actual.position.z-goal) <= .030
    assert 'height_corrections' not in report
    if reverse:
        assert commands == []
        assert report['retreat_route'] == 'reversed_approach'
        assert node._approach_command_paths is None
        assert events[-1][0] == 'retreat_reverse_approach_completed'
    else:
        assert len(commands) == 1
        assert commands[0][1] == 'retreat'


def test_original_retreat_failure_never_starts_correction():
    node, _, commands, _ = plant()
    def fail(*args, **kwargs):
        raise RuntimeError('original motion failed')
    node.linear = fail
    node.correct_height_endpoint = lambda *args, **kwargs: pytest.fail('Must not recover failed motion')
    with pytest.raises(RuntimeError, match='original motion failed'):
        node.retreat_to_pregrasp('right', pose(.5743888152472258), {})
    assert commands == []


def test_collection_validation_checks_correction_configuration_before_startup():
    assert 'lift_height_correction_enabled must be boolean' in configuration_issues(
        {'lift_height_correction_enabled': 'true'})
    assert 'lift_height_tolerance_m must be less than lift_height_m' in configuration_issues(
        {'lift_height_tolerance_m': .05, 'lift_height_m': .05})


@pytest.mark.parametrize('sag', [.008, .010, -.020])
def test_lift_at_minimum_height_or_above_target_needs_no_correction(sag):
    node, _, commands, _ = plant(sag=sag)
    node.correct_height_endpoint('right', pose(.5743888152472258), 'lift')
    assert commands == []
