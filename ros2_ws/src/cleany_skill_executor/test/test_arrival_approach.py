"""Arrival replanning uses final hardware commands; all ROS/motor ports are mocked."""
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from cleany_interfaces.action import SelectReachableGrasp, ValidatePick
from cleany_interfaces.msg import GraspCandidate
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import ArmCollectionConfig, configuration_issues
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES, PoseValue
from cleany_skill_executor.core.hardware_execution import (
    ArmHold, CommandPathValidationError, ExecutionCanceled,
    JointRangeValidationError, PhysicalJointLimit, bounded_path,
)
from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
import cleany_skill_executor.hardware_execution as transport


@pytest.fixture
def command_pipeline(monkeypatch):
    names = ARM_JOINT_NAMES['right']
    measured = (1.9573594853420782, 0., 0., 0., 0.)
    held = (2.003301606306097, 0., 0., 0., 0.)
    state = JointState(name=[*names, 'right_gripper_joint'], position=[*measured, .2])
    values = {
        'hardware_velocity_rad_s': .8, 'hardware_acceleration_rad_s2': 1.2,
        'hardware_path_step_rad': .01, 'hardware_path_simplification_tolerance_rad': .002,
        'hardware_start_tolerance_rad': .005, 'hardware_arrival_tolerance_rad': .1,
        'hardware_start_blend_max_rad': .03, 'hardware_uncommanded_tolerance_rad': .03,
        'hardware_settle_time_sec': .5, 'hardware_command_continuity_max_offset_rad': .06,
        'hardware_tcp_position_tolerance_m': .03,
        'hardware_tcp_orientation_tolerance_deg': 20.,
        'hardware_height_correction_min_duration_sec': .3,
    }
    limits = {name: PhysicalJointLimit(-2., 2.0033789089786476 if name == names[0] else 2.,
        1., 2., 2048., 1., 0., 0., 4095.) for name in state.name}
    node = object.__new__(CollectionPipeline)
    node.config = dict(approach_execution_replanning_enabled=True,
        approach_execution_replanning_attempts=2, approach_collision_check_enabled=False,
        cartesian_step_m=.005, cartesian_tolerance_m=.005)
    plans, sent, trace = [], [], []
    node.event = lambda event, **fields: trace.append((event, fields))
    node.get_logger = lambda: NS(info=lambda _: None, warning=lambda _: None, error=lambda _: None)
    def pose(_):
        result = Pose()
        result.position.z = state.position[1]
        result.orientation.w = 1.
        return result
    node.pose = pose
    # A simple redundant FK model isolates the joint-range/command-continuity bug.
    node.motion = NS(set_current_state=lambda _: None, state_is_valid=lambda *_: True,
        forward_kinematics=lambda arm, q: PoseValue((0., 0., q.positions[1]), (0., 0., 0., 1.)))
    node.cartesian = object()
    def call(client, request, *args):
        if client is node.cartesian:
            start = tuple(request.start_state.joint_state.position[:5])
            plans.append(start)
            end = (1.8, request.waypoints[-1].position.z, 0., 0., 0.)
            middle = (2., (start[1] + end[1])/2, 0., 0., 0.)
            path = JointTrajectory(joint_names=list(names), points=[
                JointTrajectoryPoint(positions=list(q)) for q in (start, middle, end)])
            return NS(fraction=1., error_code=NS(val=1), solution=NS(joint_trajectory=path))
        return NS(valid=True, contacts=[])
    node.call = call
    def run_action(client, goal, timeout, **options):
        sent.append(deepcopy(goal.trajectory))
        state.position = [*goal.trajectory.points[-1].positions, .2]
        return NS(error_code=0, SUCCESSFUL=0)
    node.run_action = run_action
    node.create_motion_client = lambda *_: NS(destroy=lambda: None)
    real = object.__new__(transport.HardwareExecution)
    node.real, real.node = real, node
    real.arm_holds = {'right': ArmHold(held, held)}
    real.last_execution_metrics, real.uncommanded = {}, None
    real._sensor_scene_freeze_depth, real.motion_submissions = 0, 0
    real.verify, real.monitor = lambda _: limits, lambda: deepcopy(state)
    real.value, real.positive_integer = values.__getitem__, lambda _: 1
    real.validity = object()
    real.fail_stop = lambda: pytest.fail('Unexpected physical stop in plan-only rejection')
    real.verify_arm_endpoint = lambda arm, target, limits, actual, **options: (actual, {})
    @contextmanager
    def frozen():
        yield
    real.frozen_sensor_scene = frozen
    node.execute_trajectory = lambda *args, **options: PregraspPipeline.execute_trajectory(node, *args, **options)
    monkeypatch.setattr(transport.time, 'sleep', lambda _: None)
    monkeypatch.setattr(transport, 'ActionClient', lambda *_: NS(destroy=lambda: None))
    target = Pose()
    target.position.z, target.orientation.w = .08, 1.
    selected = NS(selected_arm='right', selected_candidate_index=0, fk_grasp_pose=target,
        grasp_joint_state=JointState(name=list(names), position=[1.8, .08, 0., 0., 0.]))
    return node, selected, state, limits, plans, sent, trace


@pytest.mark.parametrize('check_collisions', [False, True])
def test_approach_plans_from_measured_tcp_without_shifted_joint_limits(
        command_pipeline, check_collisions):
    node, selected, state, limits, plans, sent, trace = command_pipeline
    node.config['approach_collision_check_enabled'] = check_collisions
    held_yaw, measured_yaw = node.real.arm_holds['right'].command[0], state.position[0]
    report = {}
    node.approach_target(selected, report)
    assert [start[0] for start in plans] == [measured_yaw]
    assert len(sent) == 1 and node.real.motion_submissions == 1
    assert sent[0].points[0].positions[0] == held_yaw
    assert tuple(sent[0].points[-1].positions) == pytest.approx((1.8, .08, 0., 0., 0.))
    assert max(point.positions[0] for point in sent[0].points) <= limits[state.name[0]].upper
    attempts = report['approach_execution_replanning']
    assert len(attempts) == 1
    assert attempts[0]['start'] == 'measured' and attempts[0]['outcome'] == 'executed'
    origin = next(fields for event, fields in trace if event == 'approach_measured_start')
    assert origin['measured_joint_rad'][0] == measured_yaw
    assert node.real.last_execution_metrics['collision_checks_enabled'] is check_collisions


@pytest.mark.parametrize('explicit_validation', [False, True])
def test_explicit_support_conversion_preserves_the_measured_physical_destination(
        command_pipeline, explicit_validation):
    node, selected, state, _, plans, sent, _ = command_pipeline
    state.position = [1., 0., 0., 0., 0., .2]
    node.real.arm_holds['right'] = ArmHold((1., 0., 0., 0., 0.), (1., .02, 0., 0., 0.))
    action = node.run_action
    def loaded_action(*args, **options):
        result = action(*args, **options)
        state.position[1] -= .02
        return result
    node.run_action = loaded_action
    report = {}
    node.approach_target(selected, report)
    attempts = report['approach_execution_replanning']
    assert attempts[0]['outcome'] == 'executed'
    assert len(sent) == 1 and len(plans) == 2
    assert plans[0][1] == 0. and plans[1][1] == .02
    assert sent[0].points[0].positions[1] == .02
    assert sent[0].points[-1].positions[1] == pytest.approx(.10)
    assert node.pose('right').position.z == pytest.approx(.08)


@pytest.mark.parametrize('offset', [.0119081533721877, -.0119081533721877])
@pytest.mark.parametrize('check_collisions', [False, True])
@pytest.mark.parametrize('stage', ['return_to_pickup'])
def test_next_cartesian_movement_plans_and_dispatches_from_measured_state_without_old_hold_bias(
        command_pipeline, offset, check_collisions, stage):
    node, selected, state, _, plans, sent, trace = command_pipeline
    measured = (1., 0., 0., 0., 0.)
    state.position = [*measured, .2]
    # Reproduce the 65a30167 failure's 11.908 mm TCP gap, with both an old
    # entry residual and a nonzero hold correction in the pickup fallback.
    old_hold = ArmHold((1., -.02, 0., 0., 0.), (1., offset, 0., 0., 0.))
    node.real.arm_holds['right'] = old_hold
    node.linear('right', selected.fk_grasp_pose, stage,
                check_orientation=False, check_collisions=check_collisions)
    assert plans == [measured]
    assert len(sent) == 1
    assert tuple(sent[0].points[0].positions) == measured
    assert tuple(sent[0].points[0].velocities) == (0.,)*5
    assert tuple(sent[0].points[-1].positions) == (1.8, .08, 0., 0., 0.)
    assert node.real.arm_holds['right'].command == (1.8, .08, 0., 0., 0.)
    assert node.real.last_execution_metrics['start_from_measured'] is True
    assert node.real.last_execution_metrics['collision_checks_enabled'] is check_collisions
    event = next(fields for event, fields in trace if event == stage+'_measured_start')
    assert event['measured_joint_rad'] == list(measured)
    assert event['measured_tcp_position_m'] == [0., 0., 0.]


@pytest.mark.parametrize('stage', ['approach', 'lift', 'retreat_after_release'])
@pytest.mark.parametrize('sag', [.03097461591006146, .075])
@pytest.mark.parametrize('check_collisions', [False, True])
def test_physical_planning_uses_measured_tcp_and_converts_support_only_once(
        command_pipeline, stage, sag, check_collisions):
    from cleany_skill_executor.hardware_execution import HardwareExecution
    node, selected, state, _, plans, sent, trace = command_pipeline
    node.config['lift_height_correction_enabled'] = True
    measured = (1., .2, 0., 0., 0.)
    command = (1., .2+sag, 0., 0., 0.)
    state.position = [*measured, .2]
    # An unrelated nominal/command offset must never add a second support bias.
    node.real.arm_holds['right'] = ArmHold((1., .19, 0., 0., 0.), command)
    node.real.wait_for_settle = lambda _: node.real.monitor()
    node.real.verify_arm_endpoint = lambda *args, **options: HardwareExecution.verify_arm_endpoint(
        node.real, *args, **options)
    action = node.run_action
    def loaded_action(*args, **options):
        result = action(*args, **options)
        state.position[1] -= sag
        return result
    node.run_action = loaded_action
    target = deepcopy(selected.fk_grasp_pose)
    target.position.z = .3
    node.linear('right', target, stage, check_orientation=False, check_collisions=check_collisions)
    assert plans == [measured, command]  # Physical planning, then explicit motor conversion.
    assert len(sent) == 1 and tuple(sent[0].points[0].positions) == command
    assert tuple(sent[0].points[0].velocities) == (0.,)*5
    # No downward command reset may unload support before the upward motion.
    assert min(point.positions[1] for point in sent[0].points) >= command[1]-1e-9
    assert sent[0].points[-1].positions[1] == pytest.approx(.3+sag)
    assert node.pose('right').position.z == pytest.approx(.3)
    assert target.position.z == .3
    metrics = node.real.last_execution_metrics
    assert metrics['endpoint_target_source'] == 'physical_goal'
    assert metrics['final_tcp_position_error_m'] < 1e-9
    support = next(fields for event, fields in trace if event == 'cartesian_command_support')
    assert support['measured_start_position_m'] == (0., 0., .2)
    assert support['physical_target_position_m'] == (0., 0., .3)
    assert support['command_target_position_m'][2] == pytest.approx(.3+sag)


@pytest.mark.parametrize('stage', ['return_to_pickup'])
def test_measured_start_stays_measured_when_small_start_drift_refreshes_validation(command_pipeline, stage):
    node, selected, state, _, plans, sent, _ = command_pipeline
    state.position = [1., 0., 0., 0., 0., .2]
    node.real.arm_holds['right'] = ArmHold((1.5, .02, 0., 0., 0.), (1.5, .02, 0., 0., 0.))
    call, changed = node.call, []
    def drifting_call(client, request, *args):
        if client is node.real.validity and not changed:
            changed.append(True)
            state.position[0] += .006
        return call(client, request, *args)
    node.call = drifting_call
    node.linear('right', selected.fk_grasp_pose, stage, check_orientation=False)
    assert changed and plans == [(1., 0., 0., 0., 0.)]
    assert len(sent) == 1
    assert tuple(sent[0].points[0].positions) == (1.006, 0., 0., 0., 0.)
    assert node.real.last_execution_metrics['start_from_measured'] is True


def test_release_retreat_keeps_support_after_small_drift_revalidation(command_pipeline):
    node, selected, state, _, plans, sent, _ = command_pipeline
    state.position = [1., 0., 0., 0., 0., .2]
    command = (1., .02, 0., 0., 0.)
    node.real.arm_holds['right'] = ArmHold(command, command)
    call, changed = node.call, []
    def drifting_call(client, request, *args):
        if client is node.real.validity and not changed:
            changed.append(True)
            state.position[0] += .006
        return call(client, request, *args)
    node.call = drifting_call
    node.linear('right', selected.fk_grasp_pose, 'retreat_after_release', check_orientation=False)
    assert changed and plans == [(1., 0., 0., 0., 0.), command]
    assert len(sent) == 1 and tuple(sent[0].points[0].positions) == command
    assert sent[0].points[-1].positions[1] == pytest.approx(.10)
    assert node.real.last_execution_metrics['start_from_measured'] is False


def test_release_retreat_rejects_a_departed_hold_before_dispatch(command_pipeline):
    node, selected, state, _, _, sent, _ = command_pipeline
    state.position = [1., 0., 0., 0., 0., .2]
    node.real.arm_holds['right'] = ArmHold((1.5, .02, 0., 0., 0.), (1.5, .02, 0., 0., 0.))
    with pytest.raises(RuntimeError, match='departed too far'):
        node.linear('right', selected.fk_grasp_pose, 'retreat_after_release', check_orientation=False)
    assert not sent


@pytest.mark.parametrize('stage', ['retreat_after_release', 'return_to_pickup'])
def test_measured_cartesian_stage_cannot_be_requested_from_held_command(command_pipeline, stage):
    node, selected, _, _, plans, sent, _ = command_pipeline
    from cleany_skill_executor.core.cartesian import CartesianPose
    with pytest.raises(ValueError, match='restricted to lift height correction'):
        node.linear('right', selected.fk_grasp_pose, stage,
                    command_target=CartesianPose((0.,0.,.1),(0.,0.,0.,1.)))
    assert not plans and not sent


@pytest.mark.parametrize('sag', [.03097461591006146, .075])
@pytest.mark.parametrize('arrival_error', [0., .04])
@pytest.mark.parametrize('check_collisions', [False, True])
def test_recorded_return_transport_verifies_measured_tcp_against_physical_destination(
        command_pipeline, sag, arrival_error, check_collisions):
    from cleany_skill_executor.core.cartesian import CartesianPose
    from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory
    from cleany_skill_executor.hardware_execution import HardwareExecution

    node, _, state, _, _, sent, _ = command_pipeline
    names = ARM_JOINT_NAMES['right']
    start, end = (1., .58+sag, 0., 0., 0.), (1., .48+sag, 0., 0., 0.)
    node.real.arm_holds['right'] = ArmHold(start, start)
    state.position = [1., .58, 0., 0., 0., .2]
    node.real.wait_for_settle = lambda _: node.real.monitor()
    node.real.verify_arm_endpoint = lambda *args, **options: HardwareExecution.verify_arm_endpoint(
        node.real, *args, **options)
    action, stopped = node.run_action, []
    def loaded_action(*args, **options):
        result = action(*args, **options)
        state.position[1] += arrival_error-sag
        return result
    node.run_action = loaded_action
    node.real.fail_stop = lambda: stopped.append(True)
    path = RecordedTrajectory(names, (CommandPoint(0, start, (0.,)*5),
                                       CommandPoint(2*10**9, end, (0.,)*5)))
    goal = CartesianPose((0., 0., .48), (0., 0., 0., 1.))
    def execute():
        node.real.execute_return_segment('right', path, 0, 2*10**9,
            check_collisions=check_collisions, endpoint_pose=goal)
    if arrival_error:
        with pytest.raises(RuntimeError, match='Physical TCP endpoint outside tolerance'):
            execute()
        assert stopped
        metrics = node.real.last_execution_metrics
    else:
        execute()
        assert not stopped
        metrics = node.real.last_execution_metrics
        assert metrics['endpoint_within_tolerance']
        assert not metrics['endpoint_height_shortfall_deferred']
    assert len(sent) == 1
    assert tuple(sent[0].points[0].positions) == start
    assert tuple(sent[0].points[-1].positions) == end
    assert metrics['endpoint_target_source'] == 'physical_goal'
    assert metrics['command_tcp_position_m'][2] == pytest.approx(.48+sag)
    assert metrics['target_tcp_position_m'] == list(goal.position)
    assert metrics['final_tcp_position_error_m'] == pytest.approx(arrival_error)


@pytest.mark.parametrize('options', [
    {'held_command': (0.,)*5}, {'recorded_trajectory': object()},
    {'start_from_measured': 'true'}, {'start_from_measured': 1},
])
def test_measured_start_rejects_conflicting_or_malformed_commands_before_motion(command_pipeline, options):
    node, _, _, _, plans, sent, _ = command_pipeline
    with pytest.raises(ValueError, match='Measured arm start|must be boolean'):
        node.real._execute('right', None, 'arm', **{'start_from_measured': True, **options})
    assert not plans and not sent


def test_range_failure_preserves_joint_sample_and_encoder_bounds():
    limit = PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 1900., 2200.)
    with pytest.raises(JointRangeValidationError) as rejected:
        bounded_path(('yaw',), [(0.,), (1.,)], {'yaw': limit}, velocity=.8, acceleration=1.2)
    error = rejected.value
    assert error.joint_name == 'yaw' and error.sample_index > 0
    assert error.lower <= error.value <= error.upper and error.raw > error.raw_max
    assert not error.motion_started and 'joint=yaw' in str(error)


def test_reverse_retreat_replays_final_sent_path_without_new_ik_or_double_bias(command_pipeline):
    node, selected, state, _, plans, sent, trace = command_pipeline
    node.config['retreat_reverse_approach_enabled'] = True
    pregrasp = node.pose('right')
    with node.record_approach_commands('right'):
        node.approach_target(selected, {})
    forward = deepcopy(sent[0])
    assert len(plans) == 1
    assert len(node._approach_command_paths) == 1
    # Closing changed gripper feedback; roll also settled by a few ticks.
    state.position[-2], state.position[-1] = .03, -.3
    state.position[0] += .01  # validate the measured-to-held entry separately
    measured_yaw = state.position[0]
    node.real.last_execution_metrics = {'controller': 'gripper'}
    validity_states, call = [], node.call
    def checking(client, request, *args):
        if client is node.real.validity:
            assert request.robot_state.is_diff  # retain the attached object
            validity_states.append(deepcopy(request.robot_state.joint_state))
        return call(client, request, *args)
    node.call = checking
    report = {}
    node.retreat_to_pregrasp('right', pregrasp, report)
    assert len(plans) == 1 and len(sent) == 2
    backward = sent[1]
    end_ns = forward.points[-1].time_from_start.sec*10**9+forward.points[-1].time_from_start.nanosec
    for first, last in zip(forward.points, reversed(backward.points), strict=True):
        assert first.positions[:4] == last.positions[:4]
        assert tuple(-v for v in first.velocities[:4]) == tuple(last.velocities[:4])
        assert last.positions[4] == .03 and last.velocities[4] == 0.
        time_ns = lambda p: p.time_from_start.sec*10**9+p.time_from_start.nanosec
        assert time_ns(first)+time_ns(last) == end_ns
    assert validity_states and all(s.position[-1] == -.3 for s in validity_states)
    assert validity_states[0].position[0] == measured_yaw
    assert node.real.last_execution_metrics['collision_checks_enabled'] is True
    assert node.real.arm_holds['right'].command == tuple(backward.points[-1].positions)
    assert report['retreat_route'] == 'reversed_approach'
    assert node._approach_command_paths is None
    with pytest.raises(CommandPathValidationError, match='no recorded approach'):
        node.retreat_to_pregrasp('right', pregrasp)
    assert len(sent) == 2


@pytest.mark.parametrize('check_collisions', [True, False])
def test_reverse_retreat_collision_policy_covers_entry_and_recorded_path(
        command_pipeline, check_collisions):
    from cleany_skill_executor.core.hardware_execution import CollisionValidationError
    node, selected, state, _, _, sent, _ = command_pipeline
    node.config['retreat_reverse_approach_enabled'] = True
    pregrasp = node.pose('right')
    with node.record_approach_commands('right'):
        node.approach_target(selected, {})
    state.position[-1] = -.3
    state.position[0] += .01  # Require a measured-to-held bridge as well.
    checks = []
    def blocked(*args):
        checks.append(True)
        return NS(valid=False, contacts=[])
    node.call = blocked
    if check_collisions:
        with pytest.raises(CollisionValidationError, match='phase=hold_bridge'):
            node.retreat_to_pregrasp('right', pregrasp, check_collisions=True)
        assert checks and len(sent) == 1
    else:
        node.retreat_to_pregrasp('right', pregrasp, check_collisions=False)
        assert not checks and len(sent) == 2
        assert node.real.last_execution_metrics['collision_checks_enabled'] is False
    assert node._approach_command_paths is None


def test_unchecked_cartesian_lift_skips_planner_and_both_sample_checks_without_leaking_policy(
        command_pipeline):
    from cleany_skill_executor.core.hardware_execution import CollisionValidationError
    node, selected, _, _, _, sent, _ = command_pipeline
    node.approach_target(selected, {})
    request_flags, state_checks, call = [], [], node.call
    def planning(client, request, *args):
        assert client is node.cartesian  # No physical state validity service calls.
        request_flags.append(request.avoid_collisions)
        return call(client, request, *args)
    def invalid(*args):
        state_checks.append(True)
        return False
    node.call, node.motion.state_is_valid = planning, invalid
    target = node.pose('right')
    target.position.z += .05
    node.linear('right', target, 'lift', check_collisions=False)
    assert request_flags == [False] and not state_checks and len(sent) == 2
    assert node.real.last_execution_metrics['collision_checks_enabled'] is False
    target.position.z += .05
    with pytest.raises(CollisionValidationError):
        node.linear('right', target, 'retreat_after_release')
    assert request_flags == [False, True] and state_checks and len(sent) == 2


def test_failed_approach_clears_history_and_cannot_reverse(command_pipeline):
    node, _, _, _, _, sent, _ = command_pipeline
    node.config['retreat_reverse_approach_enabled'] = True
    node._approach_command_paths = ('old execution',)
    with pytest.raises(ExecutionCanceled):
        with node.record_approach_commands('right'):
            raise ExecutionCanceled('canceled')
    assert node._approach_command_paths is None
    assert node.real._arm_command_recording is None and not sent


def test_incomplete_or_interrupted_history_cannot_start_grasp(command_pipeline):
    node, selected, _, _, _, _, _ = command_pipeline
    node.config['retreat_reverse_approach_enabled'] = True
    with pytest.raises(CommandPathValidationError, match='complete command history'):
        with node.record_approach_commands('right'):
            pass
    with pytest.raises(CommandPathValidationError, match='complete command history'):
        with node.record_approach_commands('right'):
            node.approach_target(selected, {})
            node.real._arm_command_recording.interrupted = True
    assert node._approach_command_paths is None


def test_multiple_completed_approach_goals_reverse_in_completion_order(command_pipeline):
    node, selected, state, _, plans, sent, _ = command_pipeline
    node.config['retreat_reverse_approach_enabled'] = True
    pregrasp = node.pose('right')
    with node.record_approach_commands('right'):
        node.approach_target(selected, {})
        second = deepcopy(selected)
        second.fk_grasp_pose.position.z = .16
        second.grasp_joint_state.position[-1] = .04
        node.approach_target(second, {})
    assert len(sent) == 2 and len(node._approach_command_paths) == 2
    forward = deepcopy(sent)
    state.position[-2], state.position[-1] = .05, -.3
    requests = len(plans)
    node.retreat_to_pregrasp('right', pregrasp)
    assert len(sent) == 4 and len(plans) == requests
    for original, backward in zip(reversed(forward), sent[2:], strict=True):
        for first, last in zip(original.points, reversed(backward.points), strict=True):
            assert first.positions[:4] == last.positions[:4]
            assert last.positions[4] == .05 and last.velocities[4] == 0.


@pytest.mark.parametrize('problem', ['wrong_arm', 'departed_hold', 'changed_limits'])
@pytest.mark.parametrize('check_collisions', [True, False])
def test_reverse_retreat_rejects_wrong_arm_start_drift_and_new_joint_limits(
        command_pipeline, problem, check_collisions):
    node, selected, state, limits, _, sent, _ = command_pipeline
    node.config['retreat_reverse_approach_enabled'] = True
    pregrasp = node.pose('right')
    with node.record_approach_commands('right'):
        node.approach_target(selected, {})
    arm = 'right'
    error = RuntimeError
    if problem == 'wrong_arm':
        arm, error = 'left', CommandPathValidationError
    elif problem == 'departed_hold':
        state.position[0] += .2
    else:
        limits[state.name[0]] = replace(limits[state.name[0]], upper=1.9)
        error = JointRangeValidationError
    with pytest.raises(error):
        node.retreat_to_pregrasp(arm, pregrasp, check_collisions=check_collisions)
    assert len(sent) == 1


def test_interrupted_replay_never_rebuilds_or_resumes_a_different_path():
    from cleany_skill_executor.core.hardware_execution import CommunicationRecovered
    calls = []
    def interrupted(*args, **kwargs):
        calls.append('execute')
        error = CommunicationRecovered(1)
        error.cancellation_confirmed = True
        raise error
    fake = NS(_execute=interrupted, positive_integer=lambda _: 1,
              fail_stop=lambda: calls.append('stop'))
    with pytest.raises(RuntimeError, match='cannot resume its exact path'):
        transport.HardwareExecution._execute_with_recovery(
            fake, 'right', object(), 'arm', recorded_trajectory=object())
    assert calls == ['execute', 'stop']


def candidate(number):
    result = GraspCandidate(snapshot_id='approved-snapshot', object_id=1, score=float(number))
    result.header.frame_id = 'base_link'
    return result


@pytest.fixture
def reselection_pipeline():
    candidates = [candidate(index) for index in range(3)]
    selected = ValidatePick.Result(success=True, selected_arm='right', selected_candidate_index=0,
        selected_candidate=candidates[0], candidates=candidates, reference_id='pinned-reference')
    node = object.__new__(CollectionPipeline)
    node.config = {'approach_candidate_reselection_attempts': 2}
    node._selection = object()
    node.arm_config = ArmCollectionConfig(-.3, 1., .1, (.1,)*5, {})
    node.wrist_request = None
    trace, current_pose = [], Pose()
    node.real = NS(monitor=lambda: JointState(), motion_submissions=0)
    node.scene = NS(begin=lambda *_: trace.append('scene.begin'),
        allow_contacts_for=lambda *_: trace.append('scene.allow'), restore=lambda: trace.append('scene.restore'))
    node.release_grasp_scene_freeze = lambda: trace.append('unfreeze')
    node.pose = lambda _: deepcopy(current_pose)
    node.event = lambda event, **fields: trace.append(event)
    def select(client, goal, timeout, **options):
        assert goal.required_arm == 'right'
        trace.append(('select', [entry.score for entry in goal.candidates]))
        return SelectReachableGrasp.Result(success=True, selected_arm='right', selected_candidate_index=0,
            selected_candidate=goal.candidates[0])
    node.run_action = select
    node.before_execution = lambda _: setattr(node, 'arm_config', replace(node.arm_config, home=(9.,)*5))
    def prepare(result):
        trace.append(('pregrasp', result.selected_candidate.score))
        assert node.arm_config.home == (.1,)*5
        current_pose.position.z = .5
        node.real.motion_submissions += 1
    return node, selected, trace, prepare


def rejected_range():
    return JointRangeValidationError(2.1, -2., 2., 3400, 0., 4095., joint_name='yaw', sample_index=3)


def test_reselection_excludes_failed_candidate_and_moves_to_its_new_pregrasp(reselection_pipeline):
    node, selected, trace, prepare = reselection_pipeline
    def approach(result, report):
        trace.append(('approach', result.selected_candidate.score))
        if result.selected_candidate.score == 0.:
            raise rejected_range()
    node.approach_target = approach
    report = {}
    accepted, pregrasp = node.approach_with_candidate_reselection(
        selected, report, object_id='payload', prepare_pregrasp=prepare)
    assert accepted.selected_candidate.score == 1. and accepted.reference_id == 'pinned-reference'
    assert pregrasp.position.z == .5 and report['approach_candidate_reselections'] == 1
    assert report['approach_original_candidate_index'] == 1
    assert trace.index('scene.restore') < trace.index(('select', [1., 2.]))
    assert trace.index(('select', [1., 2.])) < trace.index(('pregrasp', 1.)) < trace.index(('approach', 1.))
    assert [entry.score for entry in selected.candidates] == [0., 1., 2.]


@pytest.mark.parametrize('kind', ['cancel', 'stale', 'submitted'])
def test_no_reselection_after_cancel_stale_feedback_or_submitted_motion(reselection_pipeline, kind):
    node, selected, trace, prepare = reselection_pipeline
    error = (ExecutionCanceled('cancel') if kind == 'cancel' else
             RuntimeError('stale joints') if kind == 'stale' else rejected_range())
    def approach(*_):
        if kind == 'submitted':
            node.real.motion_submissions += 1
        raise error
    node.approach_target = approach
    with pytest.raises(type(error)):
        node.approach_with_candidate_reselection(selected, {}, object_id='payload', prepare_pregrasp=prepare)
    assert not any(isinstance(entry, tuple) and entry[0] in ('select', 'pregrasp') for entry in trace)


def test_reselection_stops_at_the_configured_attempt_budget(reselection_pipeline):
    node, selected, trace, prepare = reselection_pipeline
    node.config['approach_candidate_reselection_attempts'] = 1
    node.approach_target = lambda *_: (_ for _ in ()).throw(rejected_range())
    with pytest.raises(JointRangeValidationError):
        node.approach_with_candidate_reselection(selected, {}, object_id='payload', prepare_pregrasp=prepare)
    assert sum(isinstance(entry, tuple) and entry[0] == 'select' for entry in trace) == 1


@pytest.mark.parametrize('key,value', [
    ('approach_execution_replanning_enabled', 'true'),
    ('retreat_reverse_approach_enabled', 'true'),
    ('approach_execution_replanning_attempts', 0),
    ('approach_execution_replanning_attempts', 3),
    ('approach_candidate_reselection_attempts', -1),
    ('approach_candidate_reselection_attempts', 5),
    ('approach_candidate_selection_timeout_sec', 0.),
    ('approach_candidate_selection_timeout_sec', float('nan')),
    ('pregrasp_motion_enabled', False),
    ('visual_servo_enabled', True),
    ('approach_collision_replanning_enabled', True),
])
def test_arrival_replanning_rejects_invalid_budgets_and_conflicting_modes(key, value):
    root = Path(__file__).resolve().parents[4]
    settings = json.loads((root/'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
    settings[key] = value
    issues = configuration_issues(settings)
    assert any(key in issue for issue in issues)


@pytest.mark.parametrize('changed', ['arm', 'object', 'frame', 'obb'])
def test_reselection_never_changes_the_approved_arm_or_object(reselection_pipeline, changed):
    node, selected, trace, prepare = reselection_pipeline
    result = SelectReachableGrasp.Result(success=True, selected_arm='right', selected_candidate_index=0,
        selected_candidate=deepcopy(selected.candidates[1]))
    if changed == 'arm':
        result.selected_arm = 'left'
    elif changed == 'object':
        result.selected_candidate.object_id = 2
    elif changed == 'frame':
        result.selected_candidate.header.frame_id = 'other_frame'
    else:
        result.selected_candidate.target_object.obb_size.x = 1.
    node.run_action = lambda *_, **__: result
    with pytest.raises(RuntimeError, match='approved object, frame or arm'):
        node.reselect_approach_candidate(selected, selected.candidates[1:])


def test_an_unsubmitted_alternative_pregrasp_can_be_rejected_for_the_next_candidate(reselection_pipeline):
    node, selected, trace, prepare = reselection_pipeline
    events = []
    node.event = lambda name, **fields: events.append((name, fields))
    node.approach_target = lambda result, _: (
        (_ for _ in ()).throw(rejected_range()) if result.selected_candidate.score == 0. else None)
    def reject_first_alternative(result):
        if result.selected_candidate.score == 1.:
            raise rejected_range()
        prepare(result)
    accepted, _ = node.approach_with_candidate_reselection(
        selected, {}, object_id='payload', prepare_pregrasp=reject_first_alternative)
    assert accepted.selected_candidate.score == 2.
    assert ('select', [2.]) in trace and ('pregrasp', 2.) in trace
    retries = [fields for name, fields in events if name == 'approach_candidate_reselection']
    assert retries[0]['remaining_original_candidate_indices'] == [1, 2]
    assert retries[1]['remaining_original_candidate_indices'] == [2]


def test_a_submitted_alternative_pregrasp_failure_cannot_select_again(reselection_pipeline):
    node, selected, trace, _ = reselection_pipeline
    node.approach_target = lambda *_: (_ for _ in ()).throw(rejected_range())
    def moving_pregrasp(_):
        node.real.motion_submissions += 1
        raise rejected_range()
    with pytest.raises(JointRangeValidationError):
        node.approach_with_candidate_reselection(selected, {}, object_id='payload', prepare_pregrasp=moving_pregrasp)
    assert sum(isinstance(entry, tuple) and entry[0] == 'select' for entry in trace) == 1
