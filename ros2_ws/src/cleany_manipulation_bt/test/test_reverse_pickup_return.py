"""Loaded return replays completed commands; all motor ports are synthetic."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import PickupReturnPolicy
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.core.hardware_execution import (
    ArmHold, CommandPathValidationError, ExecutionCanceled, PhysicalJointLimit,
)
from cleany_skill_executor.core.recorded_trajectory import (
    CommandPoint, RecordedTrajectory, validate_recorded_trajectory,
)
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_manipulation_bt.backend import OperationError

from test_temporary_return import temporary, raised_temporary  # noqa: F401


@pytest.fixture
def replay(raised_temporary):
    node, context, joints, _, commands = raised_temporary
    arm = context.selected.selected_arm
    names = ARM_JOINT_NAMES[arm]
    sag = .017
    node.config.update(return_to_pick_reverse_enabled=True,
                       return_to_pick_collision_check_enabled=False,
                       lift_height_correction_enabled=True)
    joints.position = [.3, 0., 0., 0., .4]
    base = node.pose(arm)
    def fk(q):
        return NS(position=(.25+q[1], base.position.y, q[0]),
                  orientation_xyzw=(0., 0., 0., 1.))
    def pose(_):
        result = deepcopy(base)
        result.position.x, result.position.y, result.position.z = fk(joints.position).position
        return result
    node.pose = pose
    node.motion = NS(set_current_state=lambda _: None, forward_kinematics=lambda _, q: fk(q.positions))
    values = {'hardware_tcp_position_tolerance_m': .03,
              'hardware_tcp_orientation_tolerance_deg': 20.,
              'hardware_velocity_rad_s': .8, 'hardware_acceleration_rad_s2': 1.2,
              'hardware_path_step_rad': .01}
    limits = {name: PhysicalJointLimit(-3., 3., 2., 4., 2048., 1., 0., 0., 4095.) for name in names}
    node.real.value = values.__getitem__
    node.real.verify = lambda _: limits
    node.real.node = node
    node.real.wait_for_settle = lambda _: joints
    node.real.arm_holds = {arm: ArmHold((.317, 0., 0., 0., .4), (.317, 0., 0., 0., .4))}
    node.real.record_arm_commands = lambda chosen: HardwareExecution.record_arm_commands(node.real, chosen)
    replays = []
    def apply(q, *, replaying=False):
        node.real.arm_holds[arm] = ArmHold(tuple(q), tuple(q))
        joints.position = list(q)
        ascent_sag = getattr(node, '_test_lift_sag_m', sag)
        joints.position[0] -= getattr(node, '_test_return_sag_m', ascent_sag) if replaying else ascent_sag
    def linear(chosen, target, stage, *args, **options):
        assert stage != 'return_to_pickup', 'Reverse return must never request a new Cartesian path'
        commands.append(stage)
        start = node.real.arm_holds[arm].command
        motor = node.pose_message(options['command_target']) if 'command_target' in options else target
        z = (motor.position.z if stage in ('lift', 'lift_height_correction') else
             target.position.z+getattr(node, '_test_lift_sag_m', sag))
        finish = (z, motor.position.x-.25, 0., 0., .4)
        recorded = RecordedTrajectory(names, (CommandPoint(0, start, (0.,)*5),
                                              CommandPoint(2*10**9, finish, (0.,)*5)))
        apply(finish)
        recording = getattr(node.real, '_arm_command_recording', None)
        if recording is not None:
            recording.trajectories.append(recorded)
        return joints
    node.linear = linear
    def execute(chosen, recorded, *, check_collisions, endpoint_pose):
        assert chosen == arm
        assert endpoint_pose == node.cartesian_pose(node.pickup_return_pose)
        assert recorded.points[0].positions == pytest.approx(node.real.arm_holds[arm].command)
        assert recorded.points[0].velocities == pytest.approx((0.,)*5)
        assert recorded.points[-1].velocities == pytest.approx((0.,)*5)
        validate_recorded_trajectory(recorded, limits, velocity=.8, acceleration=1.2, step=.01)
        replays.append((recorded, check_collisions))
        commands.append('return_to_pickup_replay')
        apply(recorded.points[-1].positions, replaying=True)
        return joints
    node.real.execute_recorded_trajectory = execute
    node.real.execute_return_segment = lambda chosen, path, start, end, **options: HardwareExecution.execute_return_segment(
        node.real, chosen, path, start, end, **options)
    node.pregrasp_pose = pose(arm)
    node.pregrasp_pose.position.z += .04
    node.pregrasp_pose.position.x += .01
    node.capture_pickup(arm)
    commands.clear()
    return node, context, joints, commands, replays


@pytest.mark.parametrize('adapter', ['bt', 'legacy'])
@pytest.mark.parametrize('pregrasp', [True, False])
@pytest.mark.parametrize('collisions', [True, False])
def test_loaded_reverse_return_includes_retreat_lift_and_corrections_and_stops_above_pickup(
        replay, adapter, pregrasp, collisions):
    node, context, joints, commands, replays = replay
    arm = context.selected.selected_arm
    node.config.update(pregrasp_motion_enabled=pregrasp, return_to_pick_collision_check_enabled=collisions)
    pickup = node.pose(arm)
    released_at = []
    jaw = node.jaw
    def release(chosen, target, stage, **options):
        if stage in ('release', 'release_at_pickup'):
            released_at.append(node.pose(arm))
        return jaw(chosen, target, stage, **options)
    node.jaw = release
    if adapter == 'bt':
        node.grasp(context)
        node.confirm_grasp(context)
        node.lift(context)
        # A 17mm shortfall is rounded to one 20mm correction, so the recorded
        # departure has lift + correction, and optionally the initial retreat.
        assert len(node._pickup_departure_paths) == (3 if pregrasp else 2)
        node.confirm_held(context)
        node.carry(context)
        node.check_placement(context)
        node.open(context)
        node.confirm_release(context)
        node.return_arm(context)
        report = node.report
    else:
        context.selected.selected_candidate = NS()
        context.selected.fk_grasp_pose = pickup
        context.selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[arm], position=list(joints.position))
        if pregrasp:
            node.linear(arm, node.pregrasp_pose, 'fixture_pregrasp')
        report = {}
        CollectionPipeline.after_pregrasp(node, context.selected, None, report)
        assert report['released'] and report['returned']
    assert len(replays) == len(released_at) == 1
    recorded, checked = replays[0]
    assert checked is collisions
    assert report['return_route'] == 'reversed_departure'
    assert report['return_start_source'] == 'measured'
    assert report['return_collision_checks_enabled'] is collisions
    assert report['return_recorded_trajectories'] == (3 if pregrasp else 2)
    assert report['return_tracking_residual_m'][2] == pytest.approx(.017)
    assert released_at[0].position.z == pytest.approx(pickup.position.z+.02, abs=1e-7)
    assert min(point.positions[0] for point in recorded.points) >= pickup.position.z+.02+.017-1e-7
    assert commands.count('lift_height_correction') == 1
    assert 'return_to_pickup' not in commands
    assert node._pickup_departure_paths is None


@pytest.mark.parametrize('sag', [.03097461591006146, .04338288349589037, .075])
def test_corrected_lift_returns_from_current_measured_tcp_without_command_gap_rejection(replay, sag):
    node, context, joints, commands, replays = replay
    node._test_lift_sag_m = sag
    arm = context.selected.selected_arm
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    node.confirm_held(context)
    # New feedback since ConfirmHeld must define the next movement, including
    # the reverse cut height. The original motor command stays continuous.
    joints.position[0] -= .004
    node._test_return_sag_m = sag+.004
    measured_joints = tuple(joints.position)
    measured = node.pose(arm)
    assert node.real.arm_holds[arm].command[0]-measured.position.z > .03
    node.carry(context)
    node.check_placement(context)
    assert len(replays) == 1
    assert node.report['return_start_source'] == 'measured'
    assert node.report['return_measured_start_joint_rad'] == measured_joints
    assert node.report['return_measured_start_position_m'] == pytest.approx(
        (measured.position.x, measured.position.y, measured.position.z))
    assert node.report['return_tracking_residual_m'][2] == pytest.approx(sag+.004)
    assert node.pose(arm).position.z == pytest.approx(node.pickup_return_pose.position.z, abs=1e-7)
    assert not node.releasing and 'release' not in commands


def test_reverse_start_fk_uses_the_same_settled_joint_snapshot(replay):
    node, context, joints, _, _ = replay
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    snapshot = deepcopy(joints)
    arm = context.selected.selected_arm
    measured = node.pose(arm)
    calls = []
    def settle(_):
        # A separately read state is deliberately different. Return selection
        # must use the settled snapshot, rather than another pose() read.
        calls.append(True)
        if len(calls) == 1:
            joints.position[0] += .01
            return snapshot
        return joints
    node.real.wait_for_settle = settle
    node.carry(context)
    assert len(calls) == 2
    assert node.report['return_measured_start_joint_rad'] == tuple(snapshot.position)
    assert node.report['return_measured_start_position_m'] == pytest.approx(
        (measured.position.x, measured.position.y, measured.position.z))
    assert node.pose(arm).position.z == pytest.approx(node.pickup_return_pose.position.z, abs=1e-7)


@pytest.mark.parametrize('failure', ['missing', 'wrong_arm', 'stale_hold', 'canceled', 'contact',
                                    'far_goal', 'wrong_height', 'execution_fault'])
def test_reverse_return_failure_keeps_payload_and_never_falls_back_or_releases(replay, failure):
    node, context, _, commands, replays = replay
    arm = context.selected.selected_arm
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    if failure == 'missing':
        node._pickup_departure_paths = None
    elif failure == 'wrong_arm':
        node._pickup_departure_arm = 'left' if arm == 'right' else 'right'
    elif failure == 'stale_hold':
        node.real.arm_holds[arm] = ArmHold((0.,)*5, (0.,)*5)
    elif failure in ('canceled', 'contact'):
        error = ExecutionCanceled('canceled') if failure == 'canceled' else RuntimeError('contact lost')
        node.real.monitor = lambda: (_ for _ in ()).throw(error)
    elif failure == 'far_goal':
        node.pickup_return_pose.position.x += .1
    elif failure == 'execution_fault':
        node.real.execute_return_segment = lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError('motor fault'))
    else:
        execute = node.real.execute_return_segment
        def sag_after_return(*args, **kwargs):
            execute(*args, **kwargs)
            result = node.pose(arm)
            result.position.z -= .02
            node.pose = lambda _: result
        node.real.execute_return_segment = sag_after_return
    with pytest.raises((RuntimeError, ValueError, OperationError)):
        node.carry(context)
        node.check_placement(context)
        node.open(context)
    assert node.carrying is not None and not node.releasing
    assert 'release' not in commands and 'return_to_pickup' not in commands
    assert len(replays) == (1 if failure == 'wrong_height' else 0)


def test_interrupted_departure_discards_history(replay):
    node, context, _, _, replays = replay
    arm = context.selected.selected_arm
    node.grasp(context)
    node.confirm_grasp(context)
    linear = node.linear
    def interrupted(chosen, target, stage, **options):
        linear(chosen, target, stage, **options)
        if stage == 'lift':
            raise RuntimeError('interrupted lift')
    node.linear = interrupted
    with pytest.raises(RuntimeError, match='interrupted lift'):
        node.lift(context)
    assert node._pickup_departure_paths is None
    with pytest.raises(CommandPathValidationError, match='no recorded departure'):
        node.reverse_to_pickup(arm, node.pickup_return_pose, PickupReturnPolicy.from_config(node.config))
    assert not replays


def test_return_rechecks_changing_sag_and_continues_only_on_saved_path(replay):
    node, context, _, commands, replays = replay
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    node._test_return_sag_m = .0062  # 17mm ascent residual becomes 6.2mm on descent.
    node.carry(context)
    node.check_placement(context)
    history = node.report['return_height_corrections']
    assert len(replays) == len(history) == 2
    assert history[0]['height_error_m'] == pytest.approx(.0108, abs=1e-7)
    assert history[1]['height_error_m'] == pytest.approx(.0058, abs=1e-7)
    assert replays[1][0].points[0].positions == replays[0][0].points[-1].positions
    assert all(checked is False for _,checked in replays)
    assert 'return_to_pickup' not in commands and 'release' not in commands


@pytest.mark.parametrize('attempts', [0, 2])
def test_return_continuation_budget_stops_without_release(replay, attempts):
    node, context, _, commands, replays = replay
    node.config['return_to_pick_height_correction_max_attempts'] = attempts
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    execute = node.real.execute_return_segment
    def stalled(*args, **kwargs):
        execute(*args, **kwargs)
        measured = node.pose(context.selected.selected_arm)
        measured.position.z = node.pickup_return_pose.position.z+.015
        node.pose = lambda _: measured
    node.real.execute_return_segment = stalled
    with pytest.raises(ValueError, match='raised pickup release height: target='):
        node.carry(context)
    assert len(replays) == attempts+1
    assert 'release' not in commands and node.carrying is not None
