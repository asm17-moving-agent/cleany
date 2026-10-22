"""Exercise the real operation adapter with motion ports replaced, without motor I/O."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose

from cleany_manipulation_bt.backend import ExecutionContext, OperationError
from cleany_manipulation_bt.jetson_operations import JetsonOperations
from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import configuration_issues, load_arm
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.manipulation.models import Error, Goal, Placement


CONFIG = Path(__file__).resolve().parents[4] / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json'


def forbidden(*args, **kwargs):
    pytest.fail('Temporary mode called contact/release/placement evidence')


@pytest.fixture(params=['left', 'right'])
def temporary(request):
    arm = request.param
    node = object.__new__(JetsonOperations)
    node.wrist_cameras = None
    node.config = json.loads(CONFIG.read_text())
    # Preserve coverage of the opt-in zero-offset joint-return compatibility path.
    # The configured raised destination is exercised separately below.
    node.config['return_to_pick_height_offset_m'] = 0.
    node.config['return_to_pick_reverse_enabled'] = False
    node.config['return_to_pick_collision_check_enabled'] = True
    node.config['retreat_after_release_collision_check_enabled'] = True
    # These existing contracts exercise the optional pregrasp/retreat flow.
    node.config['pregrasp_motion_enabled'] = True
    # Existing fixtures mock away all planning ports. Exercise the new arrival
    # recovery explicitly below rather than using incomplete candidate stubs.
    node.config['approach_execution_replanning_enabled'] = False
    node.config['retreat_reverse_approach_enabled'] = False
    # These exact-motion ports omit settling/FK/held commands. Loaded height
    # feedback is covered by test_jetson_operations and test_height_correction_runtime.
    node.config['lift_height_correction_enabled'] = False
    # This fixture replaces the entire motion port and has no URDF/FK model.
    # Actual command guarding is exercised in test_fixed_jaw_pipeline/replay.
    node.config['fixed_jaw_clearance_enabled'] = False
    node.config['direct_grasp_shortcut_enabled'] = False
    node.config['direct_grasp_pose_checks_enabled'] = True
    node.config['temporary_contact_verification'] = 'skip'
    node.arm_config = load_arm(node.config['arms'][arm], require_bins=False)
    node.arm_config = NS(closed=node.arm_config.closed, opened=node.arm_config.opened, home=(0.,)*5)
    node.report, node.carrying, node.releasing = {}, None, False
    joints = NS(name=ARM_JOINT_NAMES[arm], position=[.1, .2, .3, .4, .5])
    pickup = tuple(joints.position)
    tcp, commands = Pose(), []
    tcp.position.z = .3
    tcp.orientation.w = 1.
    node.real = NS(monitor=lambda: joints, value=lambda _: .01, last_execution_metrics={})
    node.measured = lambda: joints
    node.pose = lambda _: deepcopy(tcp)
    node.event = lambda *args, **kw: None
    node.scene = NS(begin=lambda *args: None, allow_contacts_for=lambda *args: None,
                    restore=lambda: None)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100))
    node.verify_evidence = node.close_until_contact = node._outcome = forbidden
    node.wrist_observation_required = lambda: False
    node.release_grasp_scene_freeze = lambda: None
    node._clear_references = lambda _: None
    node._assess_stationary = lambda *args, **kwargs: True
    def linear(_arm, target, stage, *args, **kwargs):
        commands.append(stage)
        tcp.position = deepcopy(target.position)
        tcp.orientation = deepcopy(target.orientation)
    node.linear = linear
    def joint_move(_arm, target, stage):
        commands.append(stage)
        joints.position = list(target)
    node.move_joint_pose = joint_move
    node.move_arm_return = lambda arm, home, closed, **kwargs: joint_move(arm, home, 'return')
    node.jaw = lambda _arm, target, stage, **kwargs: commands.append(stage)
    def attach(*args):
        node.carrying = {'release_confirmed': False}
    node.attach = attach
    node.detach = lambda: setattr(node, 'carrying', None)
    node.pregrasp_pose = deepcopy(tcp)
    node.pregrasp_pose.position.z += .02
    node.capture_pickup(arm)
    context = ExecutionContext(Goal('m', 't', 'real-execution', 'collect_trash', 'real-snapshot', 1, 'return_to_pick'))
    context.selected = NS(selected_arm=arm)
    return node, context, joints, pickup, commands


@pytest.fixture
def raised_temporary(temporary):
    node, context, _, _, commands = temporary
    node.config['return_to_pick_height_offset_m'] = .02
    node.real.value = lambda key: {
        'hardware_tcp_position_tolerance_m': .03,
        'hardware_tcp_orientation_tolerance_deg': 20.,
    }.get(key, .01)
    arm = context.selected.selected_arm
    pickup = node.pose(arm)
    pickup.position.x, pickup.position.y = .25, -.15 if arm == 'right' else .15
    node.linear(arm, pickup, 'fixture_setup')
    node.capture_pickup(arm)
    node.pregrasp_pose = deepcopy(pickup)
    node.pregrasp_pose.position.z += .005  # Below the raised release goal.
    commands.clear()
    return temporary


@pytest.mark.parametrize('adapter', ['bt', 'legacy'])
@pytest.mark.parametrize('pregrasp', [False, True])
def test_raised_return_releases_above_measured_pickup_without_old_joint_target(
        raised_temporary, adapter, pregrasp):
    node, context, joints, pickup_joints, commands = raised_temporary
    arm = context.selected.selected_arm
    pickup = node.pose(arm)
    node.config['pregrasp_motion_enabled'] = pregrasp
    paths, linear, joint_move = [], node.linear, node.move_joint_pose
    def recording(chosen, target, stage, *args, **options):
        paths.append((stage, deepcopy(target), options))
        return linear(chosen, target, stage, *args, **options)
    def joint_recording(chosen, target, stage):
        assert stage != 'return_to_pickup'
        return joint_move(chosen, target, stage)
    node.linear, node.move_joint_pose = recording, joint_recording
    if adapter == 'bt':
        node.grasp(context)
        node.confirm_grasp(context)
        node.lift(context)
        joints.position = [.8] * 5  # Raised arrival must not compare old joint angles.
        node.carry(context)
        node.check_placement(context)
        node.open(context)
        node.confirm_release(context)
        node.return_arm(context)
        report = node.report
        release = 'release'
    else:
        context.selected.selected_candidate = NS()
        context.selected.fk_grasp_pose = pickup
        context.selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[arm], position=list(pickup_joints))
        report = {}
        CollectionPipeline.after_pregrasp(node, context.selected, None, report)
        assert report['released'] and report['returned']
        release = 'release_at_pickup'
    returned = [(target, options) for stage, target, options in paths if stage == 'return_to_pickup']
    assert len(returned) == 1
    target, options = returned[0]
    assert target.position.x == pickup.position.x and target.position.y == pickup.position.y
    assert target.position.z == pytest.approx(pickup.position.z + .02)
    assert target.orientation == pickup.orientation
    assert options == dict(validate_command_path=True, check_collisions=True)
    assert 'lower_to_pickup' not in commands and 'lower_to_pregrasp' not in commands
    assert commands.index('return_to_pickup') < commands.index(release) < commands.index('return')
    retreats = [point for stage, point, _ in paths if stage == 'retreat_after_release']
    if pregrasp:
        assert not retreats  # Already at the raised retreat; no zero-length Cartesian request.
    else:
        assert len(retreats) == 1 and retreats[0].position.z >= target.position.z
    assert report['destination_position_m'] == pytest.approx((pickup.position.x, pickup.position.y, pickup.position.z+.02))
    assert 'destination_joint_rad' not in report
    assert report['placement_state'] == 'ASSUMED'


@pytest.mark.parametrize('error_m,accepted', [(-.020, False), (-.0101, False), (-.010, True),
                                             (.010, True), (.0101, False)])
def test_raised_release_checks_fresh_height_even_inside_general_pose_tolerance(
        raised_temporary, error_m, accepted):
    node, context, _, _, commands = raised_temporary
    node.carry(context)
    actual = node.pose(context.selected.selected_arm)
    actual.position.z += error_m
    node.pose = lambda _: deepcopy(actual)
    if accepted:
        node.check_placement(context)
    else:
        with pytest.raises(OperationError, match='raised pickup release height') as failure:
            node.check_placement(context)
        assert failure.value.error == Error.DESTINATION_UNAVAILABLE
    assert 'release' not in commands


@pytest.mark.parametrize('invalid', ['missing', 'wrong_arm', 'nonfinite', 'xy', 'orientation', 'hardware_fault'])
def test_raised_release_keeps_destination_pose_and_hardware_guards(raised_temporary, invalid):
    node, context, _, _, _ = raised_temporary
    node.carry(context)
    actual = node.pose(context.selected.selected_arm)
    if invalid == 'missing':
        node.pickup_return_pose = None
    elif invalid == 'wrong_arm':
        node.pickup_arm = 'right' if context.selected.selected_arm == 'left' else 'left'
    elif invalid == 'nonfinite':
        actual.position.z = float('nan')
    elif invalid == 'xy':
        actual.position.x += .031
    elif invalid == 'orientation':
        actual.orientation.z, actual.orientation.w = 1., 0.
    else:
        node.real.monitor = lambda: (_ for _ in ()).throw(RuntimeError('hardware fault'))
    node.pose = lambda _: deepcopy(actual)
    with pytest.raises(RuntimeError if invalid == 'hardware_fault' else OperationError):
        node.check_placement(context)


@pytest.mark.parametrize('adapter', ['bt', 'legacy'])
@pytest.mark.parametrize('failure', ['collision', 'low_arrival'])
def test_raised_return_failure_never_opens_gripper(raised_temporary, adapter, failure):
    node, context, _, pickup_joints, commands = raised_temporary
    linear = node.linear
    def failing_return(arm, target, stage, *args, **options):
        if stage == 'return_to_pickup':
            assert options['check_collisions']
            if failure == 'collision':
                raise RuntimeError('return collision')
            target = deepcopy(target)
            target.position.z -= .02
        return linear(arm, target, stage, *args, **options)
    node.linear = failing_return
    with pytest.raises((RuntimeError, ValueError)):
        if adapter == 'bt':
            node.grasp(context)
            node.confirm_grasp(context)
            node.lift(context)
            node.carry(context)
            node.check_placement(context)
            node.open(context)
        else:
            context.selected.selected_candidate = NS()
            context.selected.fk_grasp_pose = node.pose(context.selected.selected_arm)
            context.selected.grasp_joint_state = NS(
                name=ARM_JOINT_NAMES[context.selected.selected_arm], position=list(pickup_joints))
            CollectionPipeline.after_pregrasp(node, context.selected, None, {})
    assert node.carrying is not None and not node.releasing
    assert 'release' not in commands and 'release_at_pickup' not in commands


def test_raised_retreat_does_not_descend_after_payload_release(raised_temporary):
    node, context, _, _, _ = raised_temporary
    arm = context.selected.selected_arm
    node.carry(context)
    actual = node.pose(arm)
    actual.position.z += .007  # Unloading raises the measured TCP above the goal.
    node.linear(arm, actual, 'unloaded_feedback')
    pregrasp = deepcopy(node.pregrasp_pose)
    pregrasp.position.x += .03
    targets = []
    node.linear = lambda chosen, target, stage, **options: targets.append(target)
    node.pickup_release_retreat(arm, pregrasp)
    assert len(targets) == 1 and targets[0].position.z == actual.position.z


def test_return_release_uses_measured_pickup_for_either_arm_and_skips_evidence(temporary):
    node, context, joints, pickup, commands = temporary
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    node.confirm_held(context)
    joints.position = [.8]*5  # Represents another measured posture after the lift.
    node.carry(context)
    assert tuple(joints.position) == pickup
    node.check_placement(context)
    node.open(context)
    node.confirm_release(context)
    node.return_arm(context)
    outcome = node.verify(context)
    assert outcome.success and outcome.placement_state == Placement.ASSUMED
    assert context.arm_recovered and context.release_confirmed and node.carrying is None
    assert commands.index('return_to_pickup') < commands.index('release') < commands.index('return')
    assert node.report['contact_verification'] == 'SKIPPED'
    assert node.report['placement_verified'] is False


@pytest.mark.parametrize('collision_checks', [None, True, False])
@pytest.mark.parametrize('opening_collision_checks', [None, True, False])
def test_bt_grasp_collision_policy_is_independent_of_opening_policy(
        temporary, collision_checks, opening_collision_checks):
    node, context, _, _, _ = temporary
    node.config.pop('grasp_collision_check_enabled', None)
    if collision_checks is not None:
        node.config['grasp_collision_check_enabled'] = collision_checks
    node.config.pop('gripper_opening_collision_check_enabled', None)
    if opening_collision_checks is not None:
        node.config['gripper_opening_collision_check_enabled'] = opening_collision_checks
    commands = []
    node.jaw = lambda arm, target, stage, **options: commands.append((stage, options))
    node.grasp(context)
    node.open(context)
    assert commands == [
        ('close_assumed', {'check_collisions': collision_checks is not False}),
        ('release', {'check_collisions': opening_collision_checks is not False}),
    ]


@pytest.mark.parametrize('collision_checks', [None, True, False])
@pytest.mark.parametrize('pregrasp', [True, False])
def test_bt_lift_collision_policy_is_scoped_to_retreat_and_lift(
        temporary, collision_checks, pregrasp):
    node, context, _, _, _ = temporary
    node.config.pop('lift_collision_check_enabled', None)
    node.config['pregrasp_motion_enabled'] = pregrasp
    if collision_checks is not None:
        node.config['lift_collision_check_enabled'] = collision_checks
    paths, linear = [], node.linear
    def recording(arm, target, stage, **options):
        if stage in ('retreat', 'lift'):
            assert context.held and node.carrying is not None
        paths.append((stage, options.get('check_collisions', True)))
        return linear(arm, target, stage, **options)
    node.linear = recording
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    node.carry(context)
    node.open(context)
    node.confirm_release(context)
    node.return_arm(context)
    expected = collision_checks is not False
    assert paths == ([('retreat', expected)] if pregrasp else []) + [
        ('lift', expected), ('lower_to_pregrasp' if pregrasp else 'lower_to_pickup', True),
        ('retreat_after_release', True)]
    assert node.report['lift_collision_checks_enabled'] is expected


@pytest.mark.parametrize('collision_checks', [None, True, False])
def test_release_retreat_collision_policy_finishes_before_home_return(temporary, collision_checks):
    node, context, _, _, commands = temporary
    node.config.pop('retreat_after_release_collision_check_enabled', None)
    if collision_checks is not None:
        node.config['retreat_after_release_collision_check_enabled'] = collision_checks
    paths, linear = [], node.linear

    def recording(arm, target, stage, **options):
        paths.append((stage, options.get('check_collisions', True)))
        assert not context.arm_recovered
        return linear(arm, target, stage, **options)

    node.linear = recording
    node.open(context)
    node.confirm_release(context)
    node.return_arm(context)
    assert paths == [('retreat_after_release', collision_checks is not False)]
    assert commands == ['release', 'retreat_after_release', 'return']
    assert context.arm_recovered


@pytest.mark.parametrize('temporary_mode', [False, True])
@pytest.mark.parametrize('execution_fails', [False, True])
def test_automatic_home_return_closes_gripper_on_the_same_path(
        temporary, temporary_mode, execution_fails):
    from sensor_msgs.msg import JointState

    node, context, joints, _, commands = temporary
    arm = context.selected.selected_arm
    if not temporary_mode:
        node.config['operation_mode'] = 'full_collection'
    del node.move_arm_return  # Exercise the real shared synchronized-return planner.
    state = JointState(name=[*joints.name, f'{arm}_gripper_joint'],
                       position=[*joints.position, node.arm_config.opened])
    node.real.monitor = lambda: state
    node.check_execution = lambda: None
    node.motion = NS(set_current_state=lambda _: None, set_gripper_position=lambda *_: None,
                     _validated_cubic_path=lambda _, path: path,
                     plan=lambda *_: pytest.fail('Valid direct return must bypass RRT'))
    events, dispatched = [], []
    node.event = lambda event, **fields: events.append((event, fields))

    def execute(chosen, trajectory, jaw):
        assert not context.arm_recovered
        assert commands == (['release', 'retreat_after_release'] if temporary_mode else ['release'])
        assert chosen == arm
        assert tuple(trajectory.points[-1].positions) == node.arm_config.home
        assert jaw.joint_names == [f'{arm}_gripper_joint']
        assert jaw.points[0].positions == pytest.approx([node.arm_config.opened])
        assert jaw.points[-1].positions == pytest.approx([node.arm_config.closed])
        dispatched.append((trajectory, jaw))
        if execution_fails:
            raise RuntimeError('gripper goal not confirmed')

    node.real.execute_arm_gripper = execute
    node.open(context)
    context.release_confirmed = True
    if execution_fails:
        with pytest.raises(RuntimeError, match='gripper goal not confirmed'):
            node.return_arm(context)
        assert not context.arm_recovered
    else:
        result = node.return_arm(context)
        assert result.arm_recovered and context.arm_recovered
    assert len(dispatched) == 1
    assert any(event == 'return_planned' and fields['gripper_goal_rad'] == node.arm_config.closed
               for event, fields in events)


def test_failed_release_retreat_does_not_start_home_return_or_close_gripper(temporary):
    node, context, _, _, commands = temporary
    node.open(context)
    node.confirm_release(context)

    def blocked(*args, **kwargs):
        raise RuntimeError('retreat unavailable')

    node.linear = blocked
    with pytest.raises(RuntimeError, match='retreat unavailable'):
        node.return_arm(context)
    assert commands == ['release']
    assert not context.arm_recovered


def test_bt_reselects_from_current_pose_and_updates_context_before_closing(temporary):
    from cleany_interfaces.msg import GraspCandidate
    from cleany_skill_executor.core.hardware_execution import JointRangeValidationError
    from cleany_manipulation_bt.backend import Observation

    node, context, _, _, commands = temporary
    node.config['approach_execution_replanning_enabled'] = True
    node.real.motion_submissions = 0
    node.wrist_request = None
    candidates = [GraspCandidate(snapshot_id=context.goal.snapshot_id,
                                object_id=context.goal.object_id, score=float(i)) for i in range(2)]
    previous = context.selected
    previous.candidates, previous.selected_candidate_index = candidates, 0
    previous.selected_candidate = candidates[0]
    next_selection = deepcopy(previous)
    next_selection.candidates = candidates[1:]
    next_selection.selected_candidate = candidates[1]
    node.reselect_approach_candidate = lambda prior, remaining: next_selection
    def pregrasp(current):
        assert current is context and current.selected is next_selection
        commands.append('new_pregrasp')
        node.real.motion_submissions += 1
        return Observation()
    node.pregrasp = pregrasp
    def approach(selected, report):
        if selected is previous:
            commands.append('rejected_approach')
            raise JointRangeValidationError(2.1, -2., 2., 3400, 0., 4095.)
        commands.append('accepted_approach')
    node.approach_target = approach
    assert node.approach(context).success
    assert context.selected is next_selection and not context.gripper_engaged
    node.grasp(context)
    assert commands == ['rejected_approach', 'new_pregrasp', 'accepted_approach', 'close_assumed']
    assert node.report['approach_candidate_reselections'] == 1


def test_wrong_release_pose_is_rejected_and_stop_failure_stays_failure(temporary):
    node, context, joints, _, _ = temporary
    joints.position = [.9]*5
    with pytest.raises(OperationError) as failure:
        node.check_placement(context)
    assert failure.value.error == Error.DESTINATION_UNAVAILABLE
    node._assess_stationary = lambda *args, **kw: False
    assert node.verify(context).error == Error.STOP_UNCONFIRMED


@pytest.mark.parametrize('replanning', [False, True])
@pytest.mark.parametrize('collision_checks', [None, True, False])
def test_bt_records_approach_then_reverses_with_closed_jaw_and_fixed_roll(
        temporary, replanning, collision_checks):
    from cleany_skill_executor.core.recorded_trajectory import CommandPoint, RecordedTrajectory
    from cleany_skill_executor.hardware_execution import HardwareExecution

    node, context, joints, pickup, commands = temporary
    arm = context.selected.selected_arm
    node.config.update(retreat_reverse_approach_enabled=True,
                       approach_execution_replanning_enabled=replanning)
    node.config.pop('lift_collision_check_enabled', None)
    if collision_checks is not None:
        node.config['lift_collision_check_enabled'] = collision_checks
    node.real.record_arm_commands = lambda chosen: HardwareExecution.record_arm_commands(node.real, chosen)
    # This operation-order fixture leaves Cartesian pose unchanged during its
    # synthetic approach; expose that same measured geometry through the FK port.
    def fk(_arm, _solution):
        tcp = node.pose(arm)
        return NS(position=(tcp.position.x, tcp.position.y, tcp.position.z),
                  orientation_xyzw=(tcp.orientation.x, tcp.orientation.y, tcp.orientation.z, tcp.orientation.w))
    node.motion = NS(set_current_state=lambda _: None, forward_kinematics=fk)
    names = ARM_JOINT_NAMES[arm]
    forward = RecordedTrajectory(names, (
        CommandPoint(0, pickup, (0.,)*5),
        CommandPoint(10**9, tuple(q+.1 for q in pickup), (0.,)*5)))
    context.selected.selected_candidate = NS()
    if replanning:
        context.selected.candidates = [context.selected.selected_candidate]
        context.selected.selected_candidate_index = 0
    def approach(*args):
        commands.append('approach')
        node.real._arm_command_recording.trajectories.append(forward)
        joints.position = list(forward.points[-1].positions)
    node.approach_target = approach
    assert node.approach(context).success
    assert node.grasp(context).success and node.confirm_grasp(context).success
    joints.position[-1] = .63  # roll measured after grasp
    def reverse(chosen, path, *, check_collisions, endpoint_pose):
        assert chosen == arm and context.held and node.carrying is not None
        assert endpoint_pose == node.cartesian_pose(node.pregrasp_pose)
        assert check_collisions is (collision_checks is not False)
        assert path.points[0].positions[:4] == forward.points[-1].positions[:4]
        assert path.points[-1].positions[:4] == forward.points[0].positions[:4]
        assert all(p.positions[-1] == .63 and p.velocities[-1] == 0. for p in path.points)
        commands.append('reverse_approach')
        joints.position = list(path.points[-1].positions)
        return joints
    node.real.execute_recorded_trajectory = reverse
    assert node.lift(context).success
    assert commands == ['approach', 'close_assumed', 'reverse_approach', 'lift']
    assert node.report['retreat_route'] == 'reversed_approach'
    assert node._approach_command_paths is None


def test_bt_missing_history_stops_before_retreat_or_lift(temporary):
    from cleany_skill_executor.core.hardware_execution import CommandPathValidationError
    node, context, _, _, commands = temporary
    node.config['retreat_reverse_approach_enabled'] = True
    with pytest.raises(CommandPathValidationError, match='no recorded approach'):
        node.lift(context)
    assert commands == []


def test_bt_direct_grasp_prepares_jaw_moves_then_closes_without_pregrasp(temporary):
    from dataclasses import replace

    node, context, joints, pickup, commands = temporary
    arm = context.selected.selected_arm
    node.config.update(pregrasp_motion_enabled=False, temporary_contact_verification='raw_feedback')
    node.arm_config = replace(load_arm(node.config['arms'][arm], require_bins=False), home=(0.,)*5)
    node.scene_monitor = NS(wait=lambda: None)
    node.motion = NS(set_gripper_position=lambda *args: None)
    joints.header = NS(stamp=NS(sec=0, nanosec=100))
    context.selected.selected_candidate = NS(required_opening_m=.04)
    context.selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[arm], position=pickup)
    context.selected.fk_grasp_pose = node.pose(arm)
    node.close_until_contact = lambda *args: commands.append('close_raw')
    node.real.wait_for_contact = lambda *args: None
    assert node.pregrasp(context).success
    assert commands == ['open_before_direct_grasp']
    assert node.report['pregrasp_executed'] is False
    for operation in (node.approach, node.grasp, node.confirm_grasp, node.lift,
                      node.confirm_held, node.carry, node.check_placement, node.open,
                      node.confirm_release, node.return_arm):
        assert operation(context).success
    assert commands.index('open_before_direct_grasp') < commands.index('direct_grasp')
    assert commands.index('direct_grasp') < commands.index('close_raw') < commands.index('lift')
    assert 'retreat' not in commands and 'approach' not in commands
    assert 'lower_to_pickup' in commands and 'lower_to_pregrasp' not in commands
    assert node.report['direct_grasp_executed'] is True
    assert context.release_confirmed and context.arm_recovered


@pytest.mark.parametrize('pose_checks', [True, False])
def test_bt_attempts_close_after_tcp_error_only_in_relaxed_grasp_mode(temporary, pose_checks):
    node, context, joints, pickup, commands = temporary
    arm = context.selected.selected_arm
    node.config.update(pregrasp_motion_enabled=False, temporary_contact_verification='raw_feedback',
                       direct_grasp_pose_checks_enabled=pose_checks)
    target = node.pose(arm)
    target.position.x += .04
    target.orientation.z = target.orientation.w = 2**-.5
    context.selected.selected_candidate = NS(required_opening_m=.04)
    context.selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[arm], position=pickup)
    context.selected.fk_grasp_pose = target
    node.get_logger = lambda: NS(warning=lambda _: None)
    def joint_move(_arm, positions, stage, **options):
        assert options == ({} if pose_checks else {'allow_endpoint_error': True})
        commands.append(stage)
    node.move_joint_pose = joint_move
    node.close_until_contact = lambda *args: commands.append('close_raw')
    if pose_checks:
        with pytest.raises(RuntimeError, match='measured TCP target not reached'):
            node.approach(context)
        assert 'close_raw' not in commands
    else:
        assert node.approach(context).success
        assert node.grasp(context).success
        assert commands == ['direct_grasp', 'close_raw']
        assert node.report['direct_grasp_tcp_position_error_m'] == pytest.approx(.04)
        assert node.report['direct_grasp_tcp_orientation_error_deg'] == pytest.approx(90.)
        assert not context.held  # Closing is not proof of a confirmed grasp.


def test_open_failure_does_not_mark_release_confirmed(temporary):
    node, context, _, _, _ = temporary
    node.jaw = lambda *args, **options: (_ for _ in ()).throw(RuntimeError('controller failed'))
    with pytest.raises(RuntimeError, match='controller failed'):
        node.open(context)
    assert not context.release_confirmed


@pytest.mark.parametrize('invalid', ['missing_destination', 'nonfinite_feedback', 'wrong_position'])
def test_no_release_when_original_position_was_not_reached(temporary, invalid):
    node, context, joints, _, commands = temporary
    if invalid == 'missing_destination':
        node.pickup_joints = None
    elif invalid == 'nonfinite_feedback':
        joints.position[0] = float('nan')
    else:
        joints.position = [.9]*5
    with pytest.raises(OperationError):
        node.check_placement(context)
    assert not context.release_confirmed and 'release' not in commands


def test_release_pose_check_does_not_relabel_a_hardware_guard_failure(temporary):
    node, context, _, _, _ = temporary
    node.real.monitor = lambda: (_ for _ in ()).throw(RuntimeError('hardware fault'))
    with pytest.raises(RuntimeError, match='hardware fault'):
        node.check_placement(context)


def test_temporary_preflight_and_payload_monitor_do_not_require_contact_ports(temporary):
    node, context, _, _, _ = temporary
    arm = context.selected.selected_arm
    raw = node.config['arms'][arm]
    limits = {name: NS(check=lambda _: None) for name in (*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint')}
    node.real.verify = lambda _: limits
    node.real.closed_contact_position = node.real.contact_decision = forbidden
    node.evidence = NS(wait_for_service=forbidden)
    selected = NS(selected_arm=arm, selected_candidate=NS(required_opening_m=.04),
                  gripper_joint_position=load_arm(raw, require_bins=False).opening(.04))
    node.before_execution(selected)
    assert node.destination == 'return_to_pick'
    node.carrying = {'release_confirmed': False}
    node.monitor_payload()


@pytest.mark.parametrize('arrival_replanning', [False, True])
@pytest.mark.parametrize('collision_checks', [None, True, False])
def test_legacy_pipeline_also_returns_releases_and_stops_after_one_cycle(
        temporary, arrival_replanning, collision_checks):
    node, context, joints, pickup, commands = temporary
    node.config['visual_servo_enabled'] = False
    node.config['approach_execution_replanning_enabled'] = arrival_replanning
    node.config.pop('lift_collision_check_enabled', None)
    if collision_checks is not None:
        node.config['lift_collision_check_enabled'] = collision_checks
    paths, linear = [], node.linear
    def recording(arm, target, stage, *args, **options):
        paths.append((stage, options.get('check_collisions', True)))
        return linear(arm, target, stage, *args, **options)
    node.linear = recording
    if arrival_replanning:
        # This fixture tests operation order with exact motion ports. Held-start
        # approach planning is exercised by test_approach_replanning.py.
        node.replan_approach_from_arrival = lambda chosen, report: node.linear(
            chosen.selected_arm, chosen.fk_grasp_pose, 'approach')
    selected = context.selected
    selected.selected_candidate = NS()
    selected.candidates, selected.selected_candidate_index = [selected.selected_candidate], 0
    selected.fk_grasp_pose = node.pose(selected.selected_arm)
    # The shared pipeline calls its static helper on the selected result.
    selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[selected.selected_arm], position=list(pickup))
    report = {}
    CollectionPipeline.after_pregrasp(node, selected, None, report)
    assert report['released'] and report['returned'] and report['placement_state'] == 'ASSUMED'
    assert commands.index('return_to_pickup') < commands.index('release_at_pickup') < commands.index('return')
    expected = collision_checks is not False
    assert ('retreat', expected) in paths and ('lift', expected) in paths
    assert ('retreat_after_release', True) in paths
    assert report['lift_collision_checks_enabled'] is expected
    calls = []
    node.events = []
    node.run = lambda query: calls.append(query) or dict(success=True, **report)
    collected = node.run_collection('actual objects')
    assert collected['success'] and len(calls) == 1


@pytest.mark.parametrize('shortfall', [.008, .010, .0101, .040])
def test_legacy_lift_records_shortfall_without_blocking_return(temporary, shortfall):
    node, context, _, pickup, commands = temporary
    node.config['visual_servo_enabled'] = False
    linear = node.linear
    def loaded_lift(arm, target, stage, *args, **options):
        measured = deepcopy(target)
        if stage == 'lift':
            measured.position.z -= shortfall
        return linear(arm, measured, stage, *args, **options)
    node.linear = loaded_lift
    selected = context.selected
    selected.selected_candidate = NS()
    selected.fk_grasp_pose = node.pose(selected.selected_arm)
    selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[selected.selected_arm], position=list(pickup))
    report = {}
    CollectionPipeline.after_pregrasp(node, selected, None, report)
    assert report['lift_height_from_fk_m'] == pytest.approx(.05-shortfall)
    assert report['released'] and report['returned']
    assert 'lift_height_correction' not in commands


@pytest.mark.parametrize('adapter', ['bt', 'legacy'])
@pytest.mark.parametrize('stalled', [False, True])
def test_one_height_correction_continues_through_return_and_release(temporary, adapter, stalled):
    from cleany_skill_executor.core.hardware_execution import ArmHold

    node, context, joints, pickup, commands = temporary
    arm = context.selected.selected_arm
    node.config.update(lift_height_correction_enabled=True, visual_servo_enabled=False,
                       lift_height_correction_max_attempts=3, lift_height_tolerance_m=.010)
    node.real.wait_for_settle = lambda _: joints
    node.real.arm_holds = {}
    node.real.value = {'hardware_tcp_position_tolerance_m': .03,
                       'hardware_tcp_orientation_tolerance_deg': 20.,
                       'hardware_arrival_tolerance_rad': .01}.__getitem__
    node.motion = NS(set_current_state=lambda _: None,
        forward_kinematics=lambda _, q: NS(position=(0., 0., q.positions[0]),
                                          orientation_xyzw=(0., 0., 0., 1.)))
    linear = node.linear
    def loaded_motion(selected_arm, target, stage, *args, **options):
        measured = deepcopy(target)
        motor = node.pose_message(options['command_target']) if 'command_target' in options else target
        if stage == 'lift':
            measured.position.z -= .011
        elif stage == 'lift_height_correction':
            measured = node.pose(arm) if stalled else deepcopy(motor)
            if not stalled:
                measured.position.z -= .011
        linear(selected_arm, measured, stage, *args, **options)
        q = (motor.position.z, 0., 0., 0., 0.)
        node.real.arm_holds[arm] = ArmHold(q, q)
    node.linear = loaded_motion
    if adapter == 'bt':
        node.grasp(context)
        node.confirm_grasp(context)
        node.lift(context)
        node.confirm_held(context)
        node.carry(context)
        node.check_placement(context)
        node.open(context)
        node.confirm_release(context)
        node.return_arm(context)
        report, release = node.report, 'release'
    else:
        context.selected.selected_candidate = NS()
        context.selected.fk_grasp_pose = node.pose(arm)
        context.selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[arm], position=list(pickup))
        report, release = {}, 'release_at_pickup'
        CollectionPipeline.after_pregrasp(node, context.selected, None, report)
    correction = report['height_corrections']['lift']
    assert correction['status'] == 'applied'
    assert correction['total_correction_m'] == pytest.approx(.015)
    assert commands.count('lift_height_correction') == 1
    assert commands.index('lift_height_correction') < commands.index('return_to_pickup')
    assert commands.index('return_to_pickup') < commands.index(release) < commands.index('return')
    if stalled:
        assert correction['error_m'] == pytest.approx(.011)


@pytest.mark.parametrize('key', ['temporary_contact_verification', 'temporary_placement_verification', 'label_destinations'])
def test_temporary_bypasses_require_explicit_configuration(key):
    config = json.loads(CONFIG.read_text())
    assert set(config['arms']) == {'left', 'right'} and not configuration_issues(config)
    del config[key]
    assert configuration_issues(config)


def test_temporary_raw_feedback_requires_contact_and_fresh_post_lift_identity(temporary):
    from cleany_interfaces.srv import CheckManipulation
    node, context, _, _, _ = temporary
    node.config['temporary_contact_verification'] = 'raw_feedback'
    node.config['wrist_observation_enabled'] = True
    calls = []
    def close(_, report):
        calls.append('close_raw')
        report.update(grasp_contact_confirmed=True, grasp_assumed=False)
    node.close_until_contact = close
    node.real.wait_for_contact = lambda arm,after:calls.append(('contact',arm,after))
    node.confirm_wrist_target = lambda *_:calls.append('fresh_wrist_grasp')
    node.verify_evidence = lambda _,stage,after:calls.append(('evidence',stage,after))
    node.wrist_observation_required = lambda:True
    node.confirm_held_target = lambda *_:calls.append('fresh_wrist_lift')
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    node.confirm_held(context)
    assert calls == ['close_raw',('contact',context.selected.selected_arm,100),
                     'fresh_wrist_grasp',('evidence',CheckManipulation.Request.GRASP,100),
                     'fresh_wrist_lift',('evidence',CheckManipulation.Request.LIFT,100)]
    assert context.held and node.report['grasp_verified']
    assert not node.report['grasp_assumed']


def test_temporary_raw_feedback_failure_cannot_attach_or_lift(temporary):
    node, context, _, _, commands = temporary
    node.config['temporary_contact_verification'] = 'raw_feedback'
    node.close_until_contact = lambda *_:(_ for _ in ()).throw(RuntimeError('empty gripper'))
    with pytest.raises(RuntimeError, match='empty gripper'):
        node.grasp(context)
    assert not context.held and node.carrying is None
    assert 'lift' not in commands


def test_temporary_without_wrist_keeps_fresh_contact_and_returns_without_identity_claim(temporary):
    node, context, joints, pickup, commands = temporary
    node.config['temporary_contact_verification'] = 'raw_feedback'
    assert node.config['wrist_observation_enabled'] is False
    calls = []
    node.confirm_wrist_target = node.confirm_held_target = forbidden
    node.close_until_contact = lambda _, report: report.update(
        grasp_contact_confirmed=True, grasp_assumed=False)
    node.real.wait_for_contact = lambda arm, after: calls.append((arm, after))
    node.grasp(context)
    node.confirm_grasp(context)
    node.lift(context)
    node.confirm_held(context)
    assert calls == [(context.selected.selected_arm, 100)] * 2
    assert context.held and node.report['grasp_contact_confirmed']
    assert node.report['grasp_verified'] is False
    assert node.report['grasp_identity_verified'] is False
    assert node.report['grasp_evidence'] == 'motor_contact_only_after_lift'
    joints.position = [.8] * 5
    node.carry(context)
    assert tuple(joints.position) == pickup
    node.check_placement(context)
    node.open(context)
    node.confirm_release(context)
    node.return_arm(context)
    assert node.verify(context).success and context.arm_recovered


def test_temporary_without_wrist_rejects_lost_contact_after_lift(temporary):
    node, context, _, _, commands = temporary
    node.config['temporary_contact_verification'] = 'raw_feedback'
    node.lift_after_ns = 100
    node.real.wait_for_contact = lambda *_: (_ for _ in ()).throw(RuntimeError('empty gripper'))
    with pytest.raises(RuntimeError, match='empty gripper'):
        node.confirm_held(context)
    assert not node.report.get('grasp_verified')
    assert 'return_to_pickup' not in commands


@pytest.mark.parametrize('stationary', [False, True])
def test_temporary_pregrasp_without_wrist_skips_handoff(temporary, stationary):
    node, context, _, _, _ = temporary
    node.config['assume_stationary_target'] = stationary
    node.wrist_observation_required = lambda: CollectionPipeline.wrist_observation_required(node)
    assert not node.wrist_observation_required()
    context.selected.reference_id = 'head-reference'
    observation = NS(observed_center=NS(x=.3, y=.1, z=.4))
    node.observe = forbidden if stationary else lambda *_: observation
    node._check_object_envelope = lambda *_: None
    node.execute_pregrasp = lambda _: 100
    node.observe_arrival = forbidden if stationary else lambda *_: observation
    node.validate_target_translation = lambda *_: None
    node.call = forbidden
    assert node.pregrasp(context).success
    if stationary:
        assert node.before_observation is None and node.after_observation is None
    else:
        assert node.before_observation is observation and node.after_observation is observation


def test_legacy_pipeline_without_wrist_keeps_contact_through_return_release(temporary):
    node, context, _, _, _ = temporary
    node.config.update(temporary_contact_verification='raw_feedback', visual_servo_enabled=False)
    node.confirm_wrist_target = node.confirm_held_target = forbidden
    node.real.verify = lambda _: {}
    node.real.closed_contact_position = lambda *_: -.3
    contacts = []
    decision = NS(minimum_residual_ticks=25, median_current_raw=10, minimum_load_raw=10,
                  position_span_ticks=1, window_sec=.4)
    node.real.wait_for_contact = lambda arm, after: contacts.append((arm, after)) or decision
    node.close_until_contact = lambda selected, report: CollectionPipeline.close_until_contact(node, selected, report)
    selected = context.selected
    selected.selected_candidate = NS()
    selected.fk_grasp_pose = node.pose(selected.selected_arm)
    selected.grasp_joint_state = NS(name=ARM_JOINT_NAMES[selected.selected_arm], position=[0.] * 5)
    report = {}
    CollectionPipeline.after_pregrasp(node, selected, None, report)
    assert contacts == [(selected.selected_arm, 100)] * 2
    assert report['released'] and report['returned']
    assert report['grasp_contact_confirmed'] and report['contact_verification'] == 'RAW_FEEDBACK'
    assert report['grasp_verified'] is False and report['grasp_identity_verified'] is False
