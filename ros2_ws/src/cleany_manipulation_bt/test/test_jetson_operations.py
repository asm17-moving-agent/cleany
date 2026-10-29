"""Approved-object hardware adapter contracts at the existing ROS/motion ports."""
from copy import deepcopy
from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading
import math

import pytest
from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import DetectedObject2D, DetectedObject2DArray, DetectedObject3D, GraspCandidate
from cleany_interfaces.srv import GetSceneSnapshot, ObserveObjectReference
from cleany_manipulation_bt.backend import ExecutionContext, Observation, OperationError
from cleany_skill_executor.core.hardware_execution import ExecutionCanceled
from cleany_manipulation_bt.jetson_operations import JetsonOperations
from cleany_manipulation_bt.jetson_node import JetsonManipulationNode
from cleany_skill_executor.manipulation.models import Error, Goal, ObjectState, Placement


@pytest.fixture
def operation():
    node = object.__new__(JetsonOperations)
    node.wrist_cameras = NS(activate=lambda arm: None, close=lambda: None, request_close=lambda: None)
    node.config = {'operation_mode': 'full_collection',
                   'frame': 'base_link',
                   'label_destinations': {'paper cup': 'trash_bin'}}
    node.report = {}
    values = {'bt_minimum_confidence': .25, 'bt_denylist': ['battery'],
              'bt_maximum_smaller_extent_m': .11}
    node.value = values.__getitem__
    node._snapshot = object()
    detection = DetectedObject2D(object_id=2, label='paper cup', confidence=.9,
        sorting_category='trash',  distance_valid=True,
        distance_m=.5, median_depth_m=.5, estimated_width_m=.07, estimated_height_m=.1,
        x_min=0., y_min=0., x_max=100., y_max=100.)
    snapshot = DetectedObject2DArray(snapshot_id='approved', detections=[
        DetectedObject2D(object_id=1, label='closer object', distance_valid=True, distance_m=.1),
        detection])
    node.call = lambda *args: GetSceneSnapshot.Response(found=True, detections=deepcopy(snapshot))
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 'approved', 2, 'trash_bin'))
    return node, context, snapshot


def test_prepare_preserves_approved_object_instead_of_selecting_closest(operation):
    node, context, _ = operation
    assert node.prepare(context).success
    assert context.target.object_id == 2 and context.snapshot.snapshot_id == 'approved'


@pytest.mark.parametrize('enabled,stalled,sag', [
    (True, False, .0185967956680563), (True, True, .0185967956680563),
    (False, False, .0185967956680563), (False, False, .0101),
    (False, False, .008), (True, False, .008), (True, False, .0005266009432072227),
    (False, False, .010), (True, False, .010), (True, False, -.020),
])
def test_lift_corrects_static_sag_against_original_height_before_confirm_held(
        operation, enabled, stalled, sag):
    from geometry_msgs.msg import Pose
    from cleany_skill_executor.core.hardware_execution import ArmHold

    node, context, _ = operation
    node.config.update(pregrasp_motion_enabled=False, lift_height_m=.05,
                       cartesian_tolerance_m=.005, lift_height_tolerance_m=.010,
                       lift_height_correction_enabled=enabled)
    context.selected = NS(selected_arm='right')
    actual = Pose()
    actual.position.z, actual.orientation.w = .5243888133737824, 1.
    original_z = actual.position.z
    node.pose = lambda _: deepcopy(actual)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100))
    node.event = lambda *args, **kwargs: None
    values = {'hardware_tcp_position_tolerance_m': .03, 'hardware_tcp_orientation_tolerance_deg': 20.}
    node.real = NS(wait_for_settle=lambda _: object(), value=values.__getitem__, arm_holds={})
    node.motion = NS(set_current_state=lambda _: None,
        forward_kinematics=lambda _, q: NS(position=(0., 0., q.positions[0]),
                                          orientation_xyzw=(0., 0., 0., 1.)))
    commands = []
    def linear(arm, target, stage, **kwargs):
        commands.append(stage)
        motor = node.pose_message(kwargs['command_target']) if 'command_target' in kwargs else target
        if stage == 'lift' or not stalled:
            actual.position.z = motor.position.z - sag
        q = (motor.position.z, 0., 0., 0., 0.)
        node.real.arm_holds[arm] = ArmHold(q, q)
    node.linear = linear
    assert node.lift(context).success
    if enabled:
        summary = node.report['height_corrections']['lift']
        assert summary['target_z_m'] == pytest.approx(original_z+.05)
        assert summary['tolerance_m'] == .010
    assert node.report['lift_height_from_fk_m'] == pytest.approx(actual.position.z-original_z)
    assert commands == ['lift'] + ['lift_height_correction']*(1 if enabled and sag > .010 else 0)
    if stalled:
        assert node.report['lift_height_from_fk_m'] < .040


def test_lift_includes_logged_retreat_shortfall_without_lowering_the_goal(operation):
    from geometry_msgs.msg import Pose
    from cleany_skill_executor.core.hardware_execution import ArmHold

    node, context, _ = operation
    node.config.update(pregrasp_motion_enabled=True, lift_height_m=.05,
                       lift_height_correction_enabled=True,
                       lift_height_correction_max_attempts=3)
    context.selected = NS(selected_arm='right')
    nominal_retreat, measured_retreat = .5408224849074599, .508824099160217
    sag = nominal_retreat-measured_retreat
    actual = Pose()
    actual.position.z, actual.orientation.w = .44909792964293765, 1.
    node.pregrasp_pose = deepcopy(actual)
    node.pregrasp_pose.position.z = nominal_retreat
    node.pose = lambda _: deepcopy(actual)
    node.event = lambda *args, **kwargs: None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100))
    node.retreat_to_pregrasp = lambda *args, **kwargs: setattr(actual.position, 'z', measured_retreat)
    values = {'hardware_tcp_position_tolerance_m': .030,
              'hardware_tcp_orientation_tolerance_deg': 20.}
    node.real = NS(wait_for_settle=lambda _: object(), value=values.__getitem__, arm_holds={})
    node.motion = NS(set_current_state=lambda _: None,
        forward_kinematics=lambda _, q: NS(position=(0.,0.,q.positions[0]),
                                          orientation_xyzw=(0.,0.,0.,1.)))
    targets = []
    def linear(arm, target, stage, **options):
        targets.append(deepcopy(target))
        motor = node.pose_message(options['command_target']) if 'command_target' in options else target
        actual.position.z = motor.position.z-sag
        q = (motor.position.z,0.,0.,0.,0.)
        node.real.arm_holds[arm] = ArmHold(q,q)
    node.linear = linear
    assert node.lift(context).success
    expected_goal = nominal_retreat+.050
    assert targets[0].position.z == pytest.approx(expected_goal)
    assert len(targets) == 2
    assert abs(expected_goal-actual.position.z) <= .010
    assert node.report['height_corrections']['lift']['target_z_m'] == pytest.approx(expected_goal)
    assert node.pregrasp_pose.position.z == nominal_retreat


@pytest.mark.parametrize('stationary', [False, True])
def test_arm_selection_retains_approved_target_with_optional_head_reobservation(operation, stationary):
    node, context, snapshot = operation
    node.config.update(arms={'right': {}}, assume_stationary_target=stationary,
                       wrist_observation_enabled=False, visual_servo_enabled=False)
    obj = DetectedObject3D(object_id=2, label='paper cup')
    obj.obb_pose.position.x, obj.obb_pose.position.y, obj.obb_pose.position.z = .3, -.1, .4
    obj.obb_size.x, obj.obb_size.y, obj.obb_size.z = .04, .04, .08
    candidate = GraspCandidate(snapshot_id='approved', object_id=2)
    context.inspected = NS(objects=NS(objects=[obj]))
    context.grasps = NS(candidates=[candidate])
    context.target, context.snapshot, context.verification_id = snapshot.detections[1], snapshot, 'pinned'
    node._selection, node.report = object(), {}
    node.value = lambda _: 600.
    node._selection_progress = lambda **_: None
    node.run_action = lambda *args, **kwargs: SelectReachableGrasp.Result(
        success=True, selected_arm='right', selected_candidate=candidate)
    node.before_execution = lambda _: setattr(node, 'destination', context.goal.destination_id)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100))
    node.get_parameter = lambda _: NS(value=.01)
    calls = []
    node.event = lambda stage, **kwargs: calls.append(stage)
    def observe(*_):
        calls.append('observe')
        return NS(header=NS(frame_id='base_link'), observed_center=obj.obb_pose.position)
    node.observe = observe
    result = node.select(context)
    assert result.success and result.selected_arm == 'right'
    assert context.selected.selected_candidate.snapshot_id == 'approved'
    assert context.selected.selected_candidate.object_id == 2
    assert calls == (['stationary_target_assumed'] if stationary else ['observe'])
    if stationary:
        assert node.before_observation is None and node.after_observation is None
        assert node.report['target_motion_assumed_stationary'] is True
        assert node.report['head_reobservation_performed'] is False


@pytest.mark.parametrize('stationary', [False, True])
def test_pregrasp_optional_reobservation_preserves_motion_and_endpoint_checks(operation, stationary):
    node, context, _ = operation
    node.config.update(assume_stationary_target=stationary, wrist_observation_enabled=False)
    center = NS(x=.3, y=-.1, z=.4)
    context.selected = NS(reference_id='pinned', selected_arm='right',
        selected_object=NS(obb_pose=NS(position=center), obb_size=NS(x=.04, y=.04, z=.08)))
    node.report = {}
    node.before_observation = node.after_observation = 'previous goal observation'
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100))
    node.get_parameter = lambda _: NS(value=.01)
    calls = []
    observed = NS(header=NS(frame_id='base_link'), observed_center=center)
    node.observe = lambda *_: calls.append('observe_before') or observed
    node.execute_pregrasp = lambda _: calls.append('move_and_verify_endpoint') or 200
    node.observe_arrival = lambda *_: calls.append('observe_after') or observed
    assert node.pregrasp(context).success
    assert calls == (['move_and_verify_endpoint'] if stationary else
                     ['observe_before', 'move_and_verify_endpoint', 'observe_after'])
    if stationary:
        assert node.before_observation is None and node.after_observation is None


def test_stationary_target_never_bypasses_a_physical_pregrasp_failure(operation):
    node, context, _ = operation
    node.config.update(assume_stationary_target=True, wrist_observation_enabled=False)
    context.selected = NS(reference_id='pinned')
    node.observe = lambda *_: pytest.fail('Stationary profile must not reobserve')
    def endpoint_failure(_):
        raise RuntimeError('Physical TCP endpoint outside tolerance')
    node.execute_pregrasp = endpoint_failure
    with pytest.raises(RuntimeError, match='Physical TCP endpoint outside tolerance'):
        node.pregrasp(context)


def test_stationary_approach_uses_selected_grasp_pose_without_visual_updates(operation):
    node, context, _ = operation
    node.config.update(operation_mode='pick_return_release', assume_stationary_target=True,
                       visual_servo_enabled=False, wrist_observation_enabled=False)
    target = object()
    context.selected = NS(selected_arm='right', selected_candidate=object(), fk_grasp_pose=target)
    node.scene = NS(begin=lambda *args: None, allow_contacts_for=lambda *_: None)
    node.pose = lambda _: 'measured pregrasp'
    node.selected_grasp_roll = lambda *_: 0.
    node.visual_approach = lambda *_: pytest.fail('Stationary target must retain the planned pose')
    calls = []
    node.linear = lambda *args, **kwargs: calls.append((args, kwargs))
    node.capture_pickup = lambda arm: calls.append(arm)
    assert node.approach(context).success
    assert calls == [(('right', target, 'approach', 0.), {'check_orientation': False}), 'right']


def test_bt_approach_delegates_replanning_and_captures_only_after_arrival(operation):
    node, context, _ = operation
    node.config.update(operation_mode='pick_return_release', visual_servo_enabled=False)
    selected = NS(selected_arm='right', selected_candidate=object())
    context.selected = selected
    calls = []
    node.scene = NS(begin=lambda *args: None, allow_contacts_for=lambda *_: None)
    node.pose = lambda _: 'measured pregrasp'
    def approach(target, report):
        assert target is selected and report is node.report
        calls.append('validated approach')
    node.approach_target = approach
    node.capture_pickup = lambda _: calls.append('capture pickup')
    assert node.approach(context).success
    assert calls == ['validated approach', 'capture pickup']


@pytest.mark.parametrize('fault,carrying,quiet,accepted', [
    ('', None, True, True), ('motor fault', None, True, False),
    ('', object(), True, False), ('', None, False, False),
])
def test_recovery_verification_is_passive_and_requires_an_idle_boundary(
        operation, fault, carrying, quiet, accepted):
    node, _, _ = operation
    node.real, node.carrying = NS(fault=fault), carrying
    node.quiescent = lambda: quiet
    calls = []

    def assess(context, *, cancel):
        assert cancel is False
        calls.append(context)
        return True

    node._assess_stationary = assess
    assert node.confirm_recovery_stationary() == accepted
    assert bool(calls) == accepted


@pytest.mark.parametrize('change', ['fault', 'carrying', 'pending'])
def test_recovery_rechecks_the_idle_boundary_after_waiting(operation, change):
    node, _, _ = operation
    node.real, node.carrying = NS(fault=''), None
    node.quiescent = lambda: True

    def assess(context, *, cancel):
        assert cancel is False
        if change == 'fault':
            node.real.fault = 'motor fault'
        elif change == 'carrying':
            node.carrying = object()
        else:
            node.quiescent = lambda: False
        return True

    node._assess_stationary = assess
    assert not node.confirm_recovery_stationary()


@pytest.mark.parametrize('error', [RuntimeError('failed motion'),
    ExecutionCanceled('canceled'), KeyError('programming failure')])
def test_operation_failure_closes_camera_even_before_stop_assessment(operation, error):
    node, context, _ = operation
    closed = []
    node.wrist_cameras = NS(request_close=lambda: closed.append(True),
                           close=lambda: pytest.fail('Camera reply cannot delay motion cancellation'))
    node.value = lambda _: 1.
    node.check_execution = lambda: None
    node.real = None

    def fail(_context):
        raise error

    node.approach = fail
    if isinstance(error, KeyError):
        with pytest.raises(KeyError):
            node.execute('ApproachObject', context)
    else:
        assert not node.execute('ApproachObject', context).success
    assert closed == [True]


@pytest.mark.parametrize('stage,closed', [('ApproachObject', False), ('ConfirmHeld', True),
                                       ('StopAndAssess', True)])
def test_camera_stays_on_until_last_observation_and_closes_on_stop(operation, stage, closed):
    node, context, _ = operation
    calls = []
    node.wrist_cameras = NS(close=lambda: calls.append('off'))
    node.value = lambda _: 1.
    node.check_execution = lambda: None
    node.approach = node.confirm_held = node.stop = lambda _: Observation()
    assert node.execute(stage, context).success
    assert bool(calls) == closed


@pytest.mark.parametrize('field,value,error', [
    ('sorting_category', 'lost_item', Error.TARGET_UNAVAILABLE),
    ('sorting_category', 'review', Error.TARGET_UNAVAILABLE),
    ('sorting_category', '', Error.TARGET_UNAVAILABLE),
    ('distance_valid', False, Error.TARGET_UNAVAILABLE),
    ('label', 'battery', Error.TARGET_UNAVAILABLE),
    ('confidence', .1, Error.TARGET_UNAVAILABLE),
    ('object_id', 3, Error.TARGET_UNAVAILABLE),
])
def test_unapproved_or_uncertain_target_is_blocked_before_motion(operation, field, value, error):
    node, context, snapshot = operation
    setattr(snapshot.detections[1], field, value)
    with pytest.raises(OperationError) as failed:
        node.prepare(context)
    assert failed.value.error == error


def test_snapshot_expiry_or_substitution_never_captures_another_target(operation):
    node, context, snapshot = operation
    node.call = lambda *args: GetSceneSnapshot.Response(found=False, message='expired')
    with pytest.raises(OperationError) as failed:
        node.prepare(context)
    assert failed.value.error == Error.STALE_TARGET
    snapshot.snapshot_id = 'other'
    node.call = lambda *args: GetSceneSnapshot.Response(found=True, detections=snapshot)
    with pytest.raises(OperationError):
        node.prepare(context)


def test_destination_must_match_explicit_physical_policy(operation):
    node, context, _ = operation
    node.config['label_destinations']['paper cup'] = 'other'
    with pytest.raises(OperationError) as failed:
        node.prepare(context)
    assert failed.value.error == Error.DESTINATION_UNAVAILABLE


def test_hold_only_profile_never_claims_full_collection_readiness(operation):
    node, context, _ = operation
    node.config['operation_mode'] = 'grasp_lift_hold'
    with pytest.raises(OperationError) as failed:
        node.validate(context)
    assert failed.value.error == Error.DESTINATION_UNAVAILABLE


def test_release_evidence_is_preserved_when_scene_detach_fails(operation):
    node, context, _ = operation
    node._outcome = lambda *args: None
    node.carrying = {'release_confirmed': False}
    def failed_detach():
        raise RuntimeError('scene service unavailable')
    node.detach = failed_detach
    context.selected = NS()
    with pytest.raises(RuntimeError):
        node.confirm_release(context)
    result = node._failure(context, Error.MOTION_FAILED, 'scene failure')
    assert result.object_state == ObjectState.LEFT_GRIPPER


def test_placement_success_without_stationary_joints_is_fatal(operation):
    node, context, _ = operation
    node._outcome = lambda *args: None
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10))
    node._assess_stationary = lambda *args, **kwargs: False
    result = node.verify(context)
    assert not result.success and result.error == Error.STOP_UNCONFIRMED
    assert result.placement_state == Placement.CONFIRMED and not result.stop_confirmed


def test_cancel_recovery_does_not_repeat_release_after_confirmed_release(operation):
    node, context, _ = operation
    context.release_confirmed = context.gripper_engaged = True
    node.open = lambda *args: pytest.fail('Already released object cannot be released again')
    assert node.release_in_place(context).object_state == ObjectState.LEFT_GRIPPER


def test_monitored_wait_rejects_stale_sensor_scene(operation):
    node, _, _ = operation
    node.check_execution = lambda: None
    node._stopping = False
    node.carrying = None
    node.real = NS(sensor_scene_frozen=False, monitor=lambda: pytest.fail('Stale scene must stop motion'))
    def stale_scene():
        raise RuntimeError('Depth scene is stale')
    node.scene_monitor = NS(check=stale_scene)
    with pytest.raises(RuntimeError, match='Depth scene is stale'):
        node.wait(Future(), 1., monitor=True)


@pytest.mark.parametrize('blocked', [None, 'hardware_fault', 'guard_fault', 'stale_guard',
                                   'no_context', 'stopping', 'other_error'])
def test_only_live_bt_scene_failures_defer_the_latched_stop(operation, monkeypatch, blocked):
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    node, context, _ = operation
    monkeypatch.setattr('cleany_manipulation_bt.jetson_operations.time.monotonic', lambda: 10.)
    node.execution_context, node._stopping = context, False
    node.real = NS(fault='', guard={'state': 'ready'}, guard_time=10., value=lambda _: .25)
    canceled, events = [], []
    node.event = lambda stage, **fields: events.append(stage)
    node.handles = [(NS(cancel_goal_async=lambda: canceled.append(True)), Future())]
    error = InfrastructureError('Sensor planning scene unavailable')
    if blocked == 'hardware_fault': node.real.fault = 'motor fault'
    elif blocked == 'guard_fault': node.real.guard = {'state': 'fault'}
    elif blocked == 'stale_guard': node.real.guard_time = 9.
    elif blocked == 'no_context': node.execution_context = None
    elif blocked == 'stopping': node._stopping = True
    elif blocked == 'other_error': error = RuntimeError('controller aborted')
    assert node.defer_stop_assessment(error) is (blocked is None)
    assert context.abort.is_set() is (blocked is None)
    assert canceled == ([True] if blocked is None else [])
    assert events == (['stop_assessment_requested'] if blocked is None else [])


@pytest.mark.parametrize('fresh_feedback', [True, False])
def test_scene_failure_cancels_then_confirms_encoders_or_escalates(
        operation, monkeypatch, fresh_feedback):
    from action_msgs.msg import GoalStatus
    from cleany_manipulation_bt.joint_feedback import JointFeedback
    from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
    from cleany_skill_executor.hardware_execution import HardwareExecution
    node, context, _ = operation
    clock, events = [10.], []
    monkeypatch.setattr('cleany_manipulation_bt.jetson_operations.time.monotonic', lambda: clock[0])
    node.execution_context, node._stopping = context, False
    node.event = lambda stage, **fields: events.append(stage)
    node.real = NS(node=node, fault='', guard={'state': 'ready'}, guard_time=10.,
                   value=lambda _: .25, fail_stop=lambda: events.append('latched_stop'))
    terminal = Future()
    def cancel():
        events.append('cancel')
        if not terminal.done(): terminal.set_result(NS(status=GoalStatus.STATUS_CANCELED))
    node.submissions, node.handles = [], [(NS(cancel_goal_async=cancel), terminal)]
    node._feedback, node._feedback_lock = JointFeedback(('joint',)), threading.Lock()
    values = {'bt_stop_feedback_max_age_sec': .5, 'bt_stop_timeout_sec': .5,
              'bt_stationary_velocity_rad_s': .02, 'bt_stationary_samples': 5,
              'bt_stationary_duration_sec': .25}
    node.value = values.__getitem__
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(clock[0]*1e9)))
    def tick(_):
        clock[0] += .05
        if fresh_feedback:
            node._feedback.update(['joint'], [0.], round(clock[0]*1e9), clock[0], positions=[.1])
    monkeypatch.setattr('cleany_manipulation_bt.jetson_operations.time.sleep', tick)
    HardwareExecution.stop_after_execution_error(
        node.real, InfrastructureError('Sensor planning scene unavailable'))
    assert context.abort.is_set() and events[0] == 'cancel'
    assert 'latched_stop' not in events
    node._stopping = True
    assert node._assess_stationary(context, cancel=True) is fresh_feedback
    assert context.stop_confirmed is fresh_feedback
    assert ('latched_stop' in events) is (not fresh_feedback)


def test_failed_action_cancel_request_immediately_escalates_to_independent_stop(operation):
    node, context, _ = operation
    stops, errors = [], []
    def broken_cancel():
        raise RuntimeError('cancel transport unavailable')
    node.handles = [(NS(cancel_goal_async=broken_cancel), Future())]
    node.real = NS(fail_stop=lambda: stops.append(True))
    node.get_logger = lambda: NS(error=errors.append)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    node.value = {'bt_stop_feedback_max_age_sec': .5, 'bt_stop_timeout_sec': .5,
                  'bt_stationary_velocity_rad_s': .02, 'bt_stationary_samples': 5,
                  'bt_stationary_duration_sec': .25}.__getitem__
    assert not node._assess_stationary(context, cancel=True)
    assert not context.stop_confirmed and stops == [True]
    assert 'cancel transport unavailable' in errors[0]


@pytest.mark.parametrize('frozen', [False, True])
def test_monitored_wait_preserves_explicit_sensor_scene_freeze(operation, frozen):
    node, _, _ = operation
    future, checks = Future(), []
    node.check_execution = lambda: None
    node._stopping = False
    node.carrying = None
    node.real = NS(sensor_scene_frozen=frozen, monitor=lambda: future.set_result('complete'))
    node.scene_monitor = NS(check=lambda: checks.append('scene'))
    assert node.wait(future, 1., monitor=True) == 'complete'
    assert checks == ([] if frozen else ['scene'])


def test_stop_wait_does_not_depend_on_sensor_scene_or_normal_motion_guard(operation, monkeypatch):
    node, _, _ = operation
    future = Future()
    node.check_execution = lambda: None
    node._stopping = True
    node.carrying = {'uncertain': True}
    node.real = NS(monitor=lambda: pytest.fail('Stop cannot depend on normal motion readiness'))
    node.scene_monitor = NS(check=lambda: pytest.fail('Stop cannot depend on scene freshness'))
    completion = NS(set=lambda: None, wait=lambda _: future.set_result('stopped'))
    monkeypatch.setattr('cleany_manipulation_bt.jetson_operations.threading.Event',
                        lambda: completion)
    assert node.wait(future, 1., monitor=True) == 'stopped'


def test_service_response_wakes_bt_wait_without_a_polling_delay(operation, monkeypatch):
    node, _, _ = operation
    future, notifications = Future(), []
    node.check_execution = lambda: None
    node._stopping, node.carrying = False, None
    completion = NS(set=lambda: notifications.append(True),
                    wait=lambda _: future.set_result('response'))
    monkeypatch.setattr('cleany_manipulation_bt.jetson_operations.threading.Event',
                        lambda: completion)
    monkeypatch.setattr('cleany_manipulation_bt.jetson_operations.time.sleep',
                        lambda _: pytest.fail('Do not sleep after the response'))
    assert node.wait(future, 1.) == 'response'
    assert notifications == [True]


@pytest.mark.parametrize('submitted', [True, False])
def test_response_timeout_preserves_late_action_acknowledgement(operation, submitted):
    node, _, _ = operation
    future = Future()
    node.check_execution = lambda: None
    node.submissions = [future] if submitted else []
    with pytest.raises(OperationError) as error:
        node.wait(future, 0.)
    assert error.value.error == Error.TIMEOUT
    assert future.cancelled() is (not submitted)


@pytest.mark.parametrize('invalid', [
    'approved_center_nan', 'approved_extent_nan', 'approved_extent_inf',
    'approved_extent_negative', 'approved_extent_zero', 'observed_center_nan',
    'observed_center_inf', 'wrong_frame', 'tolerance_nan', 'tolerance_negative',
])
def test_target_envelope_rejects_invalid_geometry_before_motion(operation, invalid):
    node, _, _ = operation
    center, extent = NS(x=.1, y=.2, z=.4), NS(x=.07, y=.07, z=.1)
    selected = NS(selected_object=NS(obb_pose=NS(position=center), obb_size=extent))
    observation = NS(header=NS(frame_id='base_link'), observed_center=deepcopy(center))
    node.get_parameter = lambda name: NS(value=.01)
    if invalid == 'approved_center_nan':
        center.x = math.nan
    elif invalid.startswith('approved_extent_'):
        extent.x = {'nan': math.nan, 'inf': math.inf, 'negative': -.1, 'zero': 0.}[invalid.rsplit('_', 1)[1]]
        if invalid.endswith('zero'):
            extent.y = extent.z = 0.
    elif invalid.startswith('observed_center_'):
        observation.observed_center.x = math.nan if invalid.endswith('nan') else math.inf
    elif invalid == 'wrong_frame':
        observation.header.frame_id = 'camera_depth_optical_frame'
    else:
        node.get_parameter = lambda name: NS(value=math.nan if invalid.endswith('nan') else -.01)
    with pytest.raises(OperationError):
        node._check_object_envelope(selected, observation)


def test_target_envelope_accepts_valid_planar_geometry_and_rejects_displacement(operation):
    node, _, _ = operation
    center = NS(x=.1, y=.2, z=.4)
    selected = NS(selected_object=NS(obb_pose=NS(position=center),
                                    obb_size=NS(x=.07, y=.1, z=0.)))
    observation = NS(header=NS(frame_id='base_link'), observed_center=deepcopy(center))
    node.get_parameter = lambda name: NS(value=.01)
    node._check_object_envelope(selected, observation)
    observation.observed_center.x += .5
    with pytest.raises(OperationError) as failed:
        node._check_object_envelope(selected, observation)
    assert failed.value.error == Error.STALE_TARGET


def test_target_envelope_failure_reports_distance_limit_and_comparison_coordinates(operation):
    node, _, _ = operation
    center = NS(x=.1, y=.2, z=.4)
    selected = NS(selected_object=NS(obb_pose=NS(position=center),
                                    obb_size=NS(x=.02, y=.02, z=.01)))
    observation = NS(header=NS(frame_id='base_link'),
                     observed_center=NS(x=.126, y=.2, z=.4))
    node.get_parameter = lambda name: NS(value=.01)
    with pytest.raises(OperationError) as failed:
        node._check_object_envelope(selected, observation)
    assert failed.value.error == Error.STALE_TARGET
    message = str(failed.value)
    assert 'distance=0.026000m allowed=0.025000m' in message
    assert 'object_radius=0.015000m margin=0.010000m' in message
    assert 'approved_center=(0.100000,0.200000,0.400000)' in message
    assert 'observed_center=(0.126000,0.200000,0.400000) frame=base_link' in message


@pytest.mark.parametrize('state', ['active', 'wrong_execution', 'completed', 'stopping', 'canceled'])
def test_selection_progress_only_updates_the_active_matching_operation(operation, state):
    _, context, _ = operation
    context.progress = 'SelectArmAndPath arm=right stage=1; elapsed=4.0s'
    updates = []
    record = NS(goal=context.goal, result=None, substage='SelectArmAndPath', message='BT started')
    if state == 'wrong_execution':
        record.goal = Goal('m', 't', 'another', 'collect_trash', 'approved', 2, 'trash_bin')
    elif state == 'completed':
        record.result = object()
    elif state == 'stopping':
        record.substage = 'StopAndAssess'
    elif state == 'canceled':
        context.abort.set()
    node = NS(operations=NS(execution_context=context),
              core=NS(record=record, _save=lambda **changes: updates.append(changes)))
    JetsonManipulationNode._update_operation_progress(node)
    assert updates == ([{'message': context.progress}] if state == 'active' else [])


@pytest.mark.parametrize('invalid', [
    '', 'reference', 'snapshot', 'object', 'replayed_stamp', 'future_stamp',
])
def test_reference_observation_checks_identity_and_capture_time(operation, invalid):
    node, context, _ = operation
    node.execution_context = context
    node.reference = object()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=200))
    response = ObserveObjectReference.Response(success=True,
        reference_id='pinned', source_snapshot_id=context.goal.snapshot_id,
        source_object_id=context.goal.object_id)
    response.header.stamp.nanosec = 150
    if invalid == 'reference':
        response.reference_id = 'other'
    elif invalid == 'snapshot':
        response.source_snapshot_id = 'other'
    elif invalid == 'object':
        response.source_object_id += 1
    elif invalid == 'replayed_stamp':
        response.header.stamp.nanosec = 100
    elif invalid == 'future_stamp':
        response.header.stamp.nanosec = 201
    node.call = lambda *args: response
    if invalid:
        with pytest.raises(OperationError) as failed:
            node.observe('pinned', 100)
        assert failed.value.error == Error.STALE_TARGET
    else:
        assert node.observe('pinned', 100) is response
