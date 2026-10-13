from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from threading import Event

from geometry_msgs.msg import Pose
import numpy as np
import pytest
from cleany_interfaces.msg import DetectedObject2D, DetectedObject2DArray
from cleany_interfaces.srv import GetSceneSnapshot
from cleany_manipulation_bt.backend import ExecutionContext, Observation, OperationError, WorkerBackend
from cleany_manipulation_bt.joint_feedback import JointFeedback
from cleany_manipulation_bt.ros_operations import MujocoOperations
from cleany_skill_executor.core.sorting import load_sorting_policy
from cleany_skill_executor.manipulation.models import Error, Goal, ObjectState
from cleany_skill_executor.sorting_coordinator import SortingCoordinator


@pytest.fixture
def joint_receiver(monkeypatch):
    from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
    from sensor_msgs.msg import JointState

    node = object.__new__(MujocoOperations)
    node._joint_positions, node._joint_velocities = {}, {}
    node._joint_feedback = JointFeedback(REQUIRED_JOINT_NAMES)
    wall = [10.0]
    monkeypatch.setattr('cleany_manipulation_bt.ros_operations.time.monotonic', lambda: wall[0])
    message = JointState(name=list(REQUIRED_JOINT_NAMES),
                         position=[0.] * len(REQUIRED_JOINT_NAMES),
                         velocity=[0.] * len(REQUIRED_JOINT_NAMES))
    message.header.stamp.sec = 1
    node._on_joints(message)
    return node, message, wall


def test_identical_joint_delivery_keeps_original_freshness_and_stationary_count(joint_receiver):
    from copy import deepcopy
    from cleany_manipulation_bt.joint_feedback import StationaryWindow

    node, message, wall = joint_receiver
    previous = dict(node._joint_feedback.samples)
    window = StationaryWindow(barrier_ns=1, started_at=9., max_age_sec=.5,
        maximum_velocity=.02, required_samples=3, minimum_duration_sec=.25)
    assert not window.observe(node._joint_feedback, now=wall[0], clock_ns=10**9, terminal=True)
    for instant in (10.1, 10.2, 10.3):
        wall[0] = instant
        node._on_joints(deepcopy(message))
        assert node._joint_feedback.samples == previous
        assert not node._joint_feedback.invalid_joints(instant, 10**9, .5)
        assert not window.observe(node._joint_feedback, now=instant, clock_ns=10**9, terminal=True)
    assert window.samples == 1
    assert set(node._joint_feedback.invalid_joints(10.6, 10**9, .5)) == set(message.name)


@pytest.mark.parametrize('fault', ['position', 'velocity', 'backward', 'missing_velocity'])
def test_conflicting_or_invalid_joint_delivery_still_invalidates_feedback(joint_receiver, fault):
    from copy import deepcopy

    node, original, wall = joint_receiver
    message = deepcopy(original)
    if fault == 'position':
        message.position[0] = .1
    elif fault == 'velocity':
        message.velocity[0] = .1
    elif fault == 'backward':
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 900_000_000
    else:
        message.velocity = []
    wall[0] += .1
    node._on_joints(message)
    assert set(node._joint_feedback.invalid_joints(wall[0], 10**9, .5)) == set(message.name)


@pytest.fixture
def release_target():
    node = object.__new__(MujocoOperations)
    pose = Pose()
    pose.orientation.w = 1.0
    point = np.array([-.075, -.105, .50])
    pose.position.x, pose.position.y, pose.position.z = point
    node._tcp_pose = lambda _: pose
    node._fixed_release_points = {'bin': point}
    node._bins = {'bin': NS(center_xy=(-.075, -.105),
                           outside_size=(.18, .17, .12), wall=.008, top_z=.34)}
    parameters = dict(sorting_release_clearance_m=.06,
                      sorting_release_maximum_clearance_m=.21,
                      sorting_release_edge_margin_m=.005)
    node.get_parameter = lambda name: NS(value=parameters[name])
    held = NS(offset_in_tcp=np.zeros(3), selected=NS(selected_arm='left',
              selected_candidate=NS(target_object=NS(obb_size=NS(x=.04, y=.04, z=.04)))))
    return node, held, pose


@pytest.mark.parametrize('radius', [float('nan'), float('inf'), 0.0, -0.01])
def test_action_release_rejects_invalid_payload_radius(release_target, monkeypatch, radius):
    node, held, _ = release_target
    monkeypatch.setattr('cleany_manipulation_bt.ros_operations.held_bounding_radius',
                        lambda *args: radius)
    with pytest.raises(RuntimeError, match='radius must be positive and finite'):
        node.check_release_target(held, 'bin')


@pytest.mark.parametrize('problem', [None, 'too_large', 'below_rim', 'outside_bin'])
def test_action_release_checks_actual_payload_geometry(release_target, problem):
    node, held, pose = release_target
    if problem == 'too_large':
        held.selected.selected_candidate.target_object.obb_size.x = .20
    elif problem == 'below_rim':
        pose.position.z = .35
    elif problem == 'outside_bin':
        pose.position.x += .07
    if problem:
        with pytest.raises(RuntimeError, match='bin opening'):
            node.check_release_target(held, 'bin')
    else:
        node.check_release_target(held, 'bin')


@pytest.mark.parametrize('problem', [None, 'too_large', 'low_point'])
def test_action_carry_validates_fit_before_motion(release_target, monkeypatch, problem):
    node, held, _ = release_target
    motions = []
    monkeypatch.setattr(SortingCoordinator, 'transport',
                        lambda *args: motions.append('carry'))
    if problem == 'too_large':
        held.selected.selected_candidate.target_object.obb_size.x = .20
    elif problem == 'low_point':
        node._fixed_release_points['bin'][2] = .35
    if problem:
        error = ValueError if problem == 'too_large' else RuntimeError
        message = 'does not fit inside bin opening' if problem == 'too_large' else 'does not fit payload'
        with pytest.raises(error, match=message):
            node.transport(held, 'bin')
        assert motions == []
    else:
        node.transport(held, 'bin')
        assert motions == ['carry']


def test_action_cancel_interrupts_held_contact_confirmation(monkeypatch):
    node = object.__new__(MujocoOperations)
    node.execution_context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'bin'))
    node.execution_context.abort.set()
    node.current_node = 'ConfirmHeld'
    node.get_parameter = lambda _: NS(value=.5)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10**9))
    monkeypatch.setattr('cleany_manipulation_bt.ros_operations.rclpy.spin_once',
                        lambda *a, **k: pytest.fail('Cancellation must interrupt feedback waiting'))
    held = NS(selected=NS(selected_arm='left'))
    with pytest.raises(OperationError) as raised:
        node._confirm_held_contact(held)
    assert raised.value.error == Error.CANCELED


@pytest.mark.parametrize('retry_enabled,expected_submissions', [(False, 1), (True, 2)])
def test_controller_fault_stops_without_retry_in_bt_profile(retry_enabled, expected_submissions):
    from action_msgs.msg import GoalStatus
    from moveit_msgs.msg import MoveItErrorCodes
    from cleany_skill_executor.grasp_execution import GraspExecutionNode
    submissions, messages = [], []
    handle = NS(accepted=True, get_result_async=lambda: NS(
        status=GoalStatus.STATUS_ABORTED,
        result=NS(error_code=NS(val=MoveItErrorCodes.CONTROL_FAILED))))
    node = NS(_controller_retry_enabled=retry_enabled,
        _execution_goal=lambda *_: object(),
        _move_group=NS(send_goal_async=lambda goal: submissions.append(goal) or handle),
        _future=lambda future, *_: future, _log_joint_tracking_error=lambda *_: None,
        get_logger=lambda: NS(info=messages.append, debug=messages.append, warning=messages.append))
    with pytest.raises(RuntimeError, match='execution failed: status=6 code=-4'):
        GraspExecutionNode._move_to(node, 'right', object(), 'pregrasp')
    assert len(submissions) == expected_submissions
    assert any('replanning once' in message for message in messages) == retry_enabled


@pytest.fixture
def preparation():
    goal = Goal('m', 't', 'e', 'collect_trash', 'cached', 1, 'trash_right')
    context = ExecutionContext(goal)
    detection = DetectedObject2D(object_id=1, label='cup', confidence=.9,
        distance_valid=True, distance_m=.7, sorting_category='trash', sorting_reason='Empty paper cup')
    response = GetSceneSnapshot.Response(found=True, detections=DetectedObject2DArray(
        snapshot_id='cached', detections=[detection]))
    path = Path(__file__).parents[2] / 'cleany_skill_executor/config/table_sorting_policy.yaml'
    node = NS(_snapshot=NS(call_async=lambda request: response),
              _future=lambda future, *_: future, _policy=load_sorting_policy(path),
              _bins={'trash_right': object()},
              get_parameter=lambda _: NS(value=['cup', 'crumpled tissue']))
    return node, context, response


@pytest.mark.parametrize('problem,error', [
    ('expired', Error.STALE_TARGET), ('object', Error.TARGET_UNAVAILABLE),
    ('destination', Error.DESTINATION_UNAVAILABLE), ('duplicate', Error.TARGET_UNAVAILABLE),
    ('unsupported', Error.TARGET_UNAVAILABLE), ('classification', Error.TARGET_UNAVAILABLE),
    ('depth', Error.TARGET_UNAVAILABLE),
])
def test_preparation_vetoes_invalid_targets_before_motion(preparation, problem, error):
    node, context, response = preparation
    detection = response.detections.detections[0]
    if problem == 'expired':
        response.found = False
    elif problem == 'object':
        context.goal = replace(context.goal, object_id=2)
    elif problem == 'destination':
        context.goal = replace(context.goal, destination_id='missing')
    elif problem == 'duplicate':
        response.detections.detections.append(DetectedObject2D(object_id=2, label='cup'))
    elif problem == 'unsupported':
        detection.label = 'mouse'
    elif problem == 'classification':
        detection.sorting_category = 'review'
    else:
        detection.distance_valid = False
    with pytest.raises(OperationError) as raised:
        MujocoOperations.prepare(node, context)
    assert raised.value.error == error
    assert context.selected is None and context.held is None


def test_prepare_preserves_original_approved_identity(preparation):
    node, context, _ = preparation
    assert MujocoOperations.prepare(node, context).success
    assert context.goal.snapshot_id == 'cached' and context.goal.object_id == 1
    assert context.attempt.label == 'cup'


@pytest.mark.parametrize('label', ['mouse', 'computer mouse', 'wireless mouse', 'lego brick'])
def test_lost_item_preparation_checks_skill_and_bin_before_motion(preparation, label):
    node, context, response = preparation
    detection = response.detections.detections[0]
    detection.label, detection.sorting_category = label, 'lost_item'
    context.goal = replace(context.goal, skill_name='collect_lost_item', destination_id='lost_items_left')
    node.get_parameter = lambda _: NS(value=['mouse', 'computer mouse', 'wireless mouse', 'lego brick'])
    node._bins['lost_items_left'] = object()
    assert MujocoOperations.prepare(node, context).success
    for goal, error in [(replace(context.goal, skill_name='collect_trash'), Error.TARGET_UNAVAILABLE),
                        (replace(context.goal, destination_id='trash_right'), Error.DESTINATION_UNAVAILABLE)]:
        context.goal = goal
        with pytest.raises(OperationError) as raised:
            MujocoOperations.prepare(node, context)
        assert raised.value.error == error
        assert context.selected is None and context.held is None
        context.goal = replace(context.goal, skill_name='collect_lost_item', destination_id='lost_items_left')


@pytest.mark.parametrize('category,skill,destination,label', [
    ('trash', 'collect_trash', 'trash_right', 'cup'),
    ('lost_item', 'collect_lost_item', 'lost_items_left', 'mouse'),
])
@pytest.mark.parametrize('changed', ['category', 'confidence', 'reason', 'destination'])
def test_first_motion_is_blocked_when_reobserved_approval_changes(preparation, category, skill, destination, label, changed):
    from cleany_skill_executor.core.nearest_object import ObjectAttempt
    node, context, _ = preparation
    context.goal = replace(context.goal, skill_name=skill, destination_id=destination)
    context.attempt = ObjectAttempt(1, label, .9, .7, category, 'Approved test object')
    refreshed = replace(context.attempt,
        **{'category': {'sorting_category': 'lost_item' if category == 'trash' else 'trash'},
           'confidence': {'confidence': .001}, 'reason': {'sorting_reason': ''},
           'destination': {}}[changed])
    if changed == 'destination':
        node._policy = replace(node._policy, **{
            'trash_destination' if category == 'trash' else 'lost_item_destination': 'changed_bin'})
    commands = []
    node._wrist_enabled = False
    context.selected = NS(selected_arm='right')
    node._refresh_selected_grasp = lambda *_args, **_kwargs: (context.selected, refreshed)
    node._execute_pregrasp = lambda *_: commands.append('pregrasp')
    node.check_payload_fit = lambda *_: commands.append('payload_fit')
    with pytest.raises(OperationError) as raised:
        MujocoOperations.pregrasp(node, context)
    assert raised.value.error == Error.STALE_TARGET
    assert commands == []


def test_first_pregrasp_hands_off_refreshed_scene_before_combined_motion(preparation):
    from sensor_msgs.msg import JointState
    from cleany_skill_executor.core.grasp_selection import ARM_JOINT_NAMES
    from cleany_skill_executor.core.nearest_object import ObjectAttempt

    fixture, context, _ = preparation
    node = object.__new__(MujocoOperations)
    node._policy = fixture._policy
    node._wrist_enabled = False
    node.get_parameter = lambda name: NS(value=['cup'] if name.startswith('bt_supported_') else 1.2)
    node.get_logger = lambda: NS(info=lambda _: None)
    context.attempt = ObjectAttempt(1, 'cup', .9, .7, 'trash', 'Disposable cup')
    candidate = NS(snapshot_id='refreshed')
    selected = NS(selected_arm='right', selected_candidate=candidate,
                  pregrasp_joint_state=JointState(name=list(ARM_JOINT_NAMES['right']), position=[0.] * 5))
    context.selected = selected
    state = NS(active=False, contacts=False)
    events = []

    def begin(*_):
        if state.active:
            raise RuntimeError('planning-scene transaction is already active')
        state.active = True
        events.append('begin')

    def restore():
        state.active = state.contacts = False
        events.append('restore')

    def contacts(allowed):
        state.contacts = allowed
        events.append('allow' if allowed else 'forbid')

    node._execution_scene = NS(begin=begin, restore=restore,
        allow_contacts_for=lambda _: contacts(True), disallow_target_contacts=lambda: contacts(False))

    def refresh(*_, **kwargs):
        assert kwargs['match_label']
        begin()
        contacts(True)
        return selected, context.attempt

    def move(arm, joints, label):
        assert state.active and not state.contacts
        assert arm == 'right' and joints.name[-1] == 'right_gripper_joint'
        assert joints.position[-1] == 1.2
        events.append('move')

    node._refresh_selected_grasp = refresh
    node.check_payload_fit = lambda _: events.append('fit')
    node._move_to = move
    node._verify_feedback = lambda _: events.append('verify')
    assert node.pregrasp(context).success
    assert events == ['begin', 'allow', 'fit', 'restore', 'begin', 'forbid', 'move', 'verify']
    assert selected.pregrasp_joint_state.name == list(ARM_JOINT_NAMES['right'])


def test_worker_quiescence_blocks_next_context_and_late_response_is_scoped():
    entered, release = Event(), Event()
    contexts = []
    def operation(_, context):
        contexts.append(context)
        entered.set()
        assert release.wait(2.)
        context.selected = 'late observation'
        return Observation()
    backend = WorkerBackend(operation)
    goal = Goal('m', 't', 'old', 'collect_trash', 'snapshot', 1, 'trash_right')
    backend.begin(goal)
    token = backend.start('PrepareTarget', 'old')
    assert entered.wait(1.)
    backend.cancel(token)
    with pytest.raises(RuntimeError, match='quiesced'):
        backend.begin(replace(goal, execution_id='new'))
    release.set()
    backend.pending[token][0].result(timeout=1.)
    backend.begin(replace(goal, execution_id='new'))
    assert contexts[0].abort.is_set()
    assert backend.context.selected is None and backend.context.goal.execution_id == 'new'
    backend.close()


def test_completed_acceptance_is_not_quiescent_until_its_terminal_handle_is_tracked():
    from concurrent.futures import Future
    from action_msgs.msg import GoalStatus
    accepted, terminal = Future(), Future()
    handle = NS(accepted=True)
    accepted.set_result(handle)
    node = NS(submissions=[accepted], handles=[])
    assert not MujocoOperations.quiescent(node)
    node.handles.append((handle, terminal))
    assert not MujocoOperations.quiescent(node)
    terminal.set_result(NS(status=GoalStatus.STATUS_CANCELED))
    assert MujocoOperations.quiescent(node)


@pytest.mark.parametrize('problem', ['moving', 'paused', 'disconnected', 'unsettled_action', 'none'])
def test_stop_requires_terminal_actions_and_new_stationary_samples(monkeypatch, problem):
    import cleany_manipulation_bt.ros_operations as module
    from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
    now = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    values = {'bt_stop_timeout_sec': 1., 'arm_stationary_samples': 3,
              'arm_stationary_velocity_rad_s': .02,
              'bt_stop_feedback_max_age_sec': .5, 'bt_stationary_duration_sec': .1}
    feedback = JointFeedback(REQUIRED_JOINT_NAMES)
    feedback.update(REQUIRED_JOINT_NAMES, [0.] * len(REQUIRED_JOINT_NAMES), 100, 10.)
    node = NS(get_parameter=lambda name: NS(value=values[name]),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100)),
              _joint_feedback=feedback,
              submissions=[], handles=[], quiescent=lambda: problem != 'unsettled_action')
    stamp = [100]
    def spin(*_, **__):
        now[0] += .05
        if problem not in ('paused', 'disconnected'):
            stamp[0] += 1
        if problem != 'disconnected':
            velocities = [0.] * len(REQUIRED_JOINT_NAMES)
            if problem == 'moving':
                velocities[0] = .1
            feedback.update(REQUIRED_JOINT_NAMES, velocities, stamp[0], now[0])
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    confirmed = MujocoOperations._assess_stationary(node, context, cancel=True)
    assert confirmed == (problem == 'none')



@pytest.mark.parametrize('fault,error', [('paused', Error.HARDWARE_ERROR),
                                        ('feedback', Error.HARDWARE_ERROR), ('held', Error.GRASP_LOST)])
def test_running_motion_guard_stops_on_clock_feedback_or_retention_loss(monkeypatch, fault, error):
    import cleany_manipulation_bt.ros_operations as module
    from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    feedback = JointFeedback(REQUIRED_JOINT_NAMES)
    feedback.update(REQUIRED_JOINT_NAMES, [0.] * len(REQUIRED_JOINT_NAMES), 10**9,
                    5. if fault == 'feedback' else 9.9)
    node = NS(execution_context=context, current_node='CarryObject', deadline=20.,
              _joint_feedback=feedback, _clock_value=10**9, _clock_progress_wall=9.9,
              _held_object=object(),
              get_parameter=lambda _: NS(value=2.), get_clock=lambda: NS(now=lambda: NS(nanoseconds=10**9)))
    if fault == 'paused':
        node._clock_progress_wall = 5.
    elif fault == 'held':
        def lost(_):
            raise RuntimeError('contact lost')
        node._require_held_contact = lost
    with pytest.raises(OperationError) as raised:
        MujocoOperations._guard(node)
    assert raised.value.error == error


def test_ros_operation_node_constructs_without_shadowing_rclpy_context():
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'plan_only:=false'])
    node = None
    try:
        node = MujocoOperations()
        assert node.execution_context is None and node.context is rclpy.get_default_context()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_small_cross_topic_stamp_skew_does_not_masquerade_as_stale_feedback(monkeypatch):
    import cleany_manipulation_bt.ros_operations as module
    from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    feedback = JointFeedback(REQUIRED_JOINT_NAMES)
    feedback.update(REQUIRED_JOINT_NAMES, [0.] * len(REQUIRED_JOINT_NAMES), 1_010_000_000, 9.9)
    node = NS(execution_context=context, current_node='MoveToPregrasp', deadline=20.,
              _joint_feedback=feedback, _clock_value=10**9, _clock_progress_wall=9.9,
              get_parameter=lambda _: NS(value=2.), get_clock=lambda: NS(now=lambda: NS(nanoseconds=10**9)))
    MujocoOperations._guard(node)


@pytest.mark.parametrize('label', ['paper cup', 'disposable paper cup'])
def test_explicit_cup_alias_is_unique_and_keeps_raw_approved_observation(preparation, label):
    from cleany_manipulation_bt.target_identity import label_key, VERIFICATION_LABELS
    node, context, response = preparation
    response.detections.detections[0].label = label
    node.get_parameter = lambda _: NS(value=list(VERIFICATION_LABELS))
    assert MujocoOperations.prepare(node, context).success
    assert context.attempt.label == label and context.goal.object_id == 1
    assert label_key(context.attempt.label) == 'cup'
    response.detections.detections.append(DetectedObject2D(object_id=2, label='cup'))
    with pytest.raises(OperationError):
        MujocoOperations.prepare(node, context)


@pytest.mark.parametrize('outside_y,available', [(.17, False), (.18, True)])
def test_observed_payload_fit_is_checked_before_pregrasp(outside_y, available):
    import numpy as np
    values = {'sorting_release_clearance_m': .06, 'sorting_release_maximum_clearance_m': .21,
              'sorting_release_edge_margin_m': .005}
    candidate = NS(target_object=NS(obb_size=NS(x=.1111, y=.0713, z=.0684)))
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    context.selected = NS(selected_candidate=candidate)
    node = NS(_geometry_subscription=None, _fixed_release_points={'trash_right': np.array([-.075, -.105, .5])},
              _bins={'trash_right': NS(center_xy=(-.075, -.105), outside_size=(.18, outside_y, .12),
                                      wall=.008, top_z=.34)},
              get_parameter=lambda name: NS(value=values[name]))
    if available:
        MujocoOperations.check_payload_fit(node, context)
    else:
        with pytest.raises(OperationError) as raised:
            MujocoOperations.check_payload_fit(node, context)
        assert raised.value.error == Error.DESTINATION_UNAVAILABLE


@pytest.mark.parametrize('held', [False, True])
def test_cancel_release_opens_at_current_pose_without_bin_checks(held):
    events = []
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    context.selected = NS(selected_arm='right')
    context.gripper_engaged = held
    context.held = object() if held else None
    node = NS(_held_object=context.held, _open_gripper=lambda arm: events.append(('open', arm)),
              _hold=lambda _: events.append(('settle',)),
              _execution_scene=NS(restore=lambda: events.append(('detach',))))
    result = MujocoOperations.release_in_place(node, context)
    assert events == ([('open', 'right'), ('settle',)] if held else []) + [('detach',)]
    assert result.object_state == (ObjectState.LEFT_GRIPPER if held else None)
    assert result.placement_state is None
    assert node._held_object is None and context.held is None and not context.gripper_engaged


def test_cancel_return_uses_current_state_and_startup_posture_without_destination_release():
    from sensor_msgs.msg import JointState
    target = JointState(name=['right_shoulder_pan_joint'], position=[.1])
    events = []
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    context.selected = NS(selected_arm='right')
    assert not context.release_confirmed
    node = NS(_home={'right': target},
              _move_to=lambda arm, joints, label: events.append(('move', arm, joints, label)),
              _verify_feedback=lambda joints: events.append(('verify', joints)))
    result = MujocoOperations.return_after_cancel(node, context)
    assert events == [('move', 'right', target, 'return from cancellation'), ('verify', target)]
    assert context.arm_recovered and result.arm_recovered
