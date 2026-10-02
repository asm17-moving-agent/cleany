from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from threading import Event

import pytest
from cleany_interfaces.msg import DetectedObject2D, DetectedObject2DArray
from cleany_interfaces.srv import GetSceneSnapshot
from cleany_manipulation_bt.backend import ExecutionContext, Observation, OperationError, WorkerBackend
from cleany_manipulation_bt.ros_operations import MujocoOperations
from cleany_skill_executor.core.sorting import load_sorting_policy
from cleany_skill_executor.manipulation.models import Error, Goal


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
        get_logger=lambda: NS(info=messages.append, warning=messages.append))
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
              'arm_stationary_velocity_rad_s': .02}
    node = NS(get_parameter=lambda name: NS(value=values[name]),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=100)),
              _joint_stamp_ns=100, _last_joint_wall=10.,
              _joint_velocities={name: 0. for name in REQUIRED_JOINT_NAMES},
              submissions=[], handles=[], quiescent=lambda: problem != 'unsettled_action')
    def spin(*_, **__):
        now[0] += .05
        if problem not in ('paused', 'disconnected'):
            node._joint_stamp_ns += 1
        if problem != 'disconnected':
            node._last_joint_wall = now[0]
        if problem == 'moving':
            node._joint_velocities[REQUIRED_JOINT_NAMES[0]] = .1
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    confirmed = MujocoOperations._assess_stationary(node, context, cancel=True)
    assert confirmed == (problem == 'none')



@pytest.mark.parametrize('fault,error', [('paused', Error.HARDWARE_ERROR),
                                        ('feedback', Error.HARDWARE_ERROR), ('held', Error.GRASP_LOST)])
def test_running_motion_guard_stops_on_clock_feedback_or_retention_loss(monkeypatch, fault, error):
    import cleany_manipulation_bt.ros_operations as module
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    node = NS(execution_context=context, current_node='CarryObject', deadline=20.,
              _last_joint_wall=9.9, _clock_value=10**9, _clock_progress_wall=9.9,
              _joint_stamp_ns=10**9, _held_object=object(),
              get_parameter=lambda _: NS(value=2.), get_clock=lambda: NS(now=lambda: NS(nanoseconds=10**9)))
    if fault == 'paused':
        node._clock_progress_wall = 5.
    elif fault == 'feedback':
        node._last_joint_wall = 5.
    else:
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
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.)
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))
    node = NS(execution_context=context, current_node='MoveToPregrasp', deadline=20.,
              _last_joint_wall=9.9, _clock_value=10**9, _clock_progress_wall=9.9,
              _joint_stamp_ns=1_010_000_000,
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
