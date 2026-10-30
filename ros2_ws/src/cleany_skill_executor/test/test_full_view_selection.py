"""Exercise the real coordinator with typed synthetic responses, no devices."""
from concurrent.futures import Future
import json
import threading
import time
from types import MethodType, SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import InspectScene, SelectReachableGrasp, ValidatePick
from cleany_interfaces.msg import DetectedObject2D, DetectedObject3D, GraspCandidate
from cleany_interfaces.srv import ObserveObjectReference, PlanGrasp
from cleany_skill_executor.validate_pick_node import ValidatePickNode


def completed(value):
    future = Future()
    future.set_result(value)
    return future


def coordinator(tmp_path, *, failure_stage='select', failure_code=None,
                all_fail=False, clear_fails=False, transport_fails=False):
    parameters = dict(action_timeout_sec=120., inspection_timeout_sec=45.,
        grasp_timeout_sec=45., selection_timeout_sec=120., required_arm='left',
        pin_object_reference=True, workspace_roi_pixels=[-1.]*4,
        reference_cleanup_timeout_sec=5.,
        denylist=['keyboard'], maximum_smaller_extent_m=.095,
        evidence_output_directory=str(tmp_path))
    phase1 = InspectScene.Result(success=True)
    phase1.detections.snapshot_id = 'full-view-test'
    phase1.detections.detections = [DetectedObject2D(
        object_id=i, label='keyboard' if i == 1 else 'can',
        median_depth_m=.1*i, estimated_width_m=.04, estimated_height_m=.08,
        x_min=20. if i == 2 else 600., y_min=20.,
        x_max=40. if i == 2 else 620., y_max=40.) for i in (1, 2, 3, 4)]
    events, logs = [], []
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_logger=lambda: SimpleNamespace(info=logs.append, error=logs.append),
        _sensor_scene=SimpleNamespace(wait=lambda **kwargs: None),
        _feedback=lambda *args, **kwargs: None,
        _publish_filter_debug=lambda *args: None, _debug_messages=lambda: (None, None),
        _inspection=object(), _selection=object(),
        _active_downstream_goal=None, _goal_lock=threading.Lock(), _goal_active=True)
    for name in ('_validate_object', '_clear_reference', '_wait_future', '_wait_sensor_scene'):
        setattr(node, name, MethodType(getattr(ValidatePickNode, name), node))
    node._copy_selection_result = ValidatePickNode._copy_selection_result
    node._set_failure = ValidatePickNode._set_failure
    node._check_request = ValidatePickNode._check_request
    goal = SimpleNamespace(request=ValidatePick.Goal(), is_cancel_requested=False,
        succeed=lambda: events.append(('terminal', 'succeed')),
        abort=lambda: events.append(('terminal', 'abort')),
        canceled=lambda: events.append(('terminal', 'canceled')))

    def reference(request):
        operation = 'pin' if request.operation == request.PIN else 'clear'
        events.append((operation, request.source_object_id if operation == 'pin' else request.reference_id))
        response = ObserveObjectReference.Response(success=not (clear_fails and operation == 'clear'))
        response.reference_id = f'ref-{request.source_object_id}' if operation == 'pin' else request.reference_id
        response.source_snapshot_id = request.source_snapshot_id
        response.source_object_id = request.source_object_id
        response.message = 'synthetic cleanup failure' if not response.success else ''
        return completed(response)
    node._reference = SimpleNamespace(wait_for_service=lambda **kwargs: True, call_async=reference)

    def should_fail(stage, object_id):
        return stage == failure_stage and (all_fail or object_id == 2)

    def action(client, request, *args, **kwargs):
        if client is node._inspection:
            if not request.snapshot_id:
                events.append(('detect', 0))
                return phase1
            object_id = request.selected_object_id
            events.append(('inspect', object_id))
            result = InspectScene.Result(success=True)
            if should_fail('inspect', object_id):
                if transport_fails:
                    raise RuntimeError('synthetic action timeout')
                result.success = False
                result.error_code = failure_code or result.ERROR_MASK
                result.message = 'synthetic inspection failure'
            else:
                result.objects.objects = [DetectedObject3D(object_id=object_id, label='can')]
            return result
        object_id = request.candidates[0].object_id
        events.append(('select', object_id))
        result = SelectReachableGrasp.Result(success=True)
        if should_fail('select', object_id):
            if transport_fails:
                raise RuntimeError('synthetic action timeout')
            result.success = False
            result.error_code = failure_code or result.ERROR_NO_REACHABLE_GRASP
            result.message = 'synthetic selection failure'
        else:
            result.selected_candidate_index = 0
            result.selected_candidate = request.candidates[0]
            result.selected_arm = request.required_arm
        return result
    node._run_action = action

    def grasp(request, *args):
        events.append(('grasp', request.object_id))
        result = PlanGrasp.Response(success=True)
        if should_fail('grasp', request.object_id):
            result.success = False
            result.error_code = failure_code or result.ERROR_NO_GRASP_CANDIDATE
            result.message = 'synthetic grasp failure'
        else:
            result.candidates = [GraspCandidate(object_id=request.object_id, snapshot_id=request.snapshot_id)]
        return result
    node._run_grasp_service = grasp
    return node, goal, events, parameters


@pytest.mark.parametrize('stage', ['inspect', 'grasp', 'select'])
def test_object_local_failure_advances_to_next_edge_object(tmp_path, stage):
    node, goal, events, _ = coordinator(tmp_path, failure_stage=stage)
    result = ValidatePickNode._execute(node, goal)
    assert result.success
    assert result.selected_detection.object_id == 3
    assert result.candidates[0].object_id == 3
    assert ('inspect', 4) not in events
    if stage != 'inspect':
        assert events.index(('clear', 'ref-2')) < events.index(('inspect', 3))
    assert result.reference_id == 'ref-3'
    report = json.loads((tmp_path/'full-view-test/target_attempts.json').read_text())
    assert [item['status'] for item in report['target_attempts']] == [
        'excluded', 'plan_failed', 'selected', 'not_checked']
    assert report['target_attempts'][1]['elapsed_sec'] > 0
    evidence = json.loads((tmp_path/'full-view-test/result.json').read_text())
    assert evidence['selected_object_id'] == 3
    assert len(evidence['target_attempts']) == 4


@pytest.mark.parametrize('stage,code', [
    ('inspect', InspectScene.Result.ERROR_TF),
    ('inspect', InspectScene.Result.ERROR_SNAPSHOT_NOT_FOUND),
    ('grasp', PlanGrasp.Response.ERROR_MODEL_UNAVAILABLE),
    ('select', SelectReachableGrasp.Result.ERROR_JOINT_STATE_STALE),
    ('select', SelectReachableGrasp.Result.ERROR_PLANNING_SCENE),
    ('select', SelectReachableGrasp.Result.ERROR_MOVEIT_UNAVAILABLE),
    ('select', SelectReachableGrasp.Result.ERROR_INTERNAL),
])
def test_infrastructure_failure_stops_instead_of_trying_next(tmp_path, stage, code):
    node, goal, events, _ = coordinator(tmp_path, failure_stage=stage, failure_code=code)
    result = ValidatePickNode._execute(node, goal)
    assert not result.success
    assert ('inspect', 3) not in events
    assert not result.reference_id
    report = json.loads((tmp_path/'full-view-test/target_attempts.json').read_text())
    assert report['target_attempts'][1]['status'] == 'interrupted'
    assert report['target_attempts'][2]['status'] == 'not_checked'


def test_all_objects_failed_is_bounded_and_retains_report(tmp_path):
    node, goal, events, _ = coordinator(tmp_path, all_fail=True)
    result = ValidatePickNode._execute(node, goal)
    assert not result.success
    assert [item[1] for item in events if item[0] == 'inspect'] == [2, 3, 4]
    assert result.selected_candidate_index == -1
    assert not result.reference_id
    assert result.evidence_json_path.endswith('target_attempts.json')
    report = json.loads((tmp_path/'full-view-test/target_attempts.json').read_text())
    assert [item['status'] for item in report['target_attempts']][1:] == ['plan_failed']*3


@pytest.mark.parametrize('stage', ['inspect', 'select'])
def test_transport_timeout_is_fatal(tmp_path, stage):
    node, goal, events, _ = coordinator(tmp_path, failure_stage=stage, transport_fails=True)
    assert not ValidatePickNode._execute(node, goal).success
    assert ('inspect', 3) not in events


def test_cleanup_failure_stops_before_next_object(tmp_path):
    node, goal, events, _ = coordinator(tmp_path, clear_fails=True)
    result = ValidatePickNode._execute(node, goal)
    assert not result.success
    assert 'cleanup' in result.message.lower()
    assert ('inspect', 3) not in events


@pytest.mark.parametrize('code', [InspectScene.Result.ERROR_DEPTH, InspectScene.Result.ERROR_PLANE])
def test_object_reconstruction_failure_advances(tmp_path, code):
    node, goal, events, _ = coordinator(tmp_path, failure_stage='inspect', failure_code=code)
    result = ValidatePickNode._execute(node, goal)
    assert result.success and result.selected_detection.object_id == 3


def test_scene_stale_before_object_attempt_is_fatal(tmp_path):
    node, goal, events, _ = coordinator(tmp_path)
    calls = []
    def wait(**kwargs):
        calls.append(1)
        if len(calls) > 1:
            raise RuntimeError('stale map')
    node._sensor_scene.wait = wait
    result = ValidatePickNode._execute(node, goal)
    assert not result.success
    assert not any(item[0] == 'inspect' for item in events)


def test_cancel_between_objects_still_cleans_reference(tmp_path):
    node, goal, events, _ = coordinator(tmp_path)
    action = node._run_action
    def cancel_after_first_failure(client, request, *args, **kwargs):
        result = action(client, request, *args, **kwargs)
        if client is node._selection:
            goal.is_cancel_requested = True
        return result
    node._run_action = cancel_after_first_failure
    result = ValidatePickNode._execute(node, goal)
    assert not result.success and result.error_code == result.ERROR_CANCELED
    assert ('clear', 'ref-2') in events
    assert ('inspect', 3) not in events


@pytest.mark.parametrize('canceled', [False, True])
def test_expired_or_canceled_request_does_not_attempt_objects(tmp_path, canceled):
    node, goal, events, parameters = coordinator(tmp_path)
    if canceled:
        goal.is_cancel_requested = True
    else:
        parameters['action_timeout_sec'] = 0.
    result = ValidatePickNode._execute(node, goal)
    assert not result.success
    assert not any(item[0] == 'inspect' for item in events)


@pytest.mark.parametrize('status,code', [
    (GoalStatus.STATUS_ABORTED, SelectReachableGrasp.Result.ERROR_NO_REACHABLE_GRASP),
    (GoalStatus.STATUS_ABORTED, SelectReachableGrasp.Result.ERROR_JOINT_STATE_STALE),
    (GoalStatus.STATUS_CANCELED, SelectReachableGrasp.Result.ERROR_CANCELED),
    (GoalStatus.STATUS_ABORTED, 0),
])
def test_real_action_wrapper_preserves_typed_aborts_and_stops_cancellation(status, code):
    response = SelectReachableGrasp.Result(success=False, error_code=code)
    downstream = SimpleNamespace(accepted=True,
        get_result_async=lambda: completed(SimpleNamespace(status=status, result=response)),
        cancel_goal_async=lambda: None)
    client = SimpleNamespace(wait_for_server=lambda **kwargs: True,
        send_goal_async=lambda *args, **kwargs: completed(downstream))
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=1.),
        _active_downstream_goal=None)
    node._wait_future = MethodType(ValidatePickNode._wait_future, node)
    node._check_request = ValidatePickNode._check_request
    goal = SimpleNamespace(is_cancel_requested=False)
    args = (node, client, None, 1., goal, time.monotonic()+2., 'test')
    if status == GoalStatus.STATUS_CANCELED:
        with pytest.raises(InterruptedError):
            ValidatePickNode._run_action(*args)
    elif code == 0:
        with pytest.raises(RuntimeError):
            ValidatePickNode._run_action(*args)
    else:
        assert ValidatePickNode._run_action(*args) is response
