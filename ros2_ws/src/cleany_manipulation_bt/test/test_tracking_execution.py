from types import SimpleNamespace as NS
from pathlib import Path

import pytest
from cleany_interfaces.msg import DetectedObject2D, DetectedObject2DArray, DetectedObject3D, DetectedObject3DArray
from cleany_interfaces.srv import GetSceneSnapshot, RegisterPlacementTarget
from cleany_manipulation_bt.backend import ExecutionContext, OperationError
from cleany_manipulation_bt.ros_operations import MujocoOperations
from cleany_skill_executor.core.sorting import load_sorting_policy
from cleany_skill_executor.core.nearest_object import ObjectAttempt
from cleany_skill_executor.manipulation.models import Error, Goal
from cleany_skill_executor.nearest_pregrasp_coordinator import NearestPregraspCoordinator


def test_tracked_two_cups_are_allowed_but_uncertain_target_is_blocked():
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 'cached', 1, 'trash_right'))
    array = DetectedObject2DArray(snapshot_id='cached', tracking_session_id='desk',
                                 tracking_epoch='epoch', representative_frame='base_link', detections=[
        DetectedObject2D(object_id=i, label='cup', confidence=.9, distance_valid=True, distance_m=.7,
                         position_valid=True, tracking_state='TRACKED', track_id=track, sorting_category='trash',
                         sorting_reason='Empty test cup')
        for i, track in [(1, 'a'), (2, 'b')]])
    response = GetSceneSnapshot.Response(found=True, detections=array)
    node = NS(_snapshot=NS(call_async=lambda request: response), _future=lambda future, *_: future,
              _policy=load_sorting_policy(Path(__file__).parents[2] / 'cleany_skill_executor/config/table_sorting_policy.yaml'),
              _bins={'trash_right':object()}, get_parameter=lambda _: NS(value=['cup', 'crumpled tissue']))
    assert MujocoOperations.prepare(node, context).success
    assert node._approved_tracking == ('desk', 'epoch', 'a')
    assert not node._require_unique_target_label
    array.detections[0].tracking_state = 'AMBIGUOUS'
    with pytest.raises(OperationError) as raised:
        MujocoOperations.prepare(node, context)
    assert raised.value.error == Error.TARGET_UNAVAILABLE


@pytest.mark.parametrize('problem', ['other_cup', 'ambiguous', 'epoch'])
def test_reobservation_never_substitutes_another_cup(problem):
    detection = DetectedObject2D(object_id=1, label='cup', tracking_state='TRACKED', track_id='a')
    array = DetectedObject2DArray(tracking_session_id='desk', tracking_epoch='epoch', detections=[detection])
    if problem == 'other_cup':
        detection.track_id = 'b'
    elif problem == 'ambiguous':
        detection.tracking_state = 'AMBIGUOUS'
    else:
        array.tracking_epoch = 'restarted'
    node = NS(_approved_tracking=('desk', 'epoch', 'a'))
    with pytest.raises(ValueError, match='Approved track'):
        NearestPregraspCoordinator._validate_approved_tracking(node, array)


def test_registration_is_required_before_any_motion_and_verification_uses_returned_id():
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 'cached', 1, 'trash_right'))
    context.attempt = ObjectAttempt(1, 'cup', .9, .7)
    obj = DetectedObject3D(object_id=1, label='cup', track_id='a', tracking_state='TRACKED')
    array = DetectedObject3DArray(snapshot_id='cached', tracking_session_id='desk',
                                 tracking_epoch='epoch', objects=[obj])
    captured = []
    node = NS(_approved_tracking=('desk', 'epoch', 'a'),
              _inspect_selected=lambda *_: NS(objects=array), _future=lambda future, *_: future,
              _placement_registration=NS(call_async=lambda request: captured.append(request) or
                  RegisterPlacementTarget.Response(success=True, verification_id='individual')))
    assert MujocoOperations.reconstruct(node, context).success
    assert context.verification_id == 'individual'
    assert captured[0].execution_id == 'e' and captured[0].target.track_id == 'a'
    queries = []
    node._verification = NS(call_async=lambda request: queries.append(request) or NS(success=True, message='settled'))
    node.get_parameter = lambda _: NS(value=1.)
    context.target = NS(attempt=context.attempt)
    assert MujocoOperations._verify_after(node, context, 123).success
    assert queries[0].verification_id == 'individual' and queries[0].after_stamp_ns == 123
    node._placement_registration = NS(call_async=lambda _: RegisterPlacementTarget.Response(success=False, message='ambiguous'))
    with pytest.raises(OperationError) as raised:
        MujocoOperations.reconstruct(node, context)
    assert raised.value.error == Error.VERIFICATION_UNAVAILABLE
