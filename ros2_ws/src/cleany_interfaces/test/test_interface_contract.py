from cleany_interfaces.action import SelectReachableGrasp
from cleany_interfaces.msg import (
    DetectedObject3D,
    DetectedObject3DArray,
    GraspCandidate,
)
from cleany_interfaces.srv import PlanGrasp


def test_detected_object_3d_defaults() -> None:
    detected = DetectedObject3D()

    assert detected.object_id == 0
    assert detected.label == ''
    assert detected.confidence == 0.0
    assert detected.obb_pose.orientation.w == 1.0
    assert detected.obb_size.x == 0.0


def test_detected_object_array_carries_snapshot_context() -> None:
    detected_array = DetectedObject3DArray()

    assert detected_array.header.frame_id == ''
    assert detected_array.snapshot_id == ''
    assert detected_array.objects == []


def test_plan_grasp_contract_constants_and_payloads() -> None:
    request = PlanGrasp.Request()
    response = PlanGrasp.Response()

    assert isinstance(request.target_object, DetectedObject3D)
    assert request.target_cloud.header.frame_id == ''
    assert request.context_cloud.header.frame_id == ''
    assert {
        'none': response.ERROR_NONE,
        'invalid_request': response.ERROR_INVALID_REQUEST,
        'model_unavailable': response.ERROR_MODEL_UNAVAILABLE,
        'invalid_input': response.ERROR_INVALID_INPUT,
        'no_grasp_candidate': response.ERROR_NO_GRASP_CANDIDATE,
        'internal': response.ERROR_INTERNAL,
    } == {
        'none': 0,
        'invalid_request': 1,
        'model_unavailable': 2,
        'invalid_input': 3,
        'no_grasp_candidate': 4,
        'internal': 255,
    }
    assert response.candidates == []


def test_select_reachable_grasp_contract() -> None:
    goal = SelectReachableGrasp.Goal()
    result = SelectReachableGrasp.Result()
    feedback = SelectReachableGrasp.Feedback()

    assert goal.candidates == []
    assert result.ERROR_NO_REACHABLE_GRASP == 6
    assert result.selected_candidate_index == 0
    assert isinstance(result.selected_candidate, GraspCandidate)
    assert feedback.STAGE_PREGRASP_IK == 1
    assert feedback.STAGE_PLAN_GRASP == 5
