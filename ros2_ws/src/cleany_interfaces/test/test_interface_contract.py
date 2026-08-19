from cleany_interfaces.action import InspectScene, SelectReachableGrasp
from cleany_interfaces.msg import (
    DetectedObject2D,
    DetectedObject2DArray,
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


def test_detected_object_2d_defaults() -> None:
    detected = DetectedObject2D()
    detected_array = DetectedObject2DArray()

    assert detected.object_id == 0
    assert detected.label == ''
    assert detected.x_min == 0.0
    assert detected_array.snapshot_id == ''
    assert detected_array.detections == []


def test_detected_object_array_carries_snapshot_context() -> None:
    detected_array = DetectedObject3DArray()

    assert detected_array.header.frame_id == ''
    assert detected_array.snapshot_id == ''
    assert detected_array.objects == []


def test_inspect_scene_contract_constants_and_payloads() -> None:
    goal = InspectScene.Goal()
    result = InspectScene.Result()
    feedback = InspectScene.Feedback()

    assert goal.query == ''
    assert {
        'none': result.ERROR_NONE,
        'rgbd_timeout': result.ERROR_RGBD_TIMEOUT,
        'detector_api': result.ERROR_DETECTOR_API,
        'detector_response': result.ERROR_DETECTOR_RESPONSE,
        'mask': result.ERROR_MASK,
        'depth': result.ERROR_DEPTH,
        'plane': result.ERROR_PLANE,
        'tf': result.ERROR_TF,
        'cancelled': result.ERROR_CANCELLED,
        'snapshot_not_found': result.ERROR_SNAPSHOT_NOT_FOUND,
        'invalid_selection': result.ERROR_INVALID_SELECTION,
        'internal': result.ERROR_INTERNAL,
    } == {
        'none': 0,
        'rgbd_timeout': 1,
        'detector_api': 2,
        'detector_response': 3,
        'mask': 4,
        'depth': 5,
        'plane': 6,
        'tf': 7,
        'cancelled': 8,
        'snapshot_not_found': 9,
        'invalid_selection': 10,
        'internal': 255,
    }
    assert isinstance(result.objects, DetectedObject3DArray)
    assert isinstance(result.detections, DetectedObject2DArray)
    assert goal.snapshot_id == ''
    assert goal.selected_object_id == 0
    assert feedback.STAGE_WAITING_FOR_RGBD == 0
    assert feedback.STAGE_TRANSFORMING == 4


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


def test_inspect_scene_selected_cloud_defaults() -> None:
    result = InspectScene.Result()

    assert len(result.target_cloud.data) == 0
    assert len(result.context_cloud.data) == 0
