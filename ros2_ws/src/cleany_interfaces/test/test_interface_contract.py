from cleany_interfaces.action import ExecuteManipulationSkill, InspectScene, SelectReachableGrasp
from cleany_interfaces.msg import (
    DetectedObject2D,
    DetectedObject2DArray,
    DetectedObject3D,
    DetectedObject3DArray,
    GraspCandidate,
    ObservedObjectGeometry,
    ManipulationExecutionRecord,
)
from cleany_interfaces.srv import GetManipulationExecution, PlanGrasp, VerifyPlacement


def test_manipulation_action_and_execution_lookup_contract() -> None:
    goal = ExecuteManipulationSkill.Goal()
    result = ExecuteManipulationSkill.Result()
    feedback = ExecuteManipulationSkill.Feedback()
    response = GetManipulationExecution.Response()
    assert goal.execution_id == goal.skill_name == goal.snapshot_id == ''
    assert goal.object_id == 0
    assert result.execution_profile == result.status == result.error_code == ''
    assert not result.stop_confirmed and not result.arm_recovered
    assert feedback.execution_id == feedback.stage == feedback.selected_arm == ''
    assert feedback.substage == result.failed_substage == ''
    assert not response.found and isinstance(response.record, ManipulationExecutionRecord)
    assert not response.record.has_result
    assert not response.record.human_confirmation_required
    assert response.record.record_state == '' and response.record.revision == 0
    assert response.record.substage == response.record.failed_substage == ''
    assert response.record.completed_substages == []


def test_observed_geometry_has_explicit_identity_and_empty_default_mesh():
    geometry = ObservedObjectGeometry()
    assert geometry.snapshot_id == geometry.header.frame_id == ''
    assert geometry.object_id == 0
    assert geometry.mesh.vertices == geometry.mesh.triangles == []






def test_placement_verification_requires_post_release_evidence():
    request = VerifyPlacement.Request()
    response = VerifyPlacement.Response()
    assert request.label == request.destination_id == ''
    assert request.after_stamp_ns == 0
    assert not response.success


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
    assert not detected.distance_valid
    assert detected.distance_m == 0.0
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
    assert goal.required_arm == ''
    assert result.ERROR_NO_REACHABLE_GRASP == 6
    assert result.selected_candidate_index == 0
    assert isinstance(result.selected_candidate, GraspCandidate)
    assert feedback.STAGE_PREGRASP_IK == 1
    assert feedback.STAGE_PLAN_GRASP == 5


def test_read_only_scene_snapshot_contract():
    from cleany_interfaces.srv import GetSceneSnapshot
    request, response = GetSceneSnapshot.Request(), GetSceneSnapshot.Response()
    assert request.snapshot_id == ''
    assert not response.found and isinstance(response.detections, DetectedObject2DArray)
    assert response.message == ''
