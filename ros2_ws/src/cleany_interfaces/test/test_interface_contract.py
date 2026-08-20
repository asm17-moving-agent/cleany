from cleany_interfaces.action import InspectScene
from cleany_interfaces.msg import DetectedObject3D, DetectedObject3DArray
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
        'internal': 255,
    }
    assert isinstance(result.objects, DetectedObject3DArray)
    assert feedback.STAGE_WAITING_FOR_RGBD == 0
    assert feedback.STAGE_TRANSFORMING == 4


def test_plan_grasp_contract_constants_and_payloads() -> None:
    request = PlanGrasp.Request()
    response = PlanGrasp.Response()

    assert request.ARM_AUTO == 0
    assert request.ARM_LEFT == 1
    assert request.ARM_RIGHT == 2
    assert isinstance(request.object, DetectedObject3D)
    assert {
        'none': response.ERROR_NONE,
        'invalid_request': response.ERROR_INVALID_REQUEST,
        'invalid_arm_override': response.ERROR_INVALID_ARM_OVERRIDE,
        'unreachable': response.ERROR_UNREACHABLE,
        'ik_failed': response.ERROR_IK_FAILED,
        'fk_validation_failed': response.ERROR_FK_VALIDATION_FAILED,
        'internal': response.ERROR_INTERNAL,
    } == {
        'none': 0,
        'invalid_request': 1,
        'invalid_arm_override': 2,
        'unreachable': 3,
        'ik_failed': 4,
        'fk_validation_failed': 5,
        'internal': 255,
    }
    assert response.grasp_point.header.frame_id == ''
    assert response.joint_target.name == []
