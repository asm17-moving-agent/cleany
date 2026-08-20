import math

import numpy as np
import pytest
from cleany_interfaces.msg import DetectedObject3D
from cleany_interfaces.srv import PlanGrasp

from cleany_grasping.grasp_node import (
    GraspNode,
    _rotation_from_quaternion,
    _transform_target_object,
)


def test_target_obb_pose_is_transformed_with_candidate_frame():
    target = DetectedObject3D()
    target.object_id = 7
    target.obb_pose.position.x = 1.0
    half_sqrt = math.sqrt(0.5)
    target.obb_pose.orientation.z = half_sqrt
    target.obb_pose.orientation.w = half_sqrt

    frame_rotation = np.array(
        (
            (0.0, -1.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    transformed = _transform_target_object(
        target,
        frame_rotation,
        np.array((1.0, 2.0, 3.0)),
    )

    assert (
        transformed.obb_pose.position.x,
        transformed.obb_pose.position.y,
        transformed.obb_pose.position.z,
    ) == pytest.approx((1.0, 3.0, 3.0))
    transformed_rotation = _rotation_from_quaternion(
        transformed.obb_pose.orientation.x,
        transformed.obb_pose.orientation.y,
        transformed.obb_pose.orientation.z,
        transformed.obb_pose.orientation.w,
    )
    assert transformed_rotation == pytest.approx(
        np.diag((-1.0, -1.0, 1.0)),
        abs=1e-7,
    )
    assert target.obb_pose.position.x == 1.0
    assert target.obb_pose.position.y == 0.0


def test_plan_request_rejects_invalid_target_obb_before_transform():
    request = PlanGrasp.Request()
    request.snapshot_id = 'snapshot'
    request.object_id = 7
    request.target_object.object_id = 7
    request.target_object.obb_size.x = 0.1
    request.target_object.obb_size.y = 0.1
    request.target_object.obb_size.z = 0.2
    request.target_object.obb_pose.orientation.w = 0.0
    request.target_cloud.header.frame_id = 'camera_frame'
    request.context_cloud.header.frame_id = 'camera_frame'

    with pytest.raises(ValueError, match='quaternion must be normalized'):
        GraspNode._validate(request)
