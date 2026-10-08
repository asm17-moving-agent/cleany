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


def test_unknown_capture_time_cannot_silently_use_latest_tf():
    request = PlanGrasp.Request()
    request.snapshot_id = 'snapshot'
    request.object_id = request.target_object.object_id = 7
    request.target_object.obb_size.x = request.target_object.obb_size.y = .1
    request.target_object.obb_size.z = .2
    request.target_object.obb_pose.orientation.w = 1.
    request.target_cloud.header.frame_id = request.context_cloud.header.frame_id = 'camera_frame'
    with pytest.raises(ValueError, match='capture timestamp must be nonzero'):
        GraspNode._validate(request)
    request.target_cloud.header.stamp.sec = request.context_cloud.header.stamp.sec = 10
    GraspNode._validate(request)


def packed_cloud():
    import struct
    from sensor_msgs.msg import PointCloud2, PointField
    message = PointCloud2(height=2, width=1, point_step=16, row_step=20)
    message.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
    message.data = struct.pack('<fffI', 1., 2., 3., 0xff0000) + b'pad!' + struct.pack('<fffI', 4., 5., 6., 0x0000ff) + b'pad!'
    return message


def test_organized_cloud_padding_and_packed_float_rgb_are_preserved():
    from cleany_grasping.grasp_node import point_cloud_from_message
    decoded = point_cloud_from_message(packed_cloud())
    np.testing.assert_allclose(decoded.points, [[1., 2., 3.], [4., 5., 6.]])
    np.testing.assert_allclose(decoded.colors, [[1., 0., 0.], [0., 0., 1.]])


@pytest.mark.parametrize('malformed', ['datatype', 'cross_point', 'short_row', 'short_data'])
def test_malformed_cloud_layout_cannot_be_reinterpreted_as_a_valid_grasp(malformed):
    from sensor_msgs.msg import PointField
    from cleany_grasping.grasp_node import point_cloud_from_message
    message = packed_cloud()
    if malformed == 'datatype':
        message.fields[0].datatype = PointField.INT32
    elif malformed == 'cross_point':
        message.fields[2].offset = 16
    elif malformed == 'short_row':
        message.row_step = 12
    else:
        message.data = bytes(message.data[:-1])
    with pytest.raises(ValueError, match='PointCloud2'):
        point_cloud_from_message(message)
