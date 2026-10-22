import math

import numpy as np
import pytest
from cleany_interfaces.msg import DetectedObject3D
from cleany_interfaces.srv import PlanGrasp

from cleany_grasping.grasp_node import (
    GraspNode,
    _predict_and_rank,
    _rotation_from_quaternion,
    _transform_target_object,
    observed_geometry_message,
)
from cleany_grasping.core.models import PointCloud, RawGrasp
from cleany_grasping.core.selector import GraspConfig


def test_repeated_terminal_task_status_does_not_keep_worker_alive():
    from types import SimpleNamespace
    calls=[]
    node=SimpleNamespace(_predictor=SimpleNamespace(hold_for=lambda *a:calls.append(a)),
        _task_hold_active=False,get_parameter=lambda _:SimpleNamespace(value=600.))
    status=lambda value:SimpleNamespace(status_list=[SimpleNamespace(status=value)])
    GraspNode._task_status(node,status(4))
    assert calls==[]
    GraspNode._task_status(node,status(2))
    GraspNode._task_status(node,status(4))
    GraspNode._task_status(node,status(4))
    assert calls==[('task',600.),('task',0.)]


def test_deferred_prepare_releases_only_the_vlm_hold():
    from types import SimpleNamespace
    calls=[]
    node=SimpleNamespace(_predictor=SimpleNamespace(hold_for=lambda *a:calls.append(a),prepare=lambda:False),
        get_parameter=lambda _:SimpleNamespace(value=180.))
    response=SimpleNamespace()
    GraspNode._prepare_window(node,SimpleNamespace(data=True),response)
    assert not response.success and calls==[('vlm',180.),('vlm',0.)]


def test_observed_mesh_message_roundtrips_with_frame_pose_and_capture_identity():
    from cleany_interfaces.msg import ObservedObjectGeometry
    from rclpy.serialization import serialize_message, deserialize_message
    from std_msgs.msg import Header
    target = DetectedObject3D(object_id=7)
    target.obb_size.z = .04
    target.obb_pose.position.x = .5
    target.obb_pose.position.z = .36
    header = Header(frame_id='base_link')
    header.stamp.sec = 3
    points = np.array([(x,y,.38) for x in (.48,.52) for y in (-.02,.02)])
    message = observed_geometry_message(points, target, header, 'snapshot', 7)
    decoded = deserialize_message(serialize_message(message), ObservedObjectGeometry)
    assert decoded == message
    assert decoded.header == header and decoded.mesh_pose == target.obb_pose
    assert decoded.snapshot_id == 'snapshot' and decoded.object_id == 7
    assert min(p.z for p in decoded.mesh.vertices) == pytest.approx(-.02)
    assert max(p.x for p in decoded.mesh.vertices) == pytest.approx(.02)


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
    request.target_cloud.header.stamp.sec = request.context_cloud.header.stamp.sec = 1
    request.target_object_header = request.context_cloud.header

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
    with pytest.raises(ValueError, match='positive capture timestamp'):
        GraspNode._validate(request)
    request.target_cloud.header.stamp.sec = request.context_cloud.header.stamp.sec = 10
    request.target_object_header = request.target_cloud.header
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
def test_rank_retries_full_cloud_when_fast_candidates_are_invalid():
    invalid = RawGrasp(
        np.eye(3),
        np.array((1.0, 1.0, 1.0)),
        0.04,
        0.02,
        0.8,
    )
    valid = RawGrasp(
        np.eye(3),
        np.array((0.0, 0.0, 0.5)),
        0.04,
        0.02,
        0.9,
    )

    class Predictor:
        def __init__(self):
            self.full_calls = 0

        def predict(self, _target, _context, _workspace):
            return (invalid,)

        def should_retry_full_resolution(self, _context):
            return True

        def predict_full_resolution(self, _target, _context, _workspace):
            self.full_calls += 1
            return (valid,)

    predictor = Predictor()
    target = PointCloud(
        np.array(((-0.05, -0.05, 0.45), (0.05, 0.05, 0.55))),
        np.zeros((2, 3)),
    )

    candidates, ranked, retried = _predict_and_rank(
        predictor,
        target,
        target,
        np.asarray((-0.1, 0.1, -0.1, 0.1, 0.4, 0.6)),
        GraspConfig(),
    )

    assert len(candidates) == 1
    assert candidates[0] is valid
    assert len(ranked) == 1
    assert retried
    assert predictor.full_calls == 1


@pytest.mark.parametrize("object_frame", ["camera_frame", "base_link", "object_camera"])
def test_capture_transform_survives_slow_inference_and_older_snapshot(monkeypatch, object_frame):
    """A >10 s old capture must not become a latest-pose grasp after inference."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from cleany_grasping.core.models import GraspPose
    import cleany_grasping.grasp_node as module

    rclpy.init(args=[])
    node = GraspNode(predictor=object())
    try:
        from types import SimpleNamespace
        published = []
        node._geometry_publisher = SimpleNamespace(publish=published.append)
        now = node.get_clock().now().nanoseconds
        for offset, x in ((20, 1.0), (0, 3.0)):
            transform = TransformStamped()
            transform.header.stamp = Time(nanoseconds=now-offset*10**9).to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = 'camera_frame'
            transform.transform.translation.x = x
            transform.transform.rotation.w = 1.0
            node._tf_buffer.set_transform(transform, 'test')
        transform.child_frame_id = 'object_camera'
        transform.transform.translation.x = 5.0
        transform.transform.rotation.z = math.sqrt(0.5)
        transform.transform.rotation.w = math.sqrt(0.5)
        node._tf_buffer.set_transform_static(transform, 'test')
        request = PlanGrasp.Request()
        request.snapshot_id = 'old-capture'
        request.object_id = request.target_object.object_id = 1
        request.target_object.obb_pose.orientation.w = 1.0
        request.target_object.obb_size.x = 0.1
        request.target_object.obb_size.y = 0.1
        request.target_object.obb_size.z = 0.1
        cloud = PointCloud2()
        cloud.header.frame_id = 'camera_frame'
        cloud.header.stamp = Time(nanoseconds=now-15*10**9).to_msg()
        request.target_cloud = request.context_cloud = cloud
        from copy import deepcopy
        request.target_object_header = deepcopy(cloud.header)
        request.target_object_header.frame_id = object_frame
        points = PointCloud(np.array([[0., 0., .5], [.01, 0., .51], [0., .01, .5]]), np.zeros((3, 3)))
        monkeypatch.setattr(module, 'point_cloud_from_message', lambda message: points)
        monkeypatch.setattr(node, '_publish_debug', lambda *args: None)
        pose = GraspPose(np.eye(3), np.array([0., 0., .5]), np.array([1., 0., 0.]), .04, .02, .8)

        def expire_during_inference(*args):
            node._tf_buffer.clear()
            return (), (pose,), False

        monkeypatch.setattr(module, '_predict_and_rank', expire_during_inference)
        response = node._plan(request, PlanGrasp.Response())
        assert response.success, response.message
        assert response.candidates[0].tcp_pose.position.x == pytest.approx(1.5)
        assert response.candidates[0].header.stamp == cloud.header.stamp
        candidate = response.candidates[0]
        assert candidate.target_object.obb_pose.position.x == pytest.approx(
            {'camera_frame': 1.5, 'base_link': 0., 'object_camera': 5.}[object_frame])
        assert candidate.target_object.obb_pose.orientation.z == pytest.approx(
            math.sqrt(0.5) if object_frame == 'object_camera' else 0.)
        assert request.target_object.obb_pose.position.x == 0.
        assert len(published) == 1
        assert published[0].header == candidate.header
        assert published[0].mesh_pose == candidate.target_object.obb_pose
        q = published[0].mesh_pose.orientation
        rotation = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        vertices = np.array([(p.x, p.y, p.z) for p in published[0].mesh.vertices])
        center = np.array([published[0].mesh_pose.position.x, 0., 0.])
        world_vertices = vertices @ rotation.T + center
        assert world_vertices[:, 0].min() == pytest.approx(1.5)
        assert world_vertices[:, 0].max() == pytest.approx(1.51)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('fault', ['missing_frame', 'different_stamp', 'missing_stamp'])
def test_obb_frame_and_capture_stamp_are_required(fault):
    request = PlanGrasp.Request()
    request.snapshot_id = 's'
    request.object_id = request.target_object.object_id = 1
    request.target_cloud.header.frame_id = request.context_cloud.header.frame_id = 'camera'
    request.target_cloud.header.stamp.sec = request.context_cloud.header.stamp.sec = 2
    request.target_object_header.stamp.sec = 2
    if fault != 'missing_frame':
        request.target_object_header.frame_id = 'base_link'
    if fault == 'different_stamp':
        request.target_object_header.stamp.sec = 3
    elif fault == 'missing_stamp':
        request.target_object_header.stamp.sec = 0
        request.target_cloud.header.stamp.sec = request.context_cloud.header.stamp.sec = 0
    with pytest.raises(ValueError, match='explicit frame|positive capture timestamp'):
        GraspNode._validate(request)
