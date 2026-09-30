"""Exercise ROS depth messages and optical TF, without a simulator."""
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from cleany_navigation.depth_clearance_node import DepthClearanceNode
from cleany_navigation.point_cloud import cloud_xyz


@pytest.mark.parametrize('encoding,dtype,scale', [('32FC1', '<f4', 1.), ('16UC1', '<u2', 1000.)])
def test_optical_tf_units_stamp_and_missing_calibration_fail_closed(monkeypatch, encoding, dtype, scale):
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    rclpy.init(args=['--ros-args', '-p', 'pixel_stride:=1'])
    node = None
    try:
        node = DepthClearanceNode()
        outputs = []
        node.publisher = SimpleNamespace(publish=outputs.append)
        info = CameraInfo()
        info.header.frame_id = 'head_camera_depth_optical_frame'
        info.width, info.height = 2, 1
        info.k = [9., 0., 0., 0., 9., 0., 0., 0., 1.]
        info.p = [2., 0., 0., 0., 0., 2., 0., 0., 0., 0., 1., 0.]
        node.on_info(info)
        image = Image()
        image.header.frame_id = info.header.frame_id
        image.header.stamp.sec = 3
        image.width, image.height, image.encoding = 2, 1, encoding
        image.step = 2*np.dtype(dtype).itemsize
        image.data = (np.array([2., 2.])*scale).astype(dtype).tobytes()
        node.on_depth(image)
        assert not outputs  # Never guess the optical transform.
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'head_camera_depth_frame', info.header.frame_id
        tf.transform.rotation.x, tf.transform.rotation.y = -.5, .5
        tf.transform.rotation.z, tf.transform.rotation.w = -.5, .5
        tf.transform.translation.x = .1
        node.buffer.set_transform_static(tf, 'test_backend')
        node.on_depth(image)
        assert len(outputs) == 1
        np.testing.assert_allclose(cloud_xyz(outputs[-1]), [[2.1, 0., 0.], [2.1, -1., 0.]])
        assert outputs[-1].header.frame_id == 'head_camera_depth_frame'
        assert outputs[-1].header.stamp == image.header.stamp
        assert image.header.frame_id == info.header.frame_id  # Input message is not mutated.
        info.d = [float('nan')]
        node.on_depth(image)
        assert len(outputs) == 1
        info.d = []
        image.header.frame_id = 'wrong_frame'
        node.on_depth(image)
        assert len(outputs) == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
