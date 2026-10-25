"""Actual RGB-D coverage publisher to ToF evidence consumer, using synthetic IO."""
from dataclasses import asdict
import math
import json
from std_msgs.msg import String
import time
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml
from cleany_tof.core import Settings
from cleany_tof.proximity import proof_covers
from cleany_tof.ros_support import DepthProof
from cleany_navigation.body_geometry import BodyBox
import cleany_navigation.depth_coverage_node as module


def test_real_depth_node_revokes_no_return_evidence_on_invalid_pixels(tmp_path, monkeypatch, request):
    s = Settings(calibrated=True, sensor_z=1.8, sensor_pitch=-math.pi/2,
                 half_length=.1, half_width=.1)
    profile = tmp_path/'tof.yaml'
    profile.write_text(yaml.safe_dump({'/**': {'ros__parameters': asdict(s)}}))
    body = BodyBox('synthetic_body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    monkeypatch.setattr(module, 'load_body_boxes', lambda *_a, **_k: [body])
    monkeypatch.setattr(module, 'TransformListener', lambda *_a, **_k: None)
    rclpy.init(domain_id=186)
    request.addfinalizer(lambda: rclpy.shutdown() if rclpy.ok() else None)
    producer = module.DepthCoverageNode(parameter_overrides=[
        Parameter('tof_coverage_enabled', value=True), Parameter('tof_params_file', value=str(profile)),
        Parameter('update_period_s', value=.03), Parameter('cell_size', value=.05)])
    consumer = Node('synthetic_depth_proof_consumer')
    proof = DepthProof(consumer, s)
    reports = []
    consumer.create_subscription(String, 'tof/depth_coverage', lambda msg: reports.append(json.loads(msg.data)), 1)
    executor = SingleThreadedExecutor()
    executor.add_node(producer)
    executor.add_node(consumer)

    class Transforms:
        @staticmethod
        def can_transform(*_args):
            return True

        @staticmethod
        def lookup_transform(*_args):
            tf = TransformStamped()
            tf.header.stamp = producer.odom.header.stamp
            tf.transform.rotation.w = 1.
            return tf

    producer.buffer = Transforms()
    value = [4.]
    camera = CameraInfo(width=100, height=80, k=[50., 0., 50., 0., 50., 40., 0., 0., 1.],
                        p=[50., 0., 50., 0., 0., 50., 40., 0., 0., 0., 1., 0.])
    camera.header.frame_id = 'head_camera_depth_optical_frame'
    producer.on_info(camera)

    def until(predicate):
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            stamp = producer.get_clock().now().to_msg()
            image = Image(width=100, height=80, step=400, encoding='32FC1',
                          data=np.full((80, 100), value[0], dtype='<f4').tobytes())
            image.header.stamp, image.header.frame_id = stamp, camera.header.frame_id
            odom = Odometry(child_frame_id='base_link')
            odom.header.stamp = stamp
            command = Twist()
            command.linear.x = .03
            producer.on_command(command)
            producer.on_odom(odom)
            producer.on_depth(image)
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        assert predicate(), {'latest': reports[-1:] , 'received': proof.proof}

    try:
        until(lambda: proof_covers(proof.snapshot(), (.03, 0., 0.), (0., 0., 0.), s))
        value[0] = 0.
        until(lambda: proof.proof is not None and proof.proof.get('covered') is False)
        assert not proof_covers(proof.snapshot(), (.03, 0., 0.), (0., 0., 0.), s)
    finally:
        executor.shutdown()
        producer.destroy_node()
        consumer.destroy_node()
        rclpy.shutdown()
