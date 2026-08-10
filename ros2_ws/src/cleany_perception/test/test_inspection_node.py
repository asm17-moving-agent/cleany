from __future__ import annotations

import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import InspectScene
from cleany_interfaces.msg import DetectedObject3DArray
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from cleany_perception.core.models import ObjectMask
from cleany_perception.inspection_node import InspectionNode


_LATE_DEBUG_SUBSCRIBER_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _Detector:
    def __init__(self, detection, events=None) -> None:
        self.detection = detection
        self.events = events

    def detect(self, _rgb, _query):
        if self.events is not None:
            self.events.append('detect')
        return (self.detection,)


class _Segmenter:
    def __init__(self, mask) -> None:
        self.mask = mask

    def segment(self, _rgb, detections):
        return tuple(
            ObjectMask(detection=item, mask=self.mask, score=0.99)
            for item in detections
        )


class _Transformer:
    def __init__(self, transform, events=None) -> None:
        self.transform = transform
        self.events = events

    def lookup(self, _target, _source, _stamp_ns):
        if self.events is not None:
            self.events.append('tf')
        return self.transform


def _wait_until(predicate, timeout_seconds=3.0):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def _sensor_messages(scene, stamp_ns):
    snapshot = scene['snapshot']
    color = Image()
    color.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    color.header.frame_id = 'rgb_optical_frame'
    color.width = snapshot.intrinsics.width
    color.height = snapshot.intrinsics.height
    color.encoding = 'rgb8'
    color.step = color.width * 3
    color.data = snapshot.rgb.tobytes()

    depth = Image()
    depth.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    depth.header.frame_id = snapshot.source_frame
    depth.width = snapshot.intrinsics.width
    depth.height = snapshot.intrinsics.height
    depth.encoding = '32FC1'
    depth.step = depth.width * 4
    depth.data = snapshot.depth_m.astype('<f4').tobytes()

    matrix = [
        snapshot.intrinsics.fx,
        0.0,
        snapshot.intrinsics.cx,
        0.0,
        snapshot.intrinsics.fy,
        snapshot.intrinsics.cy,
        0.0,
        0.0,
        1.0,
    ]
    color_info = CameraInfo()
    color_info.header = color.header
    color_info.width = color.width
    color_info.height = color.height
    color_info.k = matrix
    depth_info = CameraInfo()
    depth_info.header = depth.header
    depth_info.width = depth.width
    depth_info.height = depth.height
    depth_info.k = matrix
    return color, color_info, depth, depth_info


def _make_node(scene, events=None):
    overrides = [
        Parameter('snapshot_timeout_seconds', value=1.0),
        Parameter('plane_distance_threshold_m', value=0.001),
        Parameter('plane_minimum_inliers', value=100),
        Parameter('plane_minimum_inlier_ratio', value=0.9),
    ]
    return InspectionNode(
        detector=_Detector(scene['detection'], events),
        segmenter=_Segmenter(scene['mask']),
        transformer=_Transformer(scene['transform'], events),
        parameter_overrides=overrides,
    )


def test_inspection_action_publishes_objects_debug_and_feedback(
    synthetic_scene,
):
    rclpy.init(args=[])
    node = None
    client_node = None
    executor = None
    spin_thread = None
    action_client = None
    events = []
    try:
        node = _make_node(synthetic_scene, events)
        client_node = rclpy.create_node('inspection_test_client')
        action_client = ActionClient(
            client_node,
            InspectScene,
            '/perception/inspect_scene',
        )
        objects = []
        client_node.create_subscription(
            DetectedObject3DArray,
            '/perception/objects',
            objects.append,
            10,
        )
        color_publisher = client_node.create_publisher(
            Image,
            '/camera/color/image_raw',
            qos_profile_sensor_data,
        )
        color_info_publisher = client_node.create_publisher(
            CameraInfo,
            '/camera/color/camera_info',
            qos_profile_sensor_data,
        )
        depth_publisher = client_node.create_publisher(
            Image,
            '/camera/depth/image_raw',
            qos_profile_sensor_data,
        )
        depth_info_publisher = client_node.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info',
            qos_profile_sensor_data,
        )

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.add_node(client_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        assert action_client.wait_for_server(timeout_sec=2.0)
        assert _wait_until(
            lambda: color_publisher.get_subscription_count() == 1
            and depth_publisher.get_subscription_count() == 1
        )

        feedback_stages = []
        goal = InspectScene.Goal()
        goal.query = 'find box'
        send_future = action_client.send_goal_async(
            goal,
            feedback_callback=lambda message: feedback_stages.append(
                message.feedback.stage
            ),
        )
        assert _wait_until(send_future.done)
        goal_handle = send_future.result()
        assert goal_handle.accepted
        assert _wait_until(lambda: bool(feedback_stages))

        sensor_messages = _sensor_messages(synthetic_scene, 2_000_000_000)
        color_publisher.publish(sensor_messages[0])
        color_info_publisher.publish(sensor_messages[1])
        depth_publisher.publish(sensor_messages[2])
        depth_info_publisher.publish(sensor_messages[3])

        result_future = goal_handle.get_result_async()
        assert _wait_until(result_future.done)
        wrapped_result = result_future.result()
        result = wrapped_result.result

        assert wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        assert result.success
        assert result.error_code == InspectScene.Result.ERROR_NONE
        assert result.objects.header.frame_id == 'base_link'
        assert result.objects.snapshot_id.startswith('rgbd-')
        assert [item.label for item in result.objects.objects] == ['box']
        assert _wait_until(lambda: bool(objects))
        assert objects[0].snapshot_id == result.objects.snapshot_id
        debug_images = []
        client_node.create_subscription(
            Image,
            '/perception/debug_image',
            debug_images.append,
            _LATE_DEBUG_SUBSCRIBER_QOS,
        )
        assert _wait_until(lambda: bool(debug_images))
        assert debug_images[0].encoding == 'rgb8'
        assert _wait_until(lambda: len(feedback_stages) == 5)
        assert feedback_stages == [
            InspectScene.Feedback.STAGE_WAITING_FOR_RGBD,
            InspectScene.Feedback.STAGE_DETECTING,
            InspectScene.Feedback.STAGE_SEGMENTING,
            InspectScene.Feedback.STAGE_RECONSTRUCTING,
            InspectScene.Feedback.STAGE_TRANSFORMING,
        ]
        assert events == ['tf', 'detect']
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if action_client is not None:
            action_client.destroy()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_inspection_action_honors_cancel_while_waiting_for_rgbd(
    synthetic_scene,
):
    rclpy.init(args=[])
    node = None
    client_node = None
    executor = None
    spin_thread = None
    action_client = None
    try:
        node = _make_node(synthetic_scene)
        client_node = rclpy.create_node('inspection_cancel_client')
        action_client = ActionClient(
            client_node,
            InspectScene,
            '/perception/inspect_scene',
        )
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.add_node(client_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        assert action_client.wait_for_server(timeout_sec=2.0)

        feedback = []
        send_future = action_client.send_goal_async(
            InspectScene.Goal(),
            feedback_callback=lambda message: feedback.append(
                message.feedback
            ),
        )
        assert _wait_until(send_future.done)
        goal_handle = send_future.result()
        assert _wait_until(lambda: bool(feedback))
        cancel_future = goal_handle.cancel_goal_async()
        assert _wait_until(cancel_future.done)
        result_future = goal_handle.get_result_async()
        assert _wait_until(result_future.done)
        wrapped_result = result_future.result()

        assert wrapped_result.status == GoalStatus.STATUS_CANCELED
        assert not wrapped_result.result.success
        assert (
            wrapped_result.result.error_code
            == InspectScene.Result.ERROR_CANCELLED
        )
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if action_client is not None:
            action_client.destroy()
        if client_node is not None:
            client_node.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
