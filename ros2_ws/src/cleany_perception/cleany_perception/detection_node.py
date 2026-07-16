"""ROS 2 node wrapper for Cleany perception.

Thin ROS layer over the pure logic in detector.py / imaging.py /
detection_to_msg.py. It only wires topics to those modules; all detection
logic lives in the core modules so it stays unit-testable without ROS
(AGENTS.md section 4).

Data flow:
- subscribe to an Image topic (e.g. /image_raw from cleany_mujoco_sim),
- buffer the latest frame, and on a timer convert it with
  imaging.image_to_ndarray() and run detector.detect(),
- publish the result as vision_msgs/Detection2DArray.

Everything environment-specific (topics, weights, conf, class filter, device,
detection rate) is a ROS parameter, never hardcoded (AGENTS.md section 4). The
subscription uses sensor-data QoS so it receives from both the sim (reliable)
and a real camera driver (best-effort).
"""

from __future__ import annotations

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from cleany_perception.detection_to_msg import detections_to_msg
from cleany_perception.detector import YoloDetector
from cleany_perception.imaging import image_to_ndarray, ndarray_to_image_msg


class DetectionNode(Node):
    """Subscribes to images, runs a detector on a timer, publishes detections.

    `detector` is injectable so tests can wire the node with a fake detector
    (no ultralytics, no model download). When omitted, a `YoloDetector` is
    built from the node's parameters.
    """

    def __init__(self, detector=None) -> None:
        super().__init__('detection_node')

        image_topic = self.declare_parameter('image_topic', '/image_raw').value
        detections_topic = self.declare_parameter('detections_topic', '/detections').value
        weights = self.declare_parameter('weights', 'yolo11n-seg.pt').value
        conf = self.declare_parameter('conf', 0.25).value
        device = self.declare_parameter('device', '').value
        detection_rate_hz = self.declare_parameter('detection_rate_hz', 5.0).value
        publish_annotated = self.declare_parameter('publish_annotated', False).value
        annotated_topic = self.declare_parameter('annotated_topic', '/detections_image').value
        classes = self.declare_parameter(
            'classes', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY),
        ).value

        if detection_rate_hz <= 0.0:
            raise ValueError('detection_rate_hz must be positive')

        # rclpy returns None (not []) for an empty array parameter, so guard it.
        class_filter = [int(c) for c in (classes or [])] or None
        self._detector = detector or YoloDetector(
            weights=weights,
            conf=conf,
            classes=class_filter,
            device=device,
        )
        # Warm-up: load the model now so the first frame isn't stalled and a
        # bad weights path fails fast at startup.
        self._detector.load()

        self._latest: Image | None = None
        self._pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self._annotated_pub = (
            self.create_publisher(Image, annotated_topic, 10) if publish_annotated else None
        )
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_timer(1.0 / detection_rate_hz, self._on_timer)

        annotated_note = f", annotated -> '{annotated_topic}'" if publish_annotated else ''
        self.get_logger().info(
            f"detection_node: sub '{image_topic}' -> pub '{detections_topic}' "
            f"@ {detection_rate_hz} Hz (weights={weights}, device={device or 'auto'})"
            f"{annotated_note}"
        )

    def _on_image(self, msg: Image) -> None:
        # Keep only the newest frame; the timer drives inference so a slow
        # detector drops stale frames instead of building a backlog.
        self._latest = msg

    def _run(self, msg: Image):
        """Decode one Image and run the detector once. Returns (ndarray, dets)."""
        image = image_to_ndarray(msg)
        detections = self._detector.detect(image)
        return image, detections

    def detect_on(self, msg: Image) -> Detection2DArray:
        """Run the full convert -> detect -> message pipeline on one Image.

        Stamp and frame_id are taken from the source image so detections stay
        aligned with the frame they came from.
        """
        _, detections = self._run(msg)
        stamp = Time.from_msg(msg.header.stamp)
        return detections_to_msg(detections, stamp, msg.header.frame_id)

    def _on_timer(self) -> None:
        msg = self._latest
        if msg is None:
            return  # no image yet -> publish nothing (distinct from "saw nothing")
        try:
            image, detections = self._run(msg)
        except Exception as exc:  # noqa: BLE001 - never let one bad frame kill the node
            self.get_logger().warn(f'detection failed: {exc}')
            return
        # Always publish, even with zero detections, so the Planner can tell
        # "camera up, saw nothing" from "no data" (AGENTS.md section 3).
        stamp = Time.from_msg(msg.header.stamp)
        self._pub.publish(detections_to_msg(detections, stamp, msg.header.frame_id))

        if self._annotated_pub is not None:
            try:
                from cleany_perception.annotate import draw_detections

                annotated = draw_detections(image, detections)
                self._annotated_pub.publish(ndarray_to_image_msg(annotated, msg.header))
            except Exception as exc:  # noqa: BLE001 - viz must never kill the node
                self.get_logger().warn(f'annotation failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
