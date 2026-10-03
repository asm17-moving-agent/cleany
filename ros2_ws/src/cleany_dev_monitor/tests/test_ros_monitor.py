"""Opt-in ROS graph/QoS/layer checks in an isolated DDS domain."""

import os
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("CLEANY_RUN_ROS_TESTS") != "1", reason="ROS opt-in"
)


def test_qos_discovery_missing_transform_and_read_only_topics(tmp_path, monkeypatch):
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda hub, config, stop: None)
    rclpy.init()
    monitor = MonitorNode(
        parameter_overrides=[Parameter("state_directory", value=str(tmp_path))]
    )
    source = rclpy.create_node("monitor_test_source")
    pub = source.create_publisher(
        String,
        "/monitor_test",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    monitor.hub.select("test", ["/monitor_test"])
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            pub.publish(String(data="hello"))
            executor.spin_once(timeout_sec=0.05)
            samples = [
                r
                for r in monitor.hub.snapshot()["rows"]
                if r["key"] == "/monitor_test" and r["kind"] == "sample"
            ]
            if samples:
                break
        assert samples[0]["data"]["data"] == "hello"
        graph = next(
            r["data"] for r in monitor.hub.snapshot()["rows"] if r["kind"] == "graph"
        )
        info = next(t for t in graph["topics"] if t["name"] == "/monitor_test")
        assert info["publishers"][0]["qos"]["reliability"] == "BEST_EFFORT"
        source.destroy_publisher(pub)
        monitor.discover()
        assert not source.get_publishers_info_by_topic("/monitor_test")
        monitor.hub.release("test")
        monitor.discover()
        assert "/monitor_test" not in monitor.subs
        assert not any(
            r["key"] == "/monitor_test" for r in monitor.hub.snapshot()["rows"]
        )
        grid = OccupancyGrid()
        grid.header.frame_id = "missing_frame"
        grid.info.width = grid.info.height = 1
        grid.info.resolution = 0.05
        grid.info.origin.orientation.w = 1.0
        grid.data = [100]
        monitor.layer("/map", grid, 0.0)
        layer = next(
            r["data"] for r in monitor.hub.snapshot()["rows"] if r["kind"] == "layer"
        )
        assert not layer["valid"] and "cells" not in layer
        published = monitor.get_publisher_names_and_types_by_node(
            monitor.get_name(), monitor.get_namespace()
        )
        assert all(name in ("/rosout", "/parameter_events") for name, _ in published)
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_camera_best_effort_subscription_and_expiry(tmp_path, monkeypatch):
    import rclpy
    from cleany_dev_monitor.node import MonitorNode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda hub, config, stop: None)
    rclpy.init()
    monitor = MonitorNode()
    source = rclpy.create_node("camera_test_source")
    pub = source.create_publisher(Image, "/test_camera/image", qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    message = Image(height=2, width=2, encoding="rgb8", step=6, data=[255, 0, 0] * 4)
    try:
        deadline = time.monotonic() + 6
        frame = None
        while time.monotonic() < deadline:
            monitor.hub.cameras.request("/test_camera/image")
            pub.publish(message)
            executor.spin_once(timeout_sec=0.05)
            frame = monitor.hub.cameras.request("/test_camera/image")[0]
            if frame:
                break
        assert frame and frame.width == 2 and frame.jpeg.startswith(b"\xff\xd8")
        # Expire observation lease without any robot motion or control calls.
        monitor.hub.cameras.clock = lambda: time.monotonic() + 5
        monitor.discover()
        assert "/test_camera/image" not in monitor.subs
    finally:
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
