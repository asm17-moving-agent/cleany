"""Real DDS/service transport with synthetic Cartographer data, no robot motion."""

import gzip
import os
import struct
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("CLEANY_RUN_ROS_TESTS") != "1", reason="ROS opt-in"
)


def test_cartographer_topics_query_versions_timeout_and_replay(tmp_path, monkeypatch):
    import rclpy
    from cartographer_ros_msgs.msg import SubmapEntry, SubmapList, SubmapTexture
    from cartographer_ros_msgs.srv import SubmapQuery
    from cleany_logger import Recorder
    from cleany_logger.observer import ObservationNode
    from cleany_logger.slam_reader import SlamReader
    from geometry_msgs.msg import Point
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import PointCloud2, PointField
    from visualization_msgs.msg import Marker, MarkerArray

    monkeypatch.setattr("cleany_dev_monitor.node.serve", lambda *args: None)
    rclpy.init()
    monitor = ObservationNode(slam_factory=SlamReader,
        parameter_overrides=[
            Parameter("state_directory", value=str(tmp_path)),
            Parameter("slam_query_timeout_s", value=0.5),
        ]
    )
    source = rclpy.create_node("cartographer_fixture")
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    pub = source.create_publisher(SubmapList, "/submap_list", 1)
    trajectories = source.create_publisher(MarkerArray, "/trajectory_node_list", 1)
    constraints = source.create_publisher(MarkerArray, "/constraint_list", 1)
    cloud = source.create_publisher(PointCloud2, "/scan_matched_points2", 1)
    version = [1]
    queried = []

    def query(request, response):
        queried.append((request.trajectory_id, request.submap_index))
        response.status.code = 0
        response.submap_version = version[0]
        texture = SubmapTexture(width=2, height=3, resolution=0.1)
        texture.cells = list(
            gzip.compress(bytes([100, 0, 0, 200, 100, 0, 0, 0, 100, 0, 0, 200]))
        )
        texture.slice_pose.orientation.w = 1.0
        response.textures = [texture]
        return response

    source.create_service(SubmapQuery, "/submap_query", query)
    x = [0.0]

    def publish():
        stamp = source.get_clock().now().to_msg()
        message = SubmapList()
        message.header.frame_id = "map"
        message.header.stamp = stamp
        entry = SubmapEntry(trajectory_id=0, submap_index=0, submap_version=version[0])
        entry.pose.position.x = x[0]
        entry.pose.orientation.w = 1.0
        message.submap = [entry]
        pub.publish(message)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = stamp
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.color.r = marker.color.a = 1.0
        marker.points = [Point(x=0.0, y=0.0), Point(x=1.0, y=2.0)]
        trajectories.publish(MarkerArray(markers=[marker]))
        constraints.publish(MarkerArray(markers=[marker]))
        points = PointCloud2()
        points.header.frame_id = "map"
        points.header.stamp = stamp
        points.height = 1
        points.width = 2
        points.point_step = 12
        points.row_step = 24
        points.fields = [
            PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
        points.data = struct.pack("<ffffff", 0.0, 0.0, 0.0, 1.0, 2.0, 0.0)
        cloud.publish(points)

    def until(predicate, timeout=8):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            publish()
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(monitor.hub.snapshot())

    recorder = Recorder(tmp_path / "records", controls=monitor.hub.recording)
    try:
        until(
            lambda: (
                bool(monitor.hub.latest_data("slam_texture", "0/0"))
                and bool(monitor.hub.latest_data("slam", "constraints"))
            )
        )
        initial = monitor.hub.latest_data("slam_texture", "0/0")
        assert initial["version"] == 1 and queried == [(0, 0)]
        assert initial["width"] == 2 and initial["height"] == 3
        until(lambda: bool(monitor.hub.latest_data("slam", "matched_points")))
        assert monitor.hub.latest_data("slam", "matched_points")["points"] == [
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 0.0],
        ]
        recorder.append(monitor.hub.snapshot())
        first = time.time()
        x[0] = 0.3
        until(
            lambda: (
                monitor.hub.latest_data("slam", "submaps")["submaps"][0]["pose"][0]
                == 0.3
            )
        )
        assert queried == [(0, 0)]  # Optimization reuses unchanged pixels.
        assert any(
            r["data"].get("kind") == "slam_pose_adjustment"
            for r in monitor.hub.snapshot()["rows"]
        )
        recorder.append(monitor.hub.snapshot(recorder.last))
        before = recorder.replay(recorder.session_id, first)["rows"]
        assert (
            next(
                r["data"]
                for r in before
                if r["kind"] == "slam" and r["key"] == "submaps"
            )["submaps"][0]["pose"][0]
            == 0.0
        )
        version[0] = 2
        until(
            lambda: monitor.hub.latest_data("slam_texture", "0/0").get("version") == 2
        )
        assert len(queried) == 2
        version[0] = 0
        until(
            lambda: monitor.hub.latest_data("slam_texture", "0/0").get("version") == 0
        )
        assert (
            monitor.hub.latest_data("slam_texture", "0/0")["epoch"] != initial["epoch"]
        )
        # Retain a discovered service but stop serving it: async requests time out.
        executor.remove_node(source)
        version[0] = 1
        until(
            lambda: (
                monitor.hub.latest_data("slam", "status").get("state")
                == "query_timeout"
            )
        )
        clients = monitor.get_client_names_and_types_by_node(
            monitor.get_name(), monitor.get_namespace()
        )
        assert clients == [("/submap_query", ["cartographer_ros_msgs/srv/SubmapQuery"])]
        published = monitor.get_publisher_names_and_types_by_node(
            monitor.get_name(), monitor.get_namespace()
        )
        assert all(name in ("/rosout", "/parameter_events") for name, _ in published)
    finally:
        recorder.close()
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
