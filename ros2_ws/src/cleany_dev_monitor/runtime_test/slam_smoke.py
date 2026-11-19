"""Synthetic room scans -> actual Cartographer -> logger observations -> durable replay.

Run only in an isolated ROS_DOMAIN_ID. Publishes sensor/odometry/TF fixtures;
never publishes commands, goals, or actuator messages. No physical validation.
"""

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import rclpy
from cleany_logger import Recorder
from cleany_logger.observer import ObservationNode
from cleany_logger.slam_reader import SlamReader
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=35.0)
    args = parser.parse_args()
    if (
        os.environ.get("ROS_LOCALHOST_ONLY") != "1"
        or int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0
    ):
        raise SystemExit("Use an isolated nonzero domain with ROS_LOCALHOST_ONLY=1")
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[4]
    config_dir = root / "ros2_ws/src/cleany_navigation/config/slam"
    log = (output / "cartographer.log").open("w")
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "cartographer_ros",
            "cartographer_node",
            "-configuration_directory",
            str(config_dir),
            "-configuration_basename",
            "cartographer_2d.lua",
            "--ros-args",
            "-r",
            "odom:=/wheel/odom",
        ],
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    rclpy.init()
    monitor = ObservationNode(slam_factory=SlamReader,
        parameter_overrides=[
            Parameter("state_directory", value=str(output / "monitor")),
            Parameter("port", value=8876),
            Parameter("source_revision", value="synthetic-room-cartographer-smoke"),
        ]
    )
    source = rclpy.create_node("synthetic_room_sensor")
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    scans = source.create_publisher(LaserScan, "/scan", 1)
    odometry = source.create_publisher(Odometry, "/wheel/odom", 1)
    static = StaticTransformBroadcaster(source)
    dynamic = TransformBroadcaster(source)
    sensor = TransformStamped()
    sensor.header.frame_id = "base_link"
    sensor.child_frame_id = "laser"
    sensor.transform.rotation.w = 1.0
    static.sendTransform(sensor)
    recorder = Recorder(output / "recordings", controls=monitor.hub.recording)
    started = time.monotonic()
    next_sample = started
    observations = 0
    try:
        while time.monotonic() - started < args.duration:
            if process.poll() is not None:
                raise RuntimeError("Cartographer exited; inspect cartographer.log")
            now = time.monotonic()
            if now >= next_sample:
                next_sample = now + 0.08
                phase = (now - started) * 0.18
                x, y = 1.5 * math.cos(phase), 1.5 * math.sin(phase)
                heading = phase + math.pi / 2
                stamp = source.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.frame_id = "odom"
                odom.header.stamp = stamp
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.z = math.sin(heading / 2)
                odom.pose.pose.orientation.w = math.cos(heading / 2)
                odom.twist.twist.linear.x = 0.27
                odom.twist.twist.angular.z = 0.18
                odometry.publish(odom)
                tf = TransformStamped()
                tf.header = odom.header
                tf.child_frame_id = "base_link"
                tf.transform.translation.x = x
                tf.transform.translation.y = y
                tf.transform.rotation = odom.pose.pose.orientation
                dynamic.sendTransform(tf)
                scan = LaserScan()
                scan.header.frame_id = "laser"
                scan.header.stamp = stamp
                scan.angle_min = -math.pi
                scan.angle_increment = 2 * math.pi / 360
                scan.angle_max = math.pi - scan.angle_increment
                scan.range_min = 0.15
                scan.range_max = 12.0
                scan.scan_time = 0.08
                ranges = []
                for i in range(360):
                    angle = heading + scan.angle_min + i * scan.angle_increment
                    dx, dy = math.cos(angle), math.sin(angle)
                    hits = []
                    # Asymmetric bounded room, with a niche along one wall.
                    for wall in (-4.0, 5.0):
                        if abs(dx) > 1e-9:
                            d = (wall - x) / dx
                            if d > 0 and -3.0 <= y + d * dy <= 4.0:
                                hits.append(d)
                    for wall in (-3.0, 4.0):
                        if abs(dy) > 1e-9:
                            d = (wall - y) / dy
                            if d > 0 and -4.0 <= x + d * dx <= 5.0:
                                hits.append(d)
                    if abs(dx) > 1e-9:
                        d = (3.0 - x) / dx
                        if d > 0 and -0.5 <= y + d * dy <= 1.0:
                            hits.append(d)
                    ranges.append(float(min(hits)) if hits else float("inf"))
                scan.ranges = ranges
                scans.publish(scan)
            executor.spin_once(timeout_sec=0.01)
            if now >= next_sample - 0.01:
                recorder.append(monitor.hub.snapshot(recorder.last))
                snapshot = monitor.hub.snapshot()
                observations = max(
                    observations,
                    sum(r["kind"] == "slam_texture" for r in snapshot["rows"]),
                )
                (output / "latest.json").write_text(
                    json.dumps(snapshot, ensure_ascii=False)
                )
        snapshot = monitor.hub.snapshot()
        recorder.append(snapshot)
        textures = [r for r in snapshot["rows"] if r["kind"] == "slam_texture"]
        assert textures, "No real Cartographer texture received"
        assert monitor.hub.latest_data("slam", "trajectory").get("valid")
        replay = recorder.replay(recorder.session_id, time.time())
        assert any(r["kind"] == "slam_texture" for r in replay["rows"])
        report = {
            "evidence": "actual Cartographer with synthetic LaserScan/odometry/TF",
            "physical_validation": False,
            "domain": os.environ["ROS_DOMAIN_ID"],
            "duration_s": args.duration,
            "max_observed_textures": observations,
            "textures": [
                {
                    "id": r["key"],
                    "version": r["data"]["version"],
                    "width": r["data"]["width"],
                    "height": r["data"]["height"],
                }
                for r in textures
            ],
            "session_id": recorder.session_id,
            "recording_reason": recorder.reason,
            "trajectory_markers": len(
                monitor.hub.latest_data("slam", "trajectory").get("markers", [])
            ),
            "constraint_markers": len(
                monitor.hub.latest_data("slam", "constraints").get("markers", [])
            ),
        }
        (output / "verification.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        recorder.close()
        monitor.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    main()
