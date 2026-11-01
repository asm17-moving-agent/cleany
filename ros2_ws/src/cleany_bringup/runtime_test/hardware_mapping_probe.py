#!/usr/bin/env python3
"""Device-free installed-launch proof: guarded drive, mapping reset, save and bags."""

from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=8876)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "173":
        raise SystemExit(
            "This mock probe requires ROS_DOMAIN_ID=173, isolated from hardware"
        )
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    state = output / "runtime"
    log = (output / "runtime.log").open("w")
    profile = (
        Path(get_package_share_directory("cleany_bringup"))
        / "config/mapping_legacy.yaml"
    )
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "cleany_bringup",
            "hardware_mapping.launch.py",
            "mock:=true",
            "profile:=" + str(profile),
            "state_root:=" + str(state),
            "host:=127.0.0.1",
            "phone_port:=" + str(args.port),
        ],
        cwd="/tmp",
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node("hardware_mapping_probe")
    publisher = node.create_publisher(LaserScan, "scan_raw", qos_profile_sensor_data)

    def scan():
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "lidar_link"
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / 720
        msg.angle_max = msg.angle_min + 719 * msg.angle_increment
        msg.range_min = 0.05
        msg.range_max = 12.0
        msg.scan_time = 0.1
        msg.ranges = [
            min(
                4 / max(abs(math.cos(msg.angle_min + i * msg.angle_increment)), 1e-6),
                3 / max(abs(math.sin(msg.angle_min + i * msg.angle_increment)), 1e-6),
            )
            for i in range(720)
        ]
        publisher.publish(msg)

    node.create_timer(0.1, scan)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{args.port}"
    token = None
    results = {}

    def get(path):
        return json.load(
            urlopen(
                Request(base + path, headers={"X-Control-Token": token or ""}),
                timeout=2,
            )
        )

    def post(path, data):
        return json.load(
            urlopen(
                Request(
                    base + path,
                    data=json.dumps(data).encode(),
                    headers={
                        "Content-Type": "application/json",
                        "Origin": base,
                        "X-Control-Token": token,
                    },
                ),
                timeout=3,
            )
        )

    def wait(predicate, timeout=45):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                value = predicate()
                if value:
                    return value
            except (OSError, ValueError, KeyError, URLError):
                pass
            time.sleep(0.05)
        raise RuntimeError("Condition timed out; inspect runtime/device logs")

    def record_command_sample(client):
        # Publish an explicit zero intent while recording; no mock displacement is needed.
        seq = 1
        post(
            "/api/control",
            dict(
                ui_version=2,
                client=client,
                sequence=seq,
                action="press",
                vector=[0, 0, 0],
                speed=0.5,
            ),
        )
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            seq += 1
            post(
                "/api/control",
                dict(
                    ui_version=2,
                    client=client,
                    sequence=seq,
                    action="update",
                    vector=[0, 0, 0],
                    speed=0.5,
                ),
            )
            time.sleep(0.065)
        post(
            "/api/control",
            dict(ui_version=2, client=client, sequence=seq + 1, action="stop"),
        )
        wait(lambda: get("/api/status")["stop_confirmed"])

    try:
        wait(lambda: (state / "web-token").is_file())
        token = (state / "web-token").read_text().strip()
        wait(lambda: get("/api/status")["ready"])
        with urlopen(base + "/", timeout=2) as response:
            html = response.read().decode()
        import re

        assets = re.findall(r'(?:src|href)="(/assets/[^" ]+)"', html)
        assert len(assets) >= 2
        for asset in assets:
            with urlopen(base + asset, timeout=2) as response:
                assert response.status == 200 and response.read()
        try:
            urlopen(base + "/api/status", timeout=2)
        except HTTPError as error:
            assert error.code == 403
        else:
            raise AssertionError("Missing authentication was accepted")
        results["served_ui_assets"] = assets
        results["initial"] = get("/api/status")
        time.sleep(0.5)
        infos = node.get_publishers_info_by_topic("/base/wheel_command")
        assert len(infos) == 1 and infos[0].node_name == "cleany_base_driver"
        results["sole_wheel_publisher"] = infos[0].node_name
        sequence = 1
        post(
            "/api/control",
            dict(
                ui_version=2,
                client="probe",
                sequence=sequence,
                action="press",
                vector=[1, 0, 0],
                speed=0.5,
            ),
        )
        moving = []
        end = time.monotonic() + 1.2
        while time.monotonic() < end:
            sequence += 1
            post(
                "/api/control",
                dict(
                    ui_version=2,
                    client="probe",
                    sequence=sequence,
                    action="update",
                    vector=[1, 0, 0],
                    speed=0.5,
                ),
            )
            moving.append(get("/api/status"))
            time.sleep(0.065)
        assert any(
            row["phase"] == "moving" and row["target"][0] > 0.4 for row in moving
        )
        sequence += 1
        post(
            "/api/control",
            dict(ui_version=2, client="probe", sequence=sequence, action="stop"),
        )
        wait(lambda: get("/api/status")["stop_confirmed"])
        results["released"] = get("/api/status")
        first = post("/api/reset", {"request_id": "first"})
        assert (
            post("/api/reset", {"request_id": "first"})["operation_id"]
            == first["operation_id"]
        )
        wait(lambda: get("/api/mapping/status")["phase"] in ("running", "failed"))
        first = get("/api/mapping/status")
        assert first["phase"] == "running", first
        results["first"] = first
        record_command_sample("record-first")
        time.sleep(5)
        post("/api/reset", {"request_id": "second"})
        wait(
            lambda: (
                get("/api/mapping/status").get("generation", 0) >= 2
                or get("/api/mapping/status")["phase"] == "failed"
            )
        )
        second = get("/api/mapping/status")
        assert second["phase"] == "running", second
        results["second"] = second
        record_command_sample("record-second")
        assert first["output"] != second["output"]
        old = Path(first["output"])
        for name in (
            "map.yaml",
            "map.pgm",
            "latest.pbstream",
            "recording.json",
            "shutdown.json",
        ):
            assert (old / name).is_file(), name
        time.sleep(5)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=50)
        assert process.returncode == 0, process.returncode
        results["shutdown"] = json.loads((state / "shutdown.json").read_text())
        assert (
            results["shutdown"]["mapping_saved"]
            and results["shutdown"]["stop_confirmed"]
        ), results["shutdown"]
        import rosbag2_py

        for session in (first, second):
            recording = json.loads(
                (Path(session["output"]) / "recording.json").read_text()
            )
            bag = Path(recording["state_directory"]) / "bags" / recording["session_id"]
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""),
            )
            topics = {row.name for row in reader.get_all_topics_and_types()}
            required = {
                "/scan_raw",
                "/scan",
                "/base/wheel_command",
                "/base/wheel_state",
                "/joint_states",
                "/odom",
                "/wheel/odom",
                "/tf",
                "/tf_static",
                "/map",
            }
            assert required <= topics, required - topics
            count = 0
            while reader.has_next():
                reader.read_next()
                count += 1
            results.setdefault("bags", []).append(
                {"uri": str(bag), "topics": sorted(topics), "messages": count}
            )
        results["success"] = True
        print(json.dumps(results, indent=2))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=50)
            except subprocess.TimeoutExpired:
                results["cleanup_error"] = (
                    "Runtime did not exit; ownership records retained"
                )
        (output / "verification.json").write_text(json.dumps(results, indent=2))
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()


if __name__ == "__main__":
    main()
