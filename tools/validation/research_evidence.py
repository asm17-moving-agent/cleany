#!/usr/bin/env python3
"""Record timestamped controller samples and redacted frames during field attempts."""

import json
import math
import os
import shutil
import time
from collections import deque
from pathlib import Path
from uuid import UUID

import cv2
import rclpy
from cleany_interfaces.msg import ManipulationExecutionRecord
from control_msgs.msg import DynamicJointState, JointTrajectoryControllerState
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Evidence(Node):
    """Passive, bounded sampler. All capture times use the Jetson wall clock."""

    def __init__(self):
        super().__init__("research_evidence")
        self.root = Path(os.environ["RESEARCH_EVIDENCE"])
        self.root.mkdir(parents=True, exist_ok=True)
        self.quota = int(os.environ.get("RESEARCH_QUOTA_MIB", "512")) * 1024**2
        self.reserve = int(os.environ.get("RESEARCH_RESERVE_MIB", "2048")) * 1024**2
        self.last_sample = {}
        self.pending = deque(maxlen=48)
        self.active = {}
        self.bridge = CvBridge()
        self.used = sum(p.stat().st_size for p in self.root.glob("*/*") if p.is_file())
        self.available = shutil.disk_usage(self.root).free
        self.create_subscription(
            ManipulationExecutionRecord,
            "/manipulation/execution_events",
            self.event,
            100,
        )
        for arm in ("left", "right"):
            self.create_subscription(
                JointTrajectoryControllerState,
                f"/{arm}_arm_controller/controller_state",
                lambda msg, side=arm: self.controller(side, msg),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            DynamicJointState,
            "/joint_state_broadcaster/dynamic_joint_states",
            self.motor,
            qos_profile_sensor_data,
        )
        self.image_subscription = None
        self.create_timer(5, self.maintain)

    def maintain(self):
        self.available = shutil.disk_usage(self.root).free
        for key, seen in list(self.active.items()):
            if time.monotonic() - seen > 120:
                del self.active[key]

        self.sync_image_subscription()

    def sync_image_subscription(self):
        if self.active and self.image_subscription is None:
            self.image_subscription = self.create_subscription(
                Image, "/camera/head/color/image_redacted", self.image, qos_profile_sensor_data)
        elif not self.active and self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None

    def event(self, msg):
        try:
            key = str(UUID(msg.execution_id))
        except ValueError:
            return
        if msg.record_state != "ACTIVE":
            self.active.pop(key, None)
            self.sync_image_subscription()
            return
        if key not in self.active:
            if self.used >= self.quota or self.available <= self.reserve:
                return
            (self.root / key).mkdir(exist_ok=True)
            meta = self.root / key / "capture.json"
            if not meta.exists():
                release = Path("/opt/cleany/current").resolve().name
                meta.write_text(
                    json.dumps(
                        {
                            "accepted_at_ns": msg.accepted_at_ns,
                            "release": release,
                            "time_axis": "received_wall_ns",
                            "controller_hz_max": 10,
                            "redacted_frame_hz_max": 2,
                        }
                    )
                )
            for sample, frame in self.pending:
                if sample["at_ns"] >= msg.accepted_at_ns:
                    self.save(key, sample, frame)
        self.active[key] = time.monotonic()
        self.sync_image_subscription()

    def sample(self, stream, hz):
        now = time.monotonic()
        if now - self.last_sample.get(stream, 0) < 1 / hz:
            return False
        self.last_sample[stream] = now
        return True

    def controller(self, side, msg):
        if not self.sample(side, 10):
            return
        desired = list(msg.reference.positions or msg.desired.positions)
        actual = list(msg.feedback.positions or msg.actual.positions)
        if not all(math.isfinite(x) for x in desired + actual):
            return
        self.record(
            dict(
                kind="controller",
                side=side,
                names=list(msg.joint_names),
                desired=desired,
                actual=actual,
            ),
            msg,
        )

    def motor(self, msg):
        """Record the hardware driver's quantized sent position and measured position."""
        if not self.sample("motor", 10):
            return
        sample = {
            "kind": "motor",
            "names": [],
            "desired": [],
            "actual": [],
            "feedback_stamp_ns": [],
        }
        for name, fields in zip(msg.joint_names, msg.interface_values):
            values = dict(zip(fields.interface_names, fields.values))
            sent, measured = values.get("sent_position"), values.get("position")
            stamp = values.get("feedback_stamp_ns")
            if (
                sent is None
                or measured is None
                or not all(map(math.isfinite, (sent, measured)))
            ):
                continue
            sample["names"].append(name)
            sample["desired"].append(sent)
            sample["actual"].append(measured)
            sample["feedback_stamp_ns"].append(
                int(stamp) if stamp and math.isfinite(stamp) else None
            )
        if sample["names"]:
            self.record(sample, msg)

    def image(self, msg):
        if not self.active or not self.sample("image", 2):
            return
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if img.shape[1] > 480:
            img = cv2.resize(
                img, (480, max(1, round(img.shape[0] * 480 / img.shape[1])))
            )
        ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            self.record(dict(kind="frame"), msg, jpeg.tobytes())

    def record(self, sample, msg, frame=None):
        sample.update(
            at_ns=time.time_ns(),
            stamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
        )
        self.pending.append((sample, frame))
        for key in self.active:
            self.save(key, sample, frame)

    def save(self, key, sample, frame):
        sample = dict(sample)
        directory = self.root / key
        if frame:
            sample["file"] = f"{sample['at_ns']}.jpg"
        line = (
            json.dumps(sample, separators=(",", ":"), allow_nan=False) + "\n"
        ).encode()
        size = len(line) + (len(frame) if frame else 0)
        if self.used + size > self.quota or self.available - size < self.reserve:
            return
        if frame:
            (directory / sample["file"]).write_bytes(frame)
        with (directory / "samples.jsonl").open("ab") as f:
            f.write(line)
        self.used += size
        self.available -= size


def main():
    rclpy.init()
    node = Evidence()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
