"""Read executor ownership and measured stow positions before base motion."""

import hashlib
import json
import math
import time
from pathlib import Path

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from cleany_mission_manager.core.manipulation import ManipulationPort


class PhysicalArmPort(ManipulationPort):
    def __init__(self, node, transport, journal, calibration, destination, tolerance, max_age):
        config = json.loads(Path(calibration).expanduser().read_text())
        self.digest = hashlib.sha256(
            json.dumps(config, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        self.stow = {}
        joints = ("shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll")
        if (
            config.get("operation_mode") != "full_collection"
            or config.get("commissioning_verified") is not True
            or destination not in config["label_destinations"].values()
            or not 0 < tolerance
            or not 0 < max_age
            or not all(map(math.isfinite, [tolerance, max_age]))
        ):
            raise ValueError("physical desk requires commissioned full_collection settings")
        for side in ("left", "right"):
            arm = config["arms"][side]
            home = arm.get("home_joint_rad")
            drop = arm.get("bins", {}).get(destination)
            for pose in (home, drop):
                if (
                    not isinstance(pose, list)
                    or len(pose) != len(joints)
                    or not all(type(v) in (float, int) and math.isfinite(v) for v in pose)
                ):
                    raise ValueError(f"{side} needs measured stow and bin positions")
            self.stow.update({f"{side}_{name}_joint": q for name, q in zip(joints, home)})
        self.node, self.tolerance, self.max_age = node, tolerance, max_age
        self.pose_stamp, self.pose_at = 0, -math.inf
        self.state_at = self.last_request = -math.inf
        self.stowed = self.idle = False
        self.query = None
        self.state_client = node.create_client(Trigger, "manipulation/control_state")
        node.create_subscription(JointState, "joint_states", self.joints, qos_profile_sensor_data)
        super().__init__(transport, journal, time.monotonic, execution_profile="jetson")

    def joints(self, message):
        stamp = message.header.stamp
        self.pose_stamp = stamp.sec * 10**9 + stamp.nanosec
        self.pose_at = time.monotonic()
        positions = dict(zip(message.name, message.position))
        self.stowed = len(message.name) == len(message.position) == len(positions) and all(
            abs(positions.get(joint, math.inf) - q) <= self.tolerance for joint, q in self.stow.items()
        )

    def maintain(self):
        super().maintain()
        now = time.monotonic()
        if self.query is not None:
            if self.query.done():
                try:
                    reply = self.query.result()
                    data = json.loads(reply.message)
                    self.idle = (
                        reply.success
                        and data.get("schema") == 1
                        and data.get("collection_config_sha256") == self.digest
                        and data.get("operation_mode") == "full_collection"
                        and data.get("guard_fresh") is True
                        and (data.get("guard") or {}).get("state") == "ready"
                        and data.get("backend_ready") is True
                        and data.get("hardware_fault") == ""
                        and data.get("active_execution_id") == ""
                        and data.get("blocked") == []
                        and all(
                            data.get(k) is False
                            for k in (
                                "inhibited",
                                "external_motion_active",
                                "recovery_active",
                                "arm_return_active",
                                "payload_attached",
                            )
                        )
                    )
                    self.state_at = self.last_request
                except (ValueError, TypeError, AttributeError, RuntimeError):
                    self.idle = False
                self.query = None
            elif now - self.last_request > self.max_age:
                self.state_client.remove_pending_request(self.query)
                self.query, self.idle = None, False
        if (
            self.query is None
            and now - self.last_request >= self.max_age / 3
            and self.state_client.service_is_ready()
        ):
            self.query = self.state_client.call_async(Trigger.Request())
            self.last_request = now

    def ready(self):
        if not self.node.count_publishers("/perception/manipulation_outcome"):
            return False, "PLACEMENT_EVIDENCE_UNAVAILABLE"
        return super().ready()

    def safe_to_drive(self):
        now = time.monotonic()
        if self.human_review or (self.goal and not (self.outcome and self.outcome.safe_to_drive)):
            return False, "MANIPULATION_STATE_UNSAFE_OR_UNKNOWN"
        age = (self.node.get_clock().now().nanoseconds - self.pose_stamp) * 1e-9
        if not (
            self.stowed
            and self.idle
            and 0 <= age <= self.max_age
            and now - self.pose_at <= self.max_age
            and now - self.state_at <= self.max_age
        ):
            return False, "ARM_STOW_OR_OWNERSHIP_UNVERIFIED"
        return True, ""

    def stopped(self):
        return self.safe_to_drive()[0]
