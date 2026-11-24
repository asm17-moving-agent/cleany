"""Correlate a desk inspection with the inspector's capture-stamped evidence image."""

import math
import time
from collections import deque
from pathlib import Path
from uuid import UUID, uuid4

from action_msgs.msg import GoalStatus
from cleany_interfaces.action import InspectScene
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cleany_mission_manager.core.result import FailureCode, ModuleResult, ResultStatus
from cleany_mission_manager.core.runtime_models import SceneObject, SceneSnapshot
from cleany_telemetry.camera import encode_image


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def scene_objects(detections, allowed_labels):
    """Keep untracked or unclassified objects visible for human review."""
    tracks = [item.track_id for item in detections if item.track_id]
    local_ids = [item.object_id for item in detections]
    if len(set(local_ids)) != len(local_ids) or any(i <= 0 for i in local_ids):
        raise ValueError("invalid inspection object IDs")
    objects = []
    for item in detections:
        tracked = item.tracking_state == "TRACKED" and item.track_id and tracks.count(item.track_id) == 1
        allowed = (
            tracked
            and item.label.casefold().strip() in allowed_labels
            and item.sorting_category == "trash"
            and item.distance_valid
            and math.isfinite(item.distance_m)
            and item.distance_m > 0
            and item.position_valid
        )
        identity = "track:" + item.track_id if tracked else f"untracked:{item.object_id}"
        objects.append(
            SceneObject(identity, "collect_trash" if allowed else "human_review", item.object_id)
        )
    return tuple(objects)


class PhysicalScenePort:
    """Read one fresh phase-one scene per operation, without commanding the robot."""

    def __init__(self, node, action, image_topic, directory, allowed_labels, timeout):
        self.node = node
        self.client = ActionClient(node, InspectScene, action)
        self.root = Path(directory).expanduser().resolve()
        self.latest_capture = 0
        self.allowed = {label.strip().casefold() for label in allowed_labels if label.strip()}
        if not self.allowed or not self.root.is_dir():
            raise ValueError(
                "physical inspection requires allowed labels and an existing observation directory"
            )
        self.timeout = timeout
        self.image_topic = image_topic
        self.images = deque(maxlen=3)
        node.create_subscription(Image, image_topic, self.image, qos_profile_sensor_data)
        self.operation = self.mission = None
        self.handle = self.sent = self.finished = None
        self.cancelled = False
        self.started_ns = 0
        self.tracking_epoch = None

    def image(self, msg):
        if self.operation and stamp_ns(msg.header) >= self.started_ns:
            self.images.append(msg)

    def ready(self):
        if not self.node.count_publishers(self.image_topic):
            return False, "INSPECTION_IMAGE_UNAVAILABLE"
        return (True, "") if self.client.server_is_ready() else (False, "INSPECTION_UNAVAILABLE")

    def stopped(self):
        if self.operation and self.cancelled:
            self.poll(self.operation)
        return self.operation is None

    def start(self, mission_id):
        if self.operation:
            raise RuntimeError("inspection already active")
        mission = str(UUID(str(mission_id)))
        if mission != self.mission:
            self.tracking_epoch = None
            self.latest_capture = 0
        self.mission = mission
        self.images.clear()
        self.started_ns = self.node.get_clock().now().nanoseconds
        self.sent = self.client.send_goal_async(InspectScene.Goal(tracking_session_id=self.mission))
        self.operation = str(uuid4())
        self.handle = self.finished = None
        self.cancelled = False
        return self.operation

    def poll(self, operation_id):
        if operation_id != self.operation:
            return None
        try:
            if not self.sent.done():
                return None
            if self.handle is None:
                self.handle = self.sent.result()
                if not self.handle.accepted:
                    raise ValueError("inspection rejected")
                self.finished = self.handle.get_result_async()
                if self.cancelled:
                    self.handle.cancel_goal_async()
            if not self.finished.done():
                return None
            response = self.finished.result()
            if self.cancelled:
                result = ModuleResult(False, ResultStatus.CANCELLED)
            elif response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.success:
                raise ValueError("inspection failed: " + response.result.message)
            else:
                scene = response.result.detections
                stamp = stamp_ns(scene.header)
                age = (self.node.get_clock().now().nanoseconds - stamp) * 1e-9
                if (
                    not scene.snapshot_id
                    or scene.tracking_session_id != self.mission
                    or not scene.tracking_epoch
                    or stamp <= self.latest_capture
                    or stamp < self.started_ns
                    or not 0 <= age <= self.timeout
                ):
                    raise ValueError("inspection capture is stale or belongs to another mission")
                if self.tracking_epoch is not None and scene.tracking_epoch != self.tracking_epoch:
                    raise ValueError("tracking epoch changed during the mission")
                self.tracking_epoch = scene.tracking_epoch
                image = next(
                    (
                        m
                        for m in self.images
                        if stamp_ns(m.header) == stamp and m.header.frame_id == scene.header.frame_id
                    ),
                    None,
                )
                if image is None:
                    return None
                objects = scene_objects(scene.detections, self.allowed)
                jpeg, _, _ = encode_image(image)
                # Both path components are locally validated/generated UUIDs.
                folder = self.root / self.mission
                folder.mkdir(mode=0o700, exist_ok=True)
                relative = f"{self.mission}/{self.operation}.jpg"
                temp = folder / (self.operation + ".tmp")
                try:
                    temp.write_bytes(jpeg)
                    temp.replace(self.root / relative)
                finally:
                    temp.unlink(missing_ok=True)
                self.latest_capture = stamp
                result = ModuleResult.success(
                    SceneSnapshot(
                        scene.snapshot_id,
                        time.monotonic() - age,
                        objects,
                        "observation://" + relative,
                        source="real",
                    )
                )
        except (OSError, ValueError, RuntimeError) as exc:
            result = ModuleResult.failed(FailureCode.PERCEPTION_FAIL, message=str(exc))
        self.operation = None
        self.images.clear()
        return result

    def cancel(self, operation_id):
        if self.operation == operation_id and not self.cancelled:
            self.cancelled = True
            if self.handle is not None and self.handle.accepted:
                self.handle.cancel_goal_async()
