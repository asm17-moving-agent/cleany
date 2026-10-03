"""ROS reader. Only subscriptions, graph introspection and TF buffer lookups."""

import json
import os
import threading
import time
from collections import deque

import rclpy
from rclpy.action import (
    get_action_client_names_and_types_by_node,
    get_action_names_and_types,
    get_action_server_names_and_types_by_node,
)
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSCompatibility,
    QoSProfile,
    ReliabilityPolicy,
    qos_check_compatible,
)
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException, TransformListener

from .camera import IMAGE_TYPES
from .core import Hub, grid_origin, preview, transform_point, yaw
from .server import serve


class MonitorNode(Node):
    def __init__(self, **kwargs):
        super().__init__("cleany_dev_monitor", **kwargs)
        defaults = {
            "host": "127.0.0.1",
            "port": 8768,
            "state_directory": "~/.local/state/cleany/dev-monitor",
            "runtime_namespace": "",
            "manipulation_events_topic": "mock/manipulation/execution_events",
            "map_frame": "map",
            "base_frame": "base_link",
            "map_topic": "/map",
            "path_topic": "/plan",
            "local_costmap_topic": "/local_costmap/costmap",
            "global_costmap_topic": "/global_costmap/costmap",
            "velocity_topics": ["/nav2/cmd_vel", "/safety_2d/cmd_vel", "/cmd_vel"],
            "max_grid_cells": 262144,
            "max_selected_topics": 16,
            "camera_max_streams": 2,
            "camera_fps": 5.0,
            "session_limit_bytes": 104857600,
            "total_limit_bytes": 1073741824,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in defaults}
        if not 1 <= self.config["max_selected_topics"] <= 64:
            raise ValueError("max_selected_topics must be 1..64")
        if not 1 <= self.config["max_grid_cells"] <= 1048576:
            raise ValueError("max_grid_cells must be 1..1048576")
        if (
            not 1 <= self.config["camera_max_streams"] <= 4
            or not 1 <= self.config["camera_fps"] <= 10
        ):
            raise ValueError("Camera limits: streams 1..4, fps 1..10")
        self.hub = Hub()
        self.hub.cameras.limit = self.config["camera_max_streams"]
        self.hub.cameras.fps = self.config["camera_fps"]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        prefix = (
            "/" + self.config["runtime_namespace"].strip("/")
            if self.config["runtime_namespace"].strip("/")
            else ""
        )
        self.fixed = {
            f"{prefix}/mission/debug_snapshot",
            f"{prefix}/mission/runtime_events",
            f"{prefix}/mission/result",
            "/rosout",
            "/clock",
            *self.config["velocity_topics"],
        }
        self.layers = {
            self.config["map_topic"]: "map",
            self.config["path_topic"]: "path",
            self.config["local_costmap_topic"]: "local_costmap",
            self.config["global_costmap_topic"]: "global_costmap",
        }
        self.fixed.update(self.layers)
        event_topic = self.config["manipulation_events_topic"]
        self.manipulation_events = (
            event_topic if event_topic.startswith("/") else prefix + "/" + event_topic
        )
        self.fixed.add(self.manipulation_events)
        self.subs, self.rates, self.last_ui, self.errors = {}, {}, {}, {}
        self.last_clock = None
        self.last_clock_changed = time.monotonic()
        self.runtime_boot = None
        self.runtime_seq = None
        self.hub.put(
            "config",
            "monitor",
            {
                **self.config,
                "domain": os.environ.get("ROS_DOMAIN_ID", "0"),
                "namespace": self.get_namespace(),
                "read_only": True,
            },
        )
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.discover, clock=steady)
        self.create_timer(0.1, self.pose, clock=steady)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(
            target=serve, args=(self.hub, self.config, self.stop_event), daemon=True
        )
        self.worker.start()

    def discover(self):
        graph = {
            "topics": [],
            "nodes": [],
            "actions": [],
            "services": self.get_service_names_and_types(),
        }
        topics = dict(self.get_topic_names_and_types())
        selected = self.hub.selections()
        wanted = self.fixed | selected | self.hub.cameras.selected()
        # Action status/feedback are read-only public topics, not goal/result services.
        wanted.update(
            sorted(
                name
                for name in topics
                if "/_action/" in name and name.endswith(("/status", "/feedback"))
            )[:32]
        )
        for name in list(self.subs):
            if name not in wanted:
                self.destroy_subscription(self.subs.pop(name)[0])
                self.rates.pop(name, None)
                self.errors.pop(name, None)
                self.last_ui.pop(name, None)
                self.hub.forget("sample", name)
        now = time.monotonic()
        for name, types in topics.items():
            pubs = self.get_publishers_info_by_topic(name)
            subscriptions = self.get_subscriptions_info_by_topic(name)
            endpoints = lambda items: [
                {
                    "node": i.node_namespace.rstrip("/") + "/" + i.node_name,
                    "type": i.topic_type,
                    "qos": {
                        "reliability": i.qos_profile.reliability.name,
                        "durability": i.qos_profile.durability.name,
                        "history": i.qos_profile.history.name,
                        "depth": i.qos_profile.depth,
                    },
                }
                for i in items
            ]
            if name in wanted and pubs:
                reliable = all(
                    i.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                    for i in pubs
                )
                latched = all(
                    i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                    for i in pubs
                )
                qos = QoSProfile(
                    depth=1
                    if any(t in IMAGE_TYPES for t in types)
                    else (2048 if name.endswith("runtime_events") else 5),
                    reliability=ReliabilityPolicy.RELIABLE
                    if reliable
                    else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                    if latched
                    else DurabilityPolicy.VOLATILE,
                )
                signature = (tuple(types), reliable, latched)
                if name in self.subs and self.subs[name][1] != signature:
                    self.destroy_subscription(self.subs.pop(name)[0])
                if name not in self.subs:
                    try:
                        if len(types) != 1:
                            raise ValueError("Multiple message types on one topic")
                        for endpoint in pubs:
                            compatibility, reason = qos_check_compatible(
                                endpoint.qos_profile, qos
                            )
                            if compatibility == QoSCompatibility.ERROR:
                                raise ValueError("QoS mismatch: " + reason)
                        cls = get_message(types[0])
                        sub = self.create_subscription(
                            cls,
                            name,
                            lambda msg, topic=name: self.receive(topic, msg),
                            qos,
                        )
                        self.subs[name] = (sub, signature)
                        self.errors.pop(name, None)
                    except (
                        ImportError,
                        AttributeError,
                        ValueError,
                        RuntimeError,
                    ) as exc:
                        self.errors[name] = f"{type(exc).__name__}: {exc}"
            times = self.rates.get(name, ())
            recent = [t for t in times if now - t <= 5]
            hz = (
                (len(recent) - 1) / (recent[-1] - recent[0])
                if len(recent) > 1
                else None
            )
            state = (
                "no_publisher"
                if not pubs
                else "error"
                if name in self.errors
                else "unmeasured"
                if name not in self.subs
                else "no_messages"
                if not times
                else "stale"
                if now - times[-1] > 3
                else "receiving"
            )
            graph["topics"].append(
                {
                    "name": name,
                    "types": types,
                    "publishers": endpoints(pubs),
                    "subscribers": endpoints(subscriptions),
                    "state": state,
                    "error": self.errors.get(name),
                    "received_hz": hz,
                    "age": now - times[-1] if times else None,
                    "selected": name in selected,
                    "fixed": name in self.fixed,
                }
            )
        for name, namespace in self.get_node_names_and_namespaces():
            try:
                graph["nodes"].append(
                    {
                        "name": namespace.rstrip("/") + "/" + name,
                        "action_clients": get_action_client_names_and_types_by_node(
                            self, name, namespace
                        ),
                        "action_servers": get_action_server_names_and_types_by_node(
                            self, name, namespace
                        ),
                    }
                )
            except RuntimeError as exc:
                graph.setdefault("discovery_errors", []).append(
                    {"node": name, "reason": str(exc)}
                )
        graph["actions"] = get_action_names_and_types(self)
        graph["missing_fixed_topics"] = sorted(self.fixed - topics.keys())
        self.hub.put("graph", "ros", graph)

    def receive(self, topic, message):
        now = time.monotonic()
        self.rates.setdefault(topic, deque(maxlen=1000)).append(now)
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        ros_time = stamp.sec + stamp.nanosec / 1e9 if stamp else None
        try:
            if hasattr(message, "encoding") or (
                hasattr(message, "format") and hasattr(message, "data")
            ):
                self.hub.cameras.receive(topic, message)
            if topic.endswith("/mission/debug_snapshot"):
                if message.schema_version != 1 or len(message.snapshot_json) > 1048576:
                    raise ValueError("Unsupported/oversized Runtime debug snapshot")
                data = json.loads(message.snapshot_json)
                if self.runtime_boot != message.boot_id:
                    self.hub.put(
                        "event",
                        "connection",
                        {
                            "kind": "runtime_boot",
                            "boot_id": message.boot_id,
                            "previous_boot": self.runtime_boot,
                        },
                        critical=True,
                    )
                    self.runtime_boot, self.runtime_seq = message.boot_id, None
                self.hub.put("runtime", topic, data, ros_time=ros_time)
            elif topic.endswith("/mission/runtime_events"):
                if self.runtime_boot != message.boot_id:
                    self.hub.put(
                        "event",
                        "connection",
                        {
                            "kind": "runtime_boot",
                            "boot_id": message.boot_id,
                            "previous_boot": self.runtime_boot,
                        },
                        critical=True,
                    )
                    self.runtime_seq = None
                if message.sequence != (self.runtime_seq or 0) + 1:
                    self.hub.put(
                        "event",
                        "gap",
                        {
                            "kind": "runtime_sequence_gap",
                            "expected": (self.runtime_seq or 0) + 1,
                            "received": message.sequence,
                        },
                        critical=True,
                    )
                self.runtime_boot, self.runtime_seq = message.boot_id, message.sequence
                self.hub.put(
                    "event",
                    topic,
                    {
                        "boot_id": message.boot_id,
                        "sequence": message.sequence,
                        "mission_id": message.mission_id,
                        "task_id": message.task_id,
                        "execution_id": message.execution_id,
                        "kind": message.kind,
                        "data": json.loads(message.data_json),
                    },
                    critical=True,
                    ros_time=ros_time,
                )
            elif topic == self.manipulation_events:
                self.hub.put(
                    "event",
                    topic,
                    {"kind": "manipulation_record", **preview(message)},
                    critical=True,
                    ros_time=ros_time,
                )
            elif topic.endswith("/mission/result"):
                self.hub.put(
                    "result", topic, preview(message), critical=True, ros_time=ros_time
                )
            elif topic == "/rosout":
                self.hub.put(
                    "log",
                    topic,
                    preview(message),
                    critical=True,
                    ros_time=message.stamp.sec + message.stamp.nanosec / 1e9,
                )
            elif topic == "/clock":
                value = message.clock.sec + message.clock.nanosec / 1e9
                if self.last_clock is not None and value < self.last_clock:
                    self.hub.put(
                        "event",
                        "clock",
                        {"kind": "clock_rewind", "from": self.last_clock, "to": value},
                        critical=True,
                    )
                if value != self.last_clock:
                    self.last_clock_changed = now
                self.last_clock = value
                if now - self.last_ui.get(topic, 0) >= 0.1:
                    self.hub.put(
                        "clock",
                        topic,
                        {"time": value, "unchanged_for": now - self.last_clock_changed},
                    )
                    self.last_ui[topic] = now
            elif now - self.last_ui.get(topic, 0) >= 0.1:
                self.last_ui[topic] = now
                if topic in self.layers:
                    self.layer(topic, message, ros_time)
                else:
                    self.hub.put("sample", topic, preview(message), ros_time=ros_time)
        except (TypeError, AttributeError, ValueError, OverflowError) as exc:
            self.errors[topic] = f"Decode/transform: {exc}"

    def transform(self, frame, stamp=None):
        target = self.config["map_frame"]
        if not frame:
            raise ValueError("Empty frame_id")
        if frame == target:
            return [0.0, 0.0, 0.0]
        tf = self.buffer.lookup_transform(
            target, frame, Time.from_msg(stamp) if stamp else Time()
        )
        return [
            tf.transform.translation.x,
            tf.transform.translation.y,
            yaw(tf.transform.rotation),
        ]

    def layer(self, topic, message, ros_time):
        kind = self.layers[topic]
        try:
            transform = self.transform(message.header.frame_id, message.header.stamp)
            if kind == "path":
                points = []
                for pose in message.poses[:4096]:
                    tf = self.transform(
                        pose.header.frame_id or message.header.frame_id,
                        pose.header.stamp
                        if pose.header.frame_id
                        else message.header.stamp,
                    )
                    points.append(
                        transform_point(pose.pose.position.x, pose.pose.position.y, tf)
                    )
                data = {"points": points, "truncated": len(message.poses) > 4096}
            else:
                info = message.info
                cells = info.width * info.height
                if cells > self.config["max_grid_cells"]:
                    raise ValueError(
                        f"Grid too large: {cells} cells; limit {self.config['max_grid_cells']}"
                    )
                if len(message.data) != cells:
                    raise ValueError("Grid dimensions mismatch")
                data = {
                    "width": info.width,
                    "height": info.height,
                    "resolution": info.resolution,
                    "origin": grid_origin(
                        [
                            info.origin.position.x,
                            info.origin.position.y,
                            yaw(info.origin.orientation),
                        ],
                        transform,
                    ),
                    "cells": list(message.data),
                }
            self.hub.put(
                "layer",
                kind,
                {
                    **data,
                    "frame": self.config["map_frame"],
                    "source_frame": message.header.frame_id,
                    "valid": True,
                    "topic": topic,
                },
                ros_time=ros_time,
            )
        except (
            TransformException,
            ValueError,
            TypeError,
            AttributeError,
            RuntimeError,
        ) as exc:
            self.hub.put(
                "layer",
                kind,
                {"valid": False, "reason": str(exc), "topic": topic},
                ros_time=ros_time,
            )

    def pose(self):
        try:
            pose = self.transform(self.config["base_frame"])
            tf = self.buffer.lookup_transform(
                self.config["map_frame"], self.config["base_frame"], Time()
            )
            age = self.get_clock().now().nanoseconds / 1e9 - (
                tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            )
            self.hub.put(
                "pose",
                "robot",
                {
                    "valid": abs(age) <= 2,
                    "pose": pose,
                    "age": age,
                    "reason": "" if abs(age) <= 2 else "TF stale or future",
                    "frame": self.config["map_frame"],
                },
            )
        except (
            TransformException,
            ValueError,
            TypeError,
            AttributeError,
            RuntimeError,
        ) as exc:
            self.hub.put("pose", "robot", {"valid": False, "reason": str(exc)})

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
