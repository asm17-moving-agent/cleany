"""ROS adapter with one allowlisted read-only SubmapQuery request at a time."""

import struct
import time
import zlib
from concurrent.futures import ThreadPoolExecutor

from rosidl_runtime_py.utilities import get_service
from tf2_ros import TransformException

from .navigation import cloud_points
from .slam import SlamState, marker_geometry, texture_png


class SlamReader:
    def __init__(self, node):
        self.node = node
        self.config = node.config
        self.state = SlamState(
            self.config["slam_submap_limit"],
            self.config["slam_adjustment_m"],
            self.config["slam_adjustment_rad"],
            self.config["slam_texture_cache_bytes"],
        )
        self.topics = {
            self.config["slam_" + key + "_topic"]: key
            for key in ("submaps", "trajectory", "constraints", "matched_points")
        }
        self.client = None
        self.service_type = None
        self.request = None
        self.decode = None
        self.attempted = {}
        self.closed = False
        self.last_list_received = None
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="slam-texture")

    def status(self, state, reason=""):
        previous = self.node.hub.latest_data("slam", "status")
        if (
            previous.get("state") == state
            and previous.get("reason") == reason
            and previous.get("omitted_textures") == list(self.state.omitted_textures)
        ):
            return
        self.node.hub.put(
            "slam",
            "status",
            {
                "state": state,
                "reason": reason,
                "topic": self.config["slam_submaps_topic"],
                "epoch": self.state.epoch,
                "omitted_textures": list(self.state.omitted_textures),
            },
        )

    def reset(self):
        self.state.reset()
        self.attempted.clear()
        for row in self.node.hub.snapshot()["rows"]:
            if row["kind"] in ("slam", "slam_texture"):
                self.node.hub.forget(row["kind"], row["key"])

    def receive(self, topic, message, ros_time):
        key = self.topics[topic]
        try:
            if key == "submaps":
                tf = self.node.transform(message.header.frame_id, message.header.stamp)
                now = time.monotonic()
                if (
                    self.last_list_received is not None
                    and now - self.last_list_received > 3
                ):
                    self.reset()
                self.last_list_received = now
                data, events = self.state.update(message, tf)
                for event in events:
                    self.node.hub.put(
                        "event",
                        topic,
                        {**event, "epoch": self.state.epoch},
                        critical=True,
                        ros_time=ros_time,
                    )
                for row in self.node.hub.snapshot()["rows"]:
                    if (
                        row["kind"] == "slam_texture"
                        and row["key"] not in self.state.textures
                    ):
                        self.node.hub.forget(row["kind"], row["key"])
            elif key == "matched_points":
                points, count = cloud_points(
                    message, self.config["navigation_point_limit"]
                )
                frame = message.header.frame_id
                if frame != self.config["map_frame"]:
                    # Full 3D TF first, then project to XY (same path as navigation sensors).
                    from rclpy.time import Time

                    from .navigation import transform_xyz

                    tf = self.node.buffer.lookup_transform(
                        self.config["map_frame"],
                        frame,
                        Time.from_msg(message.header.stamp),
                    ).transform
                    points = transform_xyz(
                        points,
                        [tf.translation.x, tf.translation.y, tf.translation.z],
                        [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w],
                    )
                data = {
                    "valid": True,
                    "epoch": self.state.epoch,
                    "points": points,
                    "source_count": count,
                    "sampled": count > len(points),
                }
            else:
                data = self.state.markers(
                    key,
                    marker_geometry(
                        message,
                        self.node.transform,
                        self.config["slam_marker_point_limit"],
                    ),
                    self.config["slam_marker_point_limit"],
                )
            data.update(topic=topic, frame=self.config["map_frame"])
            self.node.hub.put("slam", key, data, ros_time=ros_time)
        except (
            TransformException,
            ValueError,
            TypeError,
            AttributeError,
            OverflowError,
            struct.error,
        ) as exc:
            self.node.hub.put(
                "slam",
                key,
                {
                    "valid": False,
                    "reason": str(exc),
                    "topic": topic,
                    "epoch": self.state.epoch,
                },
                ros_time=ros_time,
            )

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        if self.decode is not None:
            future, identity, epoch, version = self.decode
            if not future.done():
                return
            self.decode = None
            try:
                data = self.state.accept(identity, epoch, version, future.result())
                if data:
                    data["topic"] = self.config["slam_submaps_topic"]
                    self.node.hub.put("slam_texture", identity, data)
                    for row in self.node.hub.snapshot()["rows"]:
                        if (
                            row["kind"] == "slam_texture"
                            and row["key"] not in self.state.textures
                        ):
                            self.node.hub.forget(row["kind"], row["key"])
                    self.status("ready")
            except (ValueError, zlib.error, TypeError, AttributeError) as exc:
                self.status("decode_error", str(exc))
        if self.request is not None:
            future, identity, epoch, version, started = self.request
            if now - started > self.config["slam_query_timeout_s"]:
                self.client.remove_pending_request(future)
                future.cancel()
                self.request = None
                self.status("query_timeout", "서브맵 조회 응답 시간 초과")
            elif future.done():
                self.request = None
                try:
                    response = future.result()
                    if response.status.code != 0 or not response.textures:
                        raise ValueError(
                            response.status.message or "Empty submap texture"
                        )
                    # Active submaps can advance during the query/decode. Keep a
                    # verified earlier image with its own version, never relabel it.
                    self.decode = (
                        self.pool.submit(
                            texture_png,
                            response.textures[0],
                            self.config["slam_texture_pixel_limit"],
                        ),
                        identity,
                        epoch,
                        response.submap_version,
                    )
                except (ValueError, TypeError, AttributeError, RuntimeError) as exc:
                    self.status("query_error", str(exc))
            return
        if self.decode is not None:
            return
        latest = self.node.hub.latest_data("slam", "submaps")
        if not latest.get("valid"):
            return
        if self.last_list_received is not None and now - self.last_list_received > 3:
            self.status("source_stale", "서브맵 목록 수신 지연, 마지막 이미지 유지")
            return
        pending = self.state.pending()
        if not pending:
            self.status(
                "cache_limit", "이미지 메모리 한도, 일부 서브맵 이미지 생략"
            ) if self.state.omitted_textures else self.status("ready")
            return
        if self.client is None:
            try:
                self.service_type = get_service("cartographer_ros_msgs/srv/SubmapQuery")
                self.client = self.node.create_client(
                    self.service_type, self.config["slam_submap_query_service"]
                )
            except (ImportError, AttributeError, ValueError) as exc:
                self.status("type_missing", str(exc))
                return
        if not self.client.service_is_ready():
            self.status("service_missing", "Cartographer 서브맵 조회 서비스 대기")
            return
        candidate = min(pending, key=lambda entry: self.attempted.get(entry["id"], 0))
        if now - self.attempted.get(candidate["id"], 0) < 1:
            return
        request = self.service_type.Request()
        request.trajectory_id = candidate["trajectory_id"]
        request.submap_index = candidate["submap_index"]
        self.attempted[candidate["id"]] = now
        try:
            future = self.client.call_async(request)
            self.request = (
                future,
                candidate["id"],
                self.state.epoch,
                candidate["version"],
                now,
            )
            self.status("querying")
        except RuntimeError as exc:
            self.status("query_error", str(exc))

    def close(self):
        self.closed = True
        if self.request:
            self.client.remove_pending_request(self.request[0])
            self.request[0].cancel()
        if self.client is not None:
            self.node.destroy_client(self.client)
        self.pool.shutdown(wait=False, cancel_futures=True)
