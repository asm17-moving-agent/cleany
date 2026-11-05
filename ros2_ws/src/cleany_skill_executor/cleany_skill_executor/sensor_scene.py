"""Read-only readiness checks for the live MoveIt depth map."""
from __future__ import annotations

import math
import threading
import time

from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header

from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
from cleany_skill_executor.core.sensor_scene import validate_sensor_scene


class SensorSceneMonitor:
    def __init__(self, node):
        self.node = node
        defaults = {
            'require_sensor_scene': False,
            'sensor_scene_maximum_age_sec': 2.0,
            'sensor_scene_startup_timeout_sec': 10.0,
            'sensor_scene_receipt_topic': '/perception/scene_cloud_receipt',
            'sensor_scene_topic': '/monitored_planning_scene',
            'sensor_scene_service': '/get_planning_scene',
            'sensor_scene_snapshot_period_sec': 1.0,
        }
        for key, value in defaults.items():
            node.declare_parameter(key, value)
        self.required = bool(node.get_parameter('require_sensor_scene').value)
        self.maximum_age = float(node.get_parameter('sensor_scene_maximum_age_sec').value)
        self.timeout = float(node.get_parameter('sensor_scene_startup_timeout_sec').value)
        if not all(math.isfinite(v) and v > 0 for v in (self.maximum_age, self.timeout)):
            raise ValueError('Sensor scene ages/timeouts must be finite and positive')
        self._lock = threading.Lock()
        self._closed = False
        self._capture_ns = 0
        self._map_received = 0.0
        self._snapshot_received = 0.0
        self._map_id = ''
        self._map_bytes = 0
        self._subscriptions = []
        if self.required:
            group = ReentrantCallbackGroup()
            period = float(node.get_parameter('sensor_scene_snapshot_period_sec').value)
            if not math.isfinite(period) or not 0 < period < self.maximum_age:
                raise ValueError('Map snapshot period must be shorter than its maximum age')
            self._snapshot_client = node.create_client(GetPlanningScene,
                str(node.get_parameter('sensor_scene_service').value), callback_group=group)
            self._snapshot_future = None
            self._snapshot_sent = 0.
            self._snapshot_timer = node.create_timer(period, self._request_snapshot, callback_group=group)
            self._subscriptions = [
                node.create_subscription(Header, str(node.get_parameter('sensor_scene_receipt_topic').value),
                                         self.on_receipt, 1, callback_group=group),
                node.create_subscription(PlanningScene, str(node.get_parameter('sensor_scene_topic').value),
                                         self.on_scene, 1, callback_group=group),
            ]

    def _request_snapshot(self):
        # Static occupancy may not appear in state-only scene diffs, and a new
        # subscriber may miss the first full scene. Read the authoritative map.
        if self._closed:
            return
        future = self._snapshot_future
        if future is not None and not future.done():
            if time.monotonic()-self._snapshot_sent > self.maximum_age:
                future.cancel()
                self._snapshot_future = None
            return
        if not self._snapshot_client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.OCTOMAP
        self._snapshot_sent = time.monotonic()
        future = self._snapshot_client.call_async(request)
        self._snapshot_future = future

        def completed(result):
            if self._closed or result.cancelled():
                return
            try:
                response = result.result()
                if response is not None:
                    self.on_scene(response.scene)
            except Exception:
                # No freshness renewal when the map query fails.
                pass

        future.add_done_callback(completed)

    def close(self):
        """Stop map polling while the executor can still drain cancellation."""
        self._closed = True
        timer = getattr(self, '_snapshot_timer', None)
        if timer is not None:
            timer.cancel()
        future = getattr(self, '_snapshot_future', None)
        if future is not None and not future.done():
            future.cancel()

    def on_receipt(self, header):
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        with self._lock:
            if header.frame_id and self._capture_ns < stamp <= now:
                self._capture_ns = stamp
                # Humble's stock updater emits the filtered cloud after updating
                # the tree. Advancing processed captures can renew a populated
                # static map, whose geometry diff may not change on every frame.
                if self._map_id == 'OcTree' and self._map_bytes:
                    self._map_received = time.monotonic()

    def on_scene(self, scene):
        with self._lock:
            octomap = scene.world.octomap.octomap
            if octomap.data:
                self._map_id, self._map_bytes = octomap.id, len(octomap.data)
                self._map_received = time.monotonic()
                self._snapshot_received = self._map_received
            elif not scene.is_diff:
                self._map_id, self._map_bytes, self._map_received = '', 0, 0.0
                self._snapshot_received = 0.0

    def check(self, *, after_stamp_ns=0):
        if not self.required:
            return
        with self._lock:
            capture_ns = self._capture_ns
            map_age = time.monotonic() - self._map_received
            snapshot_age = time.monotonic() - self._snapshot_received
            capture_age = (self.node.get_clock().now().nanoseconds - capture_ns) / 1e9
            try:
                validate_sensor_scene(capture_age, self.maximum_age, self._map_id, self._map_bytes)
                if (capture_ns <= after_stamp_ns or not 0 <= map_age <= self.maximum_age
                        or not 0 <= snapshot_age <= self.maximum_age):
                    raise RuntimeError('Map receipt/capture has not advanced or is stale')
            except RuntimeError as error:
                raise InfrastructureError(
                    f'Sensor planning scene unavailable: {error}; '
                    f'capture_age={capture_age:.3f}s map_age={map_age:.3f}s') from error

    def wait(self, *, after_stamp_ns=0, canceled=lambda: False):
        if not self.required:
            return
        deadline = time.monotonic() + self.timeout
        while True:
            if canceled():
                raise InterruptedError('Sensor scene wait canceled')
            try:
                self.check(after_stamp_ns=after_stamp_ns)
                return
            except InfrastructureError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.01)
