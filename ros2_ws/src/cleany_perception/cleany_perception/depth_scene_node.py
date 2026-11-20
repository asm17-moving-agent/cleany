"""RViz와 MoveIt에 센서 전용 장면 점군을 발행합니다."""
from __future__ import annotations

import math
import time
from collections import OrderedDict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import SetBool

from cleany_perception.depth_scene_cloud import depth_scene_cloud


class DepthSceneNode(Node):
    """시각화와 mapping을 위해 센서 깊이를 장면 점군으로 발행합니다.

    Subscriptions:
        /camera/depth/image_rect_raw (sensor_msgs/msg/Image): 촬영 깊이.
        /camera/depth/camera_info (sensor_msgs/msg/CameraInfo): 투영 내부 파라미터.

    Publishers:
        /perception/scene_cloud (sensor_msgs/msg/PointCloud2): 촬영 좌표계의 비어 있지
            않은 측정 점군이며 원래 stamp를 유지합니다.

    QoS:
        입력은 best_effort, depth 1이고 출력은 reliable, depth 1입니다.

    Notes:
        토픽 이름은 변경 가능한 기본값입니다. 반복, 오래됨, 무효 또는 빈 표본은
        건너뛰며 출력 구독자가 없으면 점군을 계산하지 않습니다.
    """
    def __init__(self) -> None:
        super().__init__('depth_scene_node')
        defaults = {
            'depth_image_topic': '/camera/depth/image_raw',
            'depth_info_topic': '/camera/depth/camera_info',
            'cloud_topic': '/perception/scene_cloud',
            'updates_service': '/perception/scene_cloud_updates_enabled',
            'minimum_depth_m': 0.1,
            'maximum_depth_m': 2.0,
            'pixel_stride': 2,
            'publish_rate_hz': 2.0,
            'maximum_depth_age_sec': 1.0,
            'diagnostic_interval_sec': 5.0,
            'depth_16u_scale_m': 0.001,
            'workspace_roi_pixels': [-1.0, -1.0, -1.0, -1.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz must be positive and finite')
        interval = float(self.get_parameter('diagnostic_interval_sec').value)
        if not math.isfinite(interval) or interval <= 0:
            raise ValueError('diagnostic_interval_sec must be positive and finite')
        self._depth: Image | None = None
        self._info: CameraInfo | None = None
        self._depths = OrderedDict()
        self._infos = OrderedDict()
        self._published_stamp: tuple[int, int] | None = None
        self._updates_enabled = True
        self._created_wall_sec = time.monotonic()
        self._last_publication_wall_sec: float | None = None
        self._last_diagnostic_wall_sec: float | None = None
        self._input_counts = {'depth': 0, 'info': 0}
        # This is a live sensor stream, not a reliable command channel.  A
        # reliable publisher can block indefinitely while MoveIt temporarily
        # holds its planning-scene/octomap lock during a query.  Keeping the
        # newest best-effort cloud lets the 2 Hz producer continue and lets the
        # freshness guard reject any genuinely stale map.
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('cloud_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter('depth_image_topic').value),
            self._on_depth, qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('depth_info_topic').value),
            self._on_info, qos_profile_sensor_data,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter('updates_service').value),
            self._set_updates_enabled,
        )
        self.create_timer(1.0 / rate, self._publish)

    def _set_updates_enabled(self, request, response):
        """Pause/resume only the cloud feeding MoveIt's OctoMap updater."""
        self._updates_enabled = bool(request.data)
        response.success = True
        response.message = (
            'Scene cloud updates enabled'
            if self._updates_enabled
            else 'Scene cloud updates frozen'
        )
        self.get_logger().info(response.message)
        return response

    def _on_depth(self, message: Image) -> None:
        self._input_counts['depth'] += 1
        self._insert(self._depths, message)

    def _on_info(self, message: CameraInfo) -> None:
        self._input_counts['info'] += 1
        self._insert(self._infos, message)

    @staticmethod
    def _insert(cache, message):
        cache[(message.header.stamp.sec, message.header.stamp.nanosec)] = message
        while len(cache) > 12:
            cache.popitem(last=False)

    def _report_stall(self, reason: str) -> None:
        """Explain missing output without relaxing capture or matching checks."""
        created = getattr(self, '_created_wall_sec', None)
        if created is None:
            return
        now = time.monotonic()
        last_publication = self._last_publication_wall_sec
        gap = now - (created if last_publication is None else last_publication)
        interval = float(self.get_parameter('diagnostic_interval_sec').value)
        previous = self._last_diagnostic_wall_sec
        if gap < interval or (previous is not None and now - previous < interval):
            return
        now_ns = self.get_clock().now().nanoseconds

        def age(cache) -> str:
            if not cache:
                return 'not_received'
            sec, nanosec = max(cache)
            return f'{(now_ns - sec * 1_000_000_000 - nanosec) / 1e9:.3f}'

        common = self._depths.keys() & self._infos.keys()
        text = (
            f'Scene cloud stalled: reason={reason} output_gap_sec={gap:.3f} '
            f'subscribers={self._publisher.get_subscription_count()} '
            f'depth_received={self._input_counts["depth"]} '
            f'info_received={self._input_counts["info"]} '
            f'depth_cache={len(self._depths)} info_cache={len(self._infos)} '
            f'matching_stamps={len(common)} depth_age_sec={age(self._depths)} '
            f'info_age_sec={age(self._infos)} pair_age_sec={age(common)}'
        )
        logger = self.get_logger()
        if reason in ('updates_paused', 'no_cloud_subscriber'):
            logger.info(text)
        else:
            logger.warning(text)
        self._last_diagnostic_wall_sec = now

    def _publish(self) -> None:
        if not self._updates_enabled:
            DepthSceneNode._report_stall(self, 'updates_paused')
            return
        if not self._publisher.get_subscription_count():
            DepthSceneNode._report_stall(self, 'no_cloud_subscriber')
            return
        keys = self._depths.keys() & self._infos.keys()
        if not keys:
            reason = ('missing_depth' if not self._depths else
                      'missing_camera_info' if not self._infos else
                      'unmatched_timestamps')
            DepthSceneNode._report_stall(self, reason)
            return
        key = max(keys)
        self._depth, self._info = self._depths[key], self._infos[key]
        stamp = self._depth.header.stamp
        key = (stamp.sec, stamp.nanosec)
        age = (self.get_clock().now().nanoseconds
               - stamp.sec * 1_000_000_000 - stamp.nanosec) / 1e9
        maximum_age = float(self.get_parameter('maximum_depth_age_sec').value)
        if key == self._published_stamp:
            DepthSceneNode._report_stall(self, 'no_new_matching_frame')
            return
        if not 0 <= age <= maximum_age:
            DepthSceneNode._report_stall(self, 'future_depth' if age < 0 else 'stale_depth')
            return
        try:
            cloud = depth_scene_cloud(
                self._depth, self._info,
                float(self.get_parameter('minimum_depth_m').value),
                float(self.get_parameter('maximum_depth_m').value),
                int(self.get_parameter('pixel_stride').value),
                float(self.get_parameter('depth_16u_scale_m').value),
                workspace_roi_pixels=(
                    ()
                    if list(self.get_parameter('workspace_roi_pixels').value)
                    == [-1.0] * 4
                    else tuple(
                        self.get_parameter('workspace_roi_pixels').value
                    )
                ),
            )
        except ValueError as error:
            self.get_logger().warning(
                f'Scene cloud rejected: {error}', throttle_duration_sec=5
            )
            DepthSceneNode._report_stall(self, 'invalid_depth_or_calibration')
            return
        if not cloud.width:
            DepthSceneNode._report_stall(self, 'no_valid_depth_points')
            return
        self._publisher.publish(cloud)
        self._published_stamp = key
        self._last_publication_wall_sec = time.monotonic()
        if getattr(self, '_last_diagnostic_wall_sec', None) is not None:
            self.get_logger().info(
                f'Scene cloud recovered: points={cloud.width} capture_age_sec={age:.3f}'
            )
            self._last_diagnostic_wall_sec = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DepthSceneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
