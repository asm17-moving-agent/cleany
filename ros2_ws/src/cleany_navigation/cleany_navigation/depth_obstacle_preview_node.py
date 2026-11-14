"""이동 권한에 연결하지 않고 측정 깊이 장애물을 표시합니다."""

import json
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.depth_clearance import decode_depth_image, project_depth_image, rectified_intrinsics
from cleany_navigation.depth_obstacles import ObstacleFilter, filter_obstacles
from cleany_navigation.point_cloud import xyz_cloud
from cleany_navigation.transforms import transform


class DepthObstaclePreviewNode(Node):
    """실제 pan/tilt TF로 시각이 있는 장애물 미리보기를 발행합니다.

    Subscriptions:
        /camera/d435/depth/image_rect_raw (Image): 왜곡 보정된 광학 깊이.
        /camera/d435/depth/camera_info (CameraInfo): 대응하는 보정 내부 파라미터.
        /scan_raw (LaserScan): 명목 물리 높이의 LiDAR 시각화.

    Publishers:
        /perception/depth/measured_points, obstacle_points, ground_points
        (PointCloud2): 베이스 좌표계 관측 hit와 공간 필터링 부분 집합.
        /perception/depth/filter_status (String): 개수와 측정 상태.
        /perception/lidar/cad_points (PointCloud2): 명목 물리 LiDAR 레이어.

    TF:
        촬영 시각의 base_link에서 optical까지 TF를 사용합니다. 별도 고정 좌표계
        d435_ground_preview와 lidar_cad_preview를 발행하며 주행용 lidar_link는
        변경하지 않습니다. 미리보기 바닥은 관측 평면이며 독립 참값이 아닙니다.

    QoS:
        영상, 스캔과 점군은 sensor_data, 상태는 reliable, depth 10입니다.

    Notes:
        costmap, 공간 제거 또는 명령 출력이 없습니다. TF가 없거나 무효 또는 오래되면
        영상을 버립니다. 센서가 끊긴 뒤에는 빈 점군으로 오래된 미리보기 점을 지웁니다.
    """

    def __init__(self) -> None:
        super().__init__('depth_obstacles_preview')
        for key, value in [('filter_file', ''), ('body_boxes_file', ''), ('base_frame', 'base_link'),
                           ('pixel_stride', 3), ('tf_wait_s', .15), ('max_age_s', .5),
                           ('lidar_cad_xyz', [.139275, 0., .192]), ('lidar_yaw', float(np.pi))]:
            self.declare_parameter(key, value)
        self.config = ObstacleFilter(**json.loads(Path(self.get_parameter('filter_file').value).read_text()))
        self.base = self.get_parameter('base_frame').value
        body = json.loads(Path(self.get_parameter('body_boxes_file').value).read_text())
        if body['frame'] != self.base:
            raise ValueError('Body geometry frame does not match depth base frame')
        self.boxes = [BodyBox(b['name'], b['center'], np.eye(3), b['half_size']) for b in body['boxes']]
        self.stride = self.get_parameter('pixel_stride').value
        self.wait_s, self.max_age = self.get_parameter('tf_wait_s').value, self.get_parameter('max_age_s').value
        if self.stride < 1 or not 0 < self.wait_s <= self.max_age:
            raise ValueError('Invalid stride/TF freshness limits')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.static = StaticTransformBroadcaster(self)
        self._publish_preview_frames()
        self.raw = self.create_publisher(PointCloud2, '/perception/depth/measured_points', qos_profile_sensor_data)
        self.obstacles = self.create_publisher(PointCloud2, '/perception/depth/obstacle_points', qos_profile_sensor_data)
        self.ground = self.create_publisher(PointCloud2, '/perception/depth/ground_points', qos_profile_sensor_data)
        self.lidar = self.create_publisher(PointCloud2, '/perception/lidar/cad_points', qos_profile_sensor_data)
        self.status = self.create_publisher(String, '/perception/depth/filter_status', 10)
        self.camera, self.pending = None, None
        self.pending_at, self.last_good_at = 0., time.monotonic()
        self.stale_sent = False
        self.create_subscription(CameraInfo, '/camera/d435/depth/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/d435/depth/image_rect_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
        self.create_timer(.01, self.process_pending)

    def _publish_preview_frames(self) -> None:
        normal = np.asarray(self.config.floor_normal)
        x_axis = np.array([1., 0., -normal[0]/normal[2]])
        x_axis /= np.linalg.norm(x_axis)
        matrix = np.column_stack((x_axis, np.cross(normal, x_axis), normal))
        descriptions = [
            ('d435_ground_preview', [0., 0., self.config.floor_offset/normal[2]], Rotation.from_matrix(matrix).as_quat()),
            ('lidar_cad_preview', self.get_parameter('lidar_cad_xyz').value,
             Rotation.from_euler('z', self.get_parameter('lidar_yaw').value).as_quat()),
        ]
        messages = []
        for child, xyz, xyzw in descriptions:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = self.base, child
            t.header.stamp = self.get_clock().now().to_msg()
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, xyzw)
            messages.append(t)
        self.static.sendTransform(messages)

    def on_info(self, message: CameraInfo) -> None:
        self.camera = message

    def on_depth(self, message: Image) -> None:
        self.pending, self.pending_at = message, time.monotonic()
        self.process_pending()

    def on_scan(self, message: LaserScan) -> None:
        ranges = np.asarray(message.ranges)
        angle = message.angle_min + np.arange(len(ranges))*message.angle_increment
        keep = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= min(message.range_max, 4.))
        p = np.column_stack((ranges[keep]*np.cos(angle[keep]), ranges[keep]*np.sin(angle[keep]), np.zeros(keep.sum())))
        self.lidar.publish(xyz_cloud(p, 'lidar_cad_preview', message.header.stamp))

    def clear_preview(self, reason: str) -> None:
        empty = xyz_cloud(np.empty((0, 3)), self.base, self.get_clock().now().to_msg())
        for pub in (self.raw, self.obstacles, self.ground):
            pub.publish(empty)
        self.status.publish(String(data=json.dumps({'state': 'unavailable', 'reason': reason})))

    def process_pending(self) -> None:
        if time.monotonic()-self.last_good_at > self.max_age and not self.stale_sent:
            self.clear_preview('no fresh transformed depth')
            self.stale_sent = True
        msg, camera = self.pending, self.camera
        if msg is None or camera is None:
            return
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds*1e-9
        if time.monotonic()-self.pending_at > self.wait_s or age > self.max_age or age < -.05:
            self.pending = None
            self.clear_preview('stale depth or unavailable capture-time TF')
            return
        when = Time.from_msg(msg.header.stamp)
        if not self.buffer.can_transform(self.base, msg.header.frame_id, when):
            return
        self.pending = None
        if (msg.width, msg.height, msg.header.frame_id) != (camera.width, camera.height, camera.header.frame_id) or any(not np.isfinite(v) or abs(v)>1e-9 for v in camera.d):
            self.clear_preview('depth/CameraInfo mismatch')
            return
        try:
            intrinsics = rectified_intrinsics(camera.k, camera.p)
            if intrinsics is None:
                raise ValueError('Invalid rectified intrinsics')
            depth = decode_depth_image(msg.data, msg.width, msg.height, msg.step, msg.encoding, msg.is_bigendian)
            forward = project_depth_image(depth, intrinsics, pixel_stride=self.stride)
            optical = np.column_stack((-forward[:, 1], -forward[:, 2], forward[:, 0]))
            pose = self.buffer.lookup_transform(self.base, msg.header.frame_id, when).transform
            p = transform(optical, pose)
            origin = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
            output, counts = filter_obstacles(p, origin, self.boxes, self.config)
        except (ValueError, TransformException) as exc:
            self.clear_preview(str(exc))
            return
        self.raw.publish(xyz_cloud(p, self.base, msg.header.stamp))
        self.obstacles.publish(xyz_cloud(output, self.base, msg.header.stamp))
        height = p @ np.asarray(self.config.floor_normal)-self.config.floor_offset
        self.ground.publish(xyz_cloud(p[abs(height)<.03], self.base, msg.header.stamp))
        self.last_good_at, self.stale_sent = time.monotonic(), False
        counts.update(state='available', stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                      min_base_z=self.config.min_base_z)
        self.status.publish(String(data=json.dumps(counts)))


def main() -> None:
    rclpy.init()
    node = DepthObstaclePreviewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
