from __future__ import annotations

from pathlib import Path

import mujoco
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from cleany_mujoco_sim.scene_loader import default_scene_path, load_model
from cleany_mujoco_sim.state import (
    apply_joint_cmd,
    camera_info_msg,
    depth_image_msg,
    image_msg,
    joint_state_msg,
    laser_scan_msg,
    odometry_msg,
    scan_sample_count,
    static_site_transform_msg,
    steps_per_tick,
    transform_msg,
)


class MujocoSimNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("mujoco_sim", **kwargs)

        self.declare_parameter('scene_path', '')
        self.declare_parameter('publish_rate_hz', 60.0)
        self.declare_parameter('headless', True)
        self.declare_parameter('base_body_name', 'chassis')
        self.declare_parameter('lidar_site_name', 'lidar_site')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('laser_frame_id', 'laser')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('scan_enabled', True)
        self.declare_parameter('scan_rate_hz', 5.5)
        self.declare_parameter('scan_sample_rate_hz', 8000.0)
        self.declare_parameter('scan_samples', 0)
        self.declare_parameter('scan_range_min', 0.15)
        self.declare_parameter('scan_range_max', 12.0)
        self.declare_parameter('camera_enabled', True)
        self.declare_parameter('camera_name', 'head_realsense_rgb')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_rate_hz', 15.0)
        self.declare_parameter('camera_frame_id', 'head_camera_rgb_optical_frame')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('depth_enabled', True)
        self.declare_parameter('depth_topic', 'depth')

        scene_path_value = self.get_parameter('scene_path').get_parameter_value().string_value
        scene_path = Path(scene_path_value) if scene_path_value else default_scene_path()
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self._headless = self.get_parameter('headless').get_parameter_value().bool_value
        self._base_body_name = self.get_parameter('base_body_name').get_parameter_value().string_value
        self._lidar_site_name = self.get_parameter('lidar_site_name').get_parameter_value().string_value
        self._odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self._base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self._laser_frame_id = self.get_parameter('laser_frame_id').get_parameter_value().string_value
        self._publish_odom_tf = self.get_parameter('publish_odom_tf').get_parameter_value().bool_value
        self._scan_enabled = self.get_parameter('scan_enabled').get_parameter_value().bool_value
        self._scan_rate_hz = self.get_parameter('scan_rate_hz').get_parameter_value().double_value
        scan_sample_rate_hz = (
            self.get_parameter('scan_sample_rate_hz').get_parameter_value().double_value
        )
        requested_scan_samples = (
            self.get_parameter('scan_samples').get_parameter_value().integer_value
        )
        self._scan_range_min = self.get_parameter('scan_range_min').get_parameter_value().double_value
        self._scan_range_max = self.get_parameter('scan_range_max').get_parameter_value().double_value
        self._camera_enabled = self.get_parameter('camera_enabled').get_parameter_value().bool_value
        self._camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self._camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self._camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self._camera_rate_hz = self.get_parameter('camera_rate_hz').get_parameter_value().double_value
        self._camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = (
            self.get_parameter('camera_info_topic').get_parameter_value().string_value
        )
        self._depth_enabled = self.get_parameter('depth_enabled').get_parameter_value().bool_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        if publish_rate_hz <= 0:
            raise ValueError('publish_rate_hz must be positive')
        if self._scan_enabled and self._scan_rate_hz <= 0:
            raise ValueError('scan_rate_hz must be positive')
        if self._scan_enabled and self._scan_range_min >= self._scan_range_max:
            raise ValueError('scan_range_min must be less than scan_range_max')
        if self._camera_enabled and self._camera_rate_hz <= 0:
            raise ValueError('camera_rate_hz must be positive')
        if self._camera_enabled and (self._camera_width <= 0 or self._camera_height <= 0):
            raise ValueError('camera_width and camera_height must be positive')

        if not scene_path.exists():
            raise FileNotFoundError(f"MuJoCo scene XML not found: {scene_path}")

        self._model, self._data = load_model(scene_path)
        self._base_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY, self._base_body_name
        )
        if self._base_body_id < 0:
            raise ValueError(f'MuJoCo body not found: {self._base_body_name}')
        self._lidar_site_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_SITE, self._lidar_site_name
        )
        if self._scan_enabled and self._lidar_site_id < 0:
            raise ValueError(f'MuJoCo site not found: {self._lidar_site_name}')
        self._camera_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_CAMERA, self._camera_name
        )
        if self._camera_enabled and self._camera_id < 0:
            raise ValueError(f'MuJoCo camera not found: {self._camera_name}')

        mujoco.mj_forward(self._model, self._data)
        self._steps_per_tick = steps_per_tick(self._model.opt.timestep, publish_rate_hz)
        self._scan_samples = 0
        self._sim_time_at_last_scan = 0.0
        if self._scan_enabled:
            self._scan_samples = scan_sample_count(
                requested_scan_samples, scan_sample_rate_hz, self._scan_rate_hz
            )
            self._sim_time_at_last_scan = -1.0 / self._scan_rate_hz

        self._renderer = None
        self._depth_renderer = None
        self._image_pub = None
        self._camera_info_pub = None
        self._depth_pub = None
        self._camera_fovy_deg = 0.0
        self._sim_time_at_last_frame = 0.0
        if self._camera_enabled:
            self._renderer = mujoco.Renderer(
                self._model, self._camera_height, self._camera_width
            )
            self._image_pub = self.create_publisher(Image, image_topic, 10)
            self._camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
            self._camera_fovy_deg = float(self._model.cam_fovy[self._camera_id])
            self._sim_time_at_last_frame = -1.0 / self._camera_rate_hz
            if self._depth_enabled:
                # Depth is rendered from the RGB camera so the two images share
                # pixels, frame and intrinsics -- mirroring the RealSense SDK's
                # depth-aligned-to-color stream instead of the raw depth sensor.
                self._depth_renderer = mujoco.Renderer(
                    self._model, self._camera_height, self._camera_width
                )
                self._depth_renderer.enable_depth_rendering()
                self._depth_pub = self.create_publisher(Image, depth_topic, 10)

        self._joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_subscription(JointState, '~/joint_cmd', self._on_joint_cmd, 10)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)
        if self._scan_enabled:
            self._static_tf_broadcaster.sendTransform(
                static_site_transform_msg(
                    self._model,
                    self._data,
                    self._base_body_id,
                    self._lidar_site_id,
                    self.get_clock().now(),
                    self._base_frame_id,
                    self._laser_frame_id,
                )
            )

        self._viewer = None
        if not self._headless:
            import mujoco.viewer as mujoco_viewer

            self._viewer = mujoco_viewer.launch_passive(self._model, self._data)

    def destroy_node(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
        if self._renderer is not None:
            self._renderer.close()
        if self._depth_renderer is not None:
            self._depth_renderer.close()

        super().destroy_node()

    def _on_joint_cmd(self, msg: JointState) -> None:
        apply_joint_cmd(self._model, self._data, msg)

    def _on_timer(self) -> None:
        for _ in range(self._steps_per_tick):
            mujoco.mj_step(self._model, self._data)
        stamp = self.get_clock().now()
        self._joint_state_pub.publish(joint_state_msg(self._model, self._data, stamp))
        self._odom_pub.publish(
            odometry_msg(
                self._model,
                self._data,
                self._base_body_id,
                stamp,
                self._odom_frame_id,
                self._base_frame_id,
            )
        )
        if self._publish_odom_tf:
            self._tf_broadcaster.sendTransform(
                transform_msg(
                    self._data,
                    self._base_body_id,
                    stamp,
                    self._odom_frame_id,
                    self._base_frame_id,
                )
            )
        should_publish_scan = (
            self._scan_enabled
            and self._data.time - self._sim_time_at_last_scan >= 1.0 / self._scan_rate_hz
        )
        if should_publish_scan:
            self._scan_pub.publish(
                laser_scan_msg(
                    self._model,
                    self._data,
                    self._lidar_site_id,
                    self._base_body_id,
                    stamp,
                    self._laser_frame_id,
                    self._scan_rate_hz,
                    self._scan_samples,
                    self._scan_range_min,
                    self._scan_range_max,
                )
            )
            self._sim_time_at_last_scan = self._data.time
        should_publish_image = (
            self._camera_enabled
            and self._data.time - self._sim_time_at_last_frame >= 1.0 / self._camera_rate_hz
        )
        if should_publish_image:
            self._renderer.update_scene(self._data, camera=self._camera_name)
            pixels = self._renderer.render()
            self._image_pub.publish(
                image_msg(pixels, stamp, self._camera_frame_id)
            )
            self._camera_info_pub.publish(
                camera_info_msg(
                    self._camera_width,
                    self._camera_height,
                    self._camera_fovy_deg,
                    stamp,
                    self._camera_frame_id,
                )
            )
            if self._depth_renderer is not None:
                self._depth_renderer.update_scene(self._data, camera=self._camera_name)
                depth = self._depth_renderer.render()
                self._depth_pub.publish(
                    depth_image_msg(depth, stamp, self._camera_frame_id)
                )
            self._sim_time_at_last_frame = self._data.time
        if self._viewer is not None:
            self._viewer.sync()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
