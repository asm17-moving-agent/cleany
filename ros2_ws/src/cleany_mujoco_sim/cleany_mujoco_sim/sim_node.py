from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from cleany_mujoco_sim.extensions import (
    MujocoSimulationContext,
    StepObserver,
)
from cleany_mujoco_sim.scene_loader import default_scene_path, load_model
from cleany_mujoco_sim.state import (
    apply_joint_cmd,
    initialize_joint_positions,
    joint_state_msg,
    laser_scan_msg,
    odometry_msg,
    scan_sample_count,
    static_site_transform_msg,
    steps_per_tick,
    transform_msg,
)


class MujocoSimNode(Node):
    def __init__(
        self,
        step_observers: Iterable[StepObserver] | None = None,
        **kwargs,
    ) -> None:
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
        self.declare_parameter(
            'initial_joint_names', Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter(
            'initial_joint_positions', Parameter.Type.DOUBLE_ARRAY
        )

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
        initial_joint_names = list(
            self.get_parameter_or(
                'initial_joint_names',
                Parameter(
                    'initial_joint_names',
                    Parameter.Type.STRING_ARRAY,
                    [],
                ),
            )
            .get_parameter_value()
            .string_array_value
        )
        initial_joint_positions = list(
            self.get_parameter_or(
                'initial_joint_positions',
                Parameter(
                    'initial_joint_positions',
                    Parameter.Type.DOUBLE_ARRAY,
                    [],
                ),
            )
            .get_parameter_value()
            .double_array_value
        )

        if publish_rate_hz <= 0:
            raise ValueError('publish_rate_hz must be positive')
        if self._scan_enabled and self._scan_rate_hz <= 0:
            raise ValueError('scan_rate_hz must be positive')
        if self._scan_enabled and self._scan_range_min >= self._scan_range_max:
            raise ValueError('scan_range_min must be less than scan_range_max')

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

        initialize_joint_positions(
            self._model,
            self._data,
            initial_joint_names,
            initial_joint_positions,
        )
        mujoco.mj_forward(self._model, self._data)
        self._simulation_context = MujocoSimulationContext(
            model=self._model,
            data=self._data,
        )
        self._step_observers = list(step_observers or ())
        self._steps_per_tick = steps_per_tick(self._model.opt.timestep, publish_rate_hz)
        self._scan_samples = 0
        self._sim_time_at_last_scan = 0.0
        if self._scan_enabled:
            self._scan_samples = scan_sample_count(
                requested_scan_samples, scan_sample_rate_hz, self._scan_rate_hz
            )
            self._sim_time_at_last_scan = -1.0 / self._scan_rate_hz

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

        super().destroy_node()

    def _on_joint_cmd(self, msg: JointState) -> None:
        apply_joint_cmd(self._model, self._data, msg)

    @property
    def simulation_context(self) -> MujocoSimulationContext:
        return self._simulation_context

    def add_step_observer(self, observer: StepObserver) -> None:
        self._step_observers.append(observer)

    def _on_timer(self) -> None:
        for _ in range(self._steps_per_tick):
            mujoco.mj_step(self._model, self._data)
        stamp = self.get_clock().now()
        for observer in self._step_observers:
            observer.after_step(self._simulation_context, stamp)
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
