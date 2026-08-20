from __future__ import annotations

import time
from collections.abc import Iterable
from pathlib import Path

import mujoco
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from cleany_mujoco_sim.base_command import (
    ChassisCommand,
    CommandLimits,
    are_finite_values,
    bounded_command,
    stopped_command,
)
from cleany_mujoco_sim.extensions import (
    MujocoSimulationContext,
    StepObserver,
)
from cleany_mujoco_sim.mecanum_kinematics import (
    MecanumGeometry,
    WheelSpeedLimit,
    limited_wheel_speeds,
    stopped_wheel_speeds,
    wheel_speeds_from_chassis,
)
from cleany_mujoco_sim.mujoco_drive import MujocoMecanumDrive
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
from cleany_mujoco_sim.wheel_speed_controller import (
    PidGains,
    VelocityControllerConfig,
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
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_linear_y', 0.3)
        self.declare_parameter('max_angular_z', 0.8)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('timeout_check_rate_hz', 20.0)
        self.declare_parameter('wheel_radius', 0.0635)
        self.declare_parameter('wheelbase_length', 0.30)
        self.declare_parameter('track_width', 0.51)
        self.declare_parameter('max_wheel_speed', 10.815)
        self.declare_parameter('base_drive_enabled', True)
        self.declare_parameter('wheel_kp', 1.0)
        self.declare_parameter('wheel_ki', 5.0)
        self.declare_parameter('wheel_kd', 0.0)
        self.declare_parameter('motor_voltage_limit', 10.8)
        self.declare_parameter('motor_no_load_speed', 10.815)
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
        self._command_limits = CommandLimits(
            max_linear_x=float(self.get_parameter('max_linear_x').value),
            max_linear_y=float(self.get_parameter('max_linear_y').value),
            max_angular_z=float(self.get_parameter('max_angular_z').value),
        )
        self._cmd_vel_timeout_sec = float(
            self.get_parameter('cmd_vel_timeout_sec').value
        )
        timeout_check_rate_hz = float(
            self.get_parameter('timeout_check_rate_hz').value
        )
        self._mecanum_geometry = MecanumGeometry(
            wheel_radius=float(self.get_parameter('wheel_radius').value),
            wheelbase_length=float(self.get_parameter('wheelbase_length').value),
            track_width=float(self.get_parameter('track_width').value),
        )
        self._wheel_speed_limit = WheelSpeedLimit(
            max_abs=float(self.get_parameter('max_wheel_speed').value)
        )
        self._base_drive_enabled = bool(
            self.get_parameter('base_drive_enabled').value
        )
        self._velocity_controller_config = VelocityControllerConfig(
            gains=PidGains(
                kp=float(self.get_parameter('wheel_kp').value),
                ki=float(self.get_parameter('wheel_ki').value),
                kd=float(self.get_parameter('wheel_kd').value),
            ),
            voltage_limit=float(self.get_parameter('motor_voltage_limit').value),
            no_load_speed=float(self.get_parameter('motor_no_load_speed').value),
        )
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
        timeout_values = (self._cmd_vel_timeout_sec, timeout_check_rate_hz)
        if not are_finite_values(timeout_values) or any(
            value <= 0.0 for value in timeout_values
        ):
            raise ValueError('command timeout values must be positive and finite')

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
        self._mujoco_drive = (
            MujocoMecanumDrive(
                self._model,
                self._velocity_controller_config,
            )
            if self._base_drive_enabled
            else None
        )

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
        self._current_chassis_command = stopped_command()
        self._target_wheel_speeds = stopped_wheel_speeds()
        self._last_cmd_vel_time: float | None = None
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
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)
        self.create_timer(1.0 / timeout_check_rate_hz, self._on_cmd_vel_timeout)
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

    def _on_cmd_vel(self, msg: Twist) -> None:
        all_axes = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        if not are_finite_values(all_axes):
            self.get_logger().warning(
                'Ignoring non-finite cmd_vel and stopping the base'
            )
            self._stop_chassis_command()
            return

        unsupported_axes = (msg.linear.z, msg.angular.x, msg.angular.y)
        if any(value != 0.0 for value in unsupported_axes):
            self.get_logger().warning(
                'Ignoring unsupported cmd_vel axes; only linear.x, linear.y, '
                'and angular.z are used'
            )

        try:
            chassis_command = bounded_command(
                ChassisCommand(
                    linear_x=msg.linear.x,
                    linear_y=msg.linear.y,
                    angular_z=msg.angular.z,
                ),
                self._command_limits,
            )
            target_wheel_speeds = limited_wheel_speeds(
                wheel_speeds_from_chassis(
                    chassis_command,
                    self._mecanum_geometry,
                ),
                self._wheel_speed_limit,
            )
        except ValueError:
            self.get_logger().warning(
                'Ignoring invalid cmd_vel and stopping the base'
            )
            self._stop_chassis_command()
            return

        self._current_chassis_command = chassis_command
        self._target_wheel_speeds = target_wheel_speeds
        self._last_cmd_vel_time = time.monotonic()

    def _on_cmd_vel_timeout(self) -> None:
        if self._last_cmd_vel_time is None:
            return
        if time.monotonic() - self._last_cmd_vel_time <= self._cmd_vel_timeout_sec:
            return

        self.get_logger().warning('cmd_vel timed out; stopping the base')
        self._stop_chassis_command()

    def _stop_chassis_command(self) -> None:
        self._current_chassis_command = stopped_command()
        self._target_wheel_speeds = stopped_wheel_speeds()
        self._last_cmd_vel_time = None
        if self._mujoco_drive is not None:
            self._mujoco_drive.reset()

    @property
    def simulation_context(self) -> MujocoSimulationContext:
        return self._simulation_context

    def add_step_observer(self, observer: StepObserver) -> None:
        self._step_observers.append(observer)

    def _on_timer(self) -> None:
        for _ in range(self._steps_per_tick):
            if self._mujoco_drive is not None:
                self._mujoco_drive.apply_control(
                    self._data,
                    self._target_wheel_speeds,
                    self._model.opt.timestep,
                )
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
