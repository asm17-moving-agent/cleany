"""Shared ROS execution path for calibration motion; no report generation."""

import math
import time
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_srvs.srv import SetBool
except ImportError:
    rclpy = None

from evaluator_core import OdomGuardError
from motion_control import MotionConfig, MotionController


def init_motion_runtime(node: Any, config: MotionConfig | None = None) -> None:
    node.motion_config = config or MotionConfig.from_json()
    node.motion_velocity = None
    node.motion_results = []
    node.last_failure = ""
    node._disable_requested = False
    node._motion_active = False


def request_abort(node: Any, reason: str) -> None:
    """Callback-safe STOP: publish zero and request disable without nested spinning."""
    node.aborted = True
    node.last_failure = reason
    node.cmd_vel_pub.publish(Twist())
    if not node._disable_requested:
        node._disable_requested = True
        req = SetBool.Request()
        req.data = False
        node.enable_client.call_async(req)


def receive_odometry(node: Any, msg: Any) -> None:
    if node.aborted:
        return
    pos, ori, velocity = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
    values = (ori.x, ori.y, ori.z, ori.w, velocity.linear.x,
              velocity.linear.y, velocity.angular.z)
    try:
        if hasattr(node, "get_clock"):
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            age_s = (node.get_clock().now().nanoseconds - stamp_ns) / 1e9
            # Queued old packets do not refresh the receipt watchdog.
            if abs(age_s) > node.guard.timeout_s:
                return
        if not all(math.isfinite(v) for v in values):
            raise OdomGuardError("Non-finite odometry orientation or velocity")
        if abs(sum(v * v for v in values[:4]) - 1) > .01:
            raise OdomGuardError("Invalid odometry quaternion norm")
        yaw = math.atan2(
            2 * (ori.w * ori.z + ori.x * ori.y),
            1 - 2 * (ori.y * ori.y + ori.z * ori.z),
        )
        node.current_pose = node.guard.update(
            msg.header.stamp.sec, msg.header.stamp.nanosec, pos.x, pos.y, yaw
        )
        node.motion_velocity = (
            math.hypot(velocity.linear.x, velocity.linear.y), velocity.angular.z
        )
    except OdomGuardError as error:
        node.get_logger().error(f"[ODOM_GUARD_FAULT] {error}")
        request_abort(node, str(error))


def stop_motion(node: Any) -> None:
    node.cmd_vel_pub.publish(Twist())
    if not node.set_base_enabled(False):
        node.last_failure = node.last_failure or "Base disable was not acknowledged"
        node.aborted = True


def await_fresh_pose(node: Any, timeout_s: float = 1.0) -> Any:
    """Passive stale data may recover; an existing safety latch may not."""
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not node.aborted and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        try:
            pose = node.guard.check_freshness(trip_on_timeout=False)
            if node.motion_velocity is not None:
                return pose
        except OdomGuardError:
            if node.guard.is_tripped:
                break
    raise OdomGuardError("No fresh odometry and velocity before motion")


def execute_motion(
    node: Any, vx: float, vy: float, wz: float, target_displacement: float,
    is_rotation: bool, dwell_after_s: float, timeout_s: float,
) -> tuple[bool, dict[str, Any]]:
    result: dict[str, Any] = {
        "is_valid": False, "reached_tolerance": False, "failure_reason": "",
        "target_displacement": target_displacement,
        "motion_config": node.motion_config.to_dict(), "trace": [],
    }
    node.motion_results.append(result)
    start_time = time.monotonic()
    try:
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("motion timeout must be finite and positive")
        if not math.isfinite(dwell_after_s) or dwell_after_s < 0:
            raise ValueError("motion dwell must be finite and nonnegative")
        start = await_fresh_pose(node)
        controller = MotionController(
            start, vx, vy, wz, target_displacement, is_rotation, node.motion_config
        )
        if hasattr(node, "get_publishers_info_by_topic"):
            publishers = node.get_publishers_info_by_topic("/cmd_vel")
            if len(publishers) != 1 or any(p.node_name != node.get_name() for p in publishers):
                raise OdomGuardError("Another /cmd_vel publisher is active")
        if not node.set_base_enabled(True):
            raise OdomGuardError("Failed to enable base")
        node.guard.check_freshness()
        if node.aborted:
            raise OdomGuardError(node.last_failure or "Motion aborted during enable")
        node._motion_active = True
        max_overshoot = 0.0
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not node.aborted:
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            pose = node.guard.check_freshness()
            if node.aborted:
                break
            linear_speed, angular_speed = node.motion_velocity
            step = controller.step(pose, linear_speed, angular_speed, now)
            max_overshoot = max(max_overshoot, step.overshoot)
            result["trace"].append({
                "elapsed_s": now - start_time, "x_m": pose.x_m, "y_m": pose.y_m,
                "yaw_rad": pose.yaw_rad, "linear_speed_mps": linear_speed,
                "angular_speed_rad_s": angular_speed, "position_error": step.error,
                "command": [step.vx_mps, step.vy_mps, step.wz_rad_s],
            })
            if step.done:
                break
            if now >= deadline:
                raise OdomGuardError(f"Motion timed out after {timeout_s:.1f}s")
            command = Twist()
            command.linear.x, command.linear.y = step.vx_mps, step.vy_mps
            command.angular.z = step.wz_rad_s
            node.cmd_vel_pub.publish(command)
            time.sleep(0.02)
        else:
            raise OdomGuardError(node.last_failure or "ROS stopped during motion")
        if node.aborted:
            raise OdomGuardError(node.last_failure or "Motion aborted")
        dwell_end = time.monotonic() + dwell_after_s
        while rclpy.ok() and not node.aborted and time.monotonic() < dwell_end:
            node.cmd_vel_pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.005)
            pose = node.guard.check_freshness()
            step = controller.step(pose, *node.motion_velocity, time.monotonic())
            max_overshoot = max(max_overshoot, step.overshoot)
            time.sleep(0.02)
        if node.aborted or not rclpy.ok():
            raise OdomGuardError(node.last_failure or "ROS stopped during dwell")
        pose = node.guard.check_freshness()
        step = controller.step(pose, *node.motion_velocity, time.monotonic())
        result.update({
            "actual_displacement": step.actual_displacement,
            "final_error": step.error, "max_overshoot": max_overshoot,
            "duration_s": time.monotonic() - start_time,
            "reached_tolerance": step.done, "is_valid": step.done,
        })
        if is_rotation:
            result["overshoot_deg"] = math.degrees(step.overshoot)
        else:
            result["overshoot_mm"] = step.overshoot * 1000
        if not step.done:
            raise OdomGuardError("Final position/speed is outside completion tolerance")
    except (OdomGuardError, ValueError) as error:
        result["is_valid"] = False
        result["failure_reason"] = str(error)
        node.get_logger().error(f"[MOTION_ABORT] {error}")
        request_abort(node, str(error))
    finally:
        node._motion_active = False
        stop_motion(node)
        if node.aborted:
            result["is_valid"] = False
            result["failure_reason"] = node.last_failure
    return result["is_valid"], result
