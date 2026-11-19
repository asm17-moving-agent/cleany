#!/usr/bin/env python3
"""Run automated holonomic odometry verification and ground-truth comparison."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import SetBool
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

import numpy as np
from lidar_scan_matcher import LidarScanMatcher, ScanMatchResult, plot_scan_match, validate_scan_match
from evaluation_results import json_finite, match_metadata, write_data_json
from motion_control import MotionConfig
from motion_runtime import execute_motion, init_motion_runtime, receive_odometry, stop_motion
from run_oneway_eval import compute_relative_odom, save_scan_capture, transform_lidar_to_base

from evaluator_core import (
    MAX_ALLOWED_ORIGIN_DISTANCE_M,
    MAX_LINEAR_VELOCITY_MPS,
    MAX_ANGULAR_VELOCITY_RAD_S,
    EvaluationPlan,
    GroundTruthResult,
    OdomFreshnessGuard,
    OdomGuardError,
    PatternBuilder,
    Pose2D,
    diagnose_error,
)


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdometryEvaluatorNode(Node if HAVE_ROS else object):  # type: ignore[misc]

    def __init__(
        self,
        plan: EvaluationPlan,
        timeout_s: float = 60.0,
        use_lidar: bool = True,
        odom_timeout_s: float = 0.15,
        motion_config: MotionConfig | None = None,
    ) -> None:
        if not HAVE_ROS:
            raise RuntimeError("ROS 2 environment is not available")
        super().__init__("odometry_evaluator")
        self.plan = plan
        self.timeout_s = timeout_s
        self.use_lidar = use_lidar

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.enable_client = self.create_client(SetBool, "/base/enable")
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, 1
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._on_scan,
            QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.matcher = LidarScanMatcher(voxel_size_m=0.015)
        self.guard = OdomFreshnessGuard(timeout_s=odom_timeout_s)
        self.recent_scans: list[LaserScan] = []

        self.current_pose: Pose2D | None = None
        self.start_pose: Pose2D | None = None
        self.aborted = False
        init_motion_runtime(self, motion_config)

    def _on_scan(self, msg: LaserScan) -> None:
        self.recent_scans.append(msg)
        if len(self.recent_scans) > 40:
            self.recent_scans.pop(0)

    def capture_scan_burst(
        self, duration_s: float = 1.2, min_scans: int = 6, timeout_s: float = 3.5
    ) -> np.ndarray | None:
        self.recent_scans.clear()
        start_time = time.monotonic()
        deadline = start_time + timeout_s
        min_time = start_time + duration_s
        while time.monotonic() < deadline and rclpy.ok() and not self.aborted:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() >= min_time and len(self.recent_scans) >= min_scans:
                break
        if self.aborted:
            return None
        if len(self.recent_scans) < min_scans:
            self.get_logger().warn(
                f"Insufficient scans captured ({len(self.recent_scans)}/{min_scans})"
            )
            return None
        points_list = []
        for msg in self.recent_scans:
            pts = self.matcher.laserscan_to_points(
                msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max
            )
            if len(pts) > 0:
                points_list.append(pts)
        if not points_list:
            return None
        combined = np.vstack(points_list)
        return self.matcher.downsample_points(combined)

    def _on_odom(self, msg: Odometry) -> None:
        receive_odometry(self, msg)

    def set_base_enabled(self, enable: bool) -> bool:
        if not self.enable_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service /base/enable not available")
            return False
        req = SetBool.Request()
        req.data = enable
        future = self.enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            res = future.result()
            self.get_logger().info(f"/base/enable({enable}) -> success={res.success}: {res.message}")
            return res.success
        return False

    def stop_robot(self) -> None:
        stop_motion(self)

    def execute_segment(self, seg) -> bool:
        success, _ = execute_motion(
            self, seg.vx_mps, seg.vy_mps, seg.wz_rad_s, seg.target_displacement,
            seg.is_rotation, seg.dwell_after_s, self.timeout_s,
        )
        return success

    def run_plan(self) -> tuple[Pose2D, Pose2D, np.ndarray | None, np.ndarray | None] | None:
        self.get_logger().info("Waiting for /odom messages...")
        wait_deadline = time.monotonic() + 10.0
        while self.current_pose is None and time.monotonic() < wait_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_pose is None:
            self.get_logger().error("Did not receive initial /odom within 10s")
            return None

        self.start_pose = self.current_pose
        self.get_logger().info(
            f"Initial pose recorded: x={self.start_pose.x_m:.4f}m, y={self.start_pose.y_m:.4f}m, yaw={self.start_pose.yaw_deg:.2f}deg"
        )

        start_scan_pts: np.ndarray | None = None
        end_scan_pts: np.ndarray | None = None

        if self.use_lidar:
            self.get_logger().info("Waiting for initial /scan messages...")
            scan_deadline = time.monotonic() + 5.0
            while not self.recent_scans and time.monotonic() < scan_deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

            self.get_logger().info("Capturing baseline LiDAR scan burst at start position...")
            start_scan_pts = self.capture_scan_burst(duration_s=1.2, min_scans=6, timeout_s=4.0)
            if start_scan_pts is not None:
                self.get_logger().info(f"Captured {len(start_scan_pts)} baseline scan points.")
            else:
                self.get_logger().warn("Could not capture baseline LiDAR scan. Continuing with odom only.")

        for seg in self.plan.segments:
            if self.aborted or not rclpy.ok():
                self.stop_robot()
                return None
            success = self.execute_segment(seg)
            if not success:
                self.stop_robot()
                return None

        self.stop_robot()
        end_pose = self.current_pose or self.start_pose

        if self.use_lidar:
            self.get_logger().info("Capturing final LiDAR scan burst after returning...")
            end_scan_pts = self.capture_scan_burst(duration_s=1.2, min_scans=6, timeout_s=4.0)
            if end_scan_pts is not None:
                self.get_logger().info(f"Captured {len(end_scan_pts)} final scan points.")

        return self.start_pose, end_pose, start_scan_pts, end_scan_pts


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify holonomic odometry error with floor marking ground truth"
    )
    parser.add_argument(
        "--pattern",
        choices=["surge", "return_x", "sway", "return_y", "yaw", "yaw_cw", "yaw_ccw", "box", "diagonal"],
        default="surge",
        help="Motion pattern to evaluate (default: surge)",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=None,
        help="Translation distance in meters (clamped <= 3.0m). Defaults: surge=2.5m, sway=2.0m, box=2.0m, diag=2.0m",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=MAX_LINEAR_VELOCITY_MPS,
        help=f"Target linear speed (max {MAX_LINEAR_VELOCITY_MPS} m/s)",
    )
    parser.add_argument(
        "--turns",
        type=float,
        default=1.0,
        help="Number of full turns for pure yaw evaluation (default: 1.0)",
    )
    parser.add_argument(
        "--yaw-speed",
        type=float,
        default=MAX_ANGULAR_VELOCITY_RAD_S,
        help=f"Target angular speed (max {MAX_ANGULAR_VELOCITY_RAD_S} rad/s)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Path to save evaluation JSON results",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print motion trajectory and verification plan without connecting to ROS",
    )
    parser.add_argument(
        "--no-lidar",
        action="store_true",
        help="Disable automatic LiDAR scan matching ground-truth estimation",
    )
    parser.add_argument(
        "--tape-error-x",
        type=float,
        default=None,
        help="Physical X error in mm (ahead/forward of tape is positive). If omitted, prompts user interactively.",
    )
    parser.add_argument(
        "--tape-error-y",
        type=float,
        default=None,
        help="Physical Y error in mm (left of tape is positive). If omitted, prompts user interactively.",
    )
    parser.add_argument(
        "--tape-error-yaw",
        type=float,
        default=None,
        help="Physical Yaw error in deg (CCW is positive). If omitted, prompts user interactively.",
    )
    parser.add_argument(
        "--odom-timeout",
        type=float,
        default=0.15,
        help="Odometry watchdog freshness timeout in seconds (default: 0.15s)",
    )
    parser.add_argument("--manual-fallback", action="store_true",
                        help="Require explicit interactive tape measurements after ICP rejection")
    parser.add_argument("--motion-config", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/odometry_evaluation"))
    parser.add_argument("--no-plots", action="store_true")
    return parser.parse_args(argv)


def get_plan(args: argparse.Namespace) -> EvaluationPlan:
    if args.pattern == "surge":
        dist = args.distance if args.distance is not None else 2.5
        return PatternBuilder.build_surge(distance_m=dist, speed_mps=args.speed)
    elif args.pattern == "return_x":
        dist = args.distance if args.distance is not None else 2.5
        return PatternBuilder.build_return_x(distance_m=dist, speed_mps=args.speed)
    elif args.pattern == "sway":
        dist = args.distance if args.distance is not None else 2.0
        return PatternBuilder.build_sway(distance_m=dist, speed_mps=args.speed)
    elif args.pattern == "return_y":
        dist = args.distance if args.distance is not None else 2.0
        return PatternBuilder.build_return_y(distance_m=dist, speed_mps=args.speed)
    elif args.pattern == "yaw":
        return PatternBuilder.build_yaw(turns=args.turns, speed_rad_s=args.yaw_speed)
    elif args.pattern == "yaw_cw":
        return PatternBuilder.build_yaw_cw(turns=args.turns, speed_rad_s=args.yaw_speed)
    elif args.pattern == "yaw_ccw":
        return PatternBuilder.build_yaw_ccw(turns=args.turns, speed_rad_s=args.yaw_speed)
    elif args.pattern == "box":
        side = args.distance if args.distance is not None else 2.0
        return PatternBuilder.build_box(side_m=side, speed_mps=args.speed)
    elif args.pattern == "diagonal":
        dist = args.distance if args.distance is not None else 2.0
        return PatternBuilder.build_diagonal(distance_m=dist, speed_mps=args.speed)
    else:
        raise ValueError(f"Unknown pattern: {args.pattern}")


def prompt_physical_error(args: argparse.Namespace) -> tuple[float, float, float]:
    if (args.tape_error_x is not None and args.tape_error_y is not None
            and args.tape_error_yaw is not None):
        values = args.tape_error_x, args.tape_error_y, args.tape_error_yaw
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Manual measurements must be finite")
        return values

    if not getattr(args, "manual_fallback", False):
        raise ValueError("No complete explicit manual measurement; automatic fallback is disabled")

    print("\n" + "=" * 60)
    print(">>> Ground Truth Floor Tape Measurement Entry <<<")
    print("Check robot reference alignment relative to start tape mark.")
    print("=" * 60)

    def read_float(prompt: str) -> float:
        try:
            value = float(input(f"{prompt}: ").strip())
        except (ValueError, EOFError) as error:
            raise ValueError("Missing or invalid explicit manual measurement") from error
        if not math.isfinite(value):
            raise ValueError("Manual measurements must be finite")
        return value

    err_x = args.tape_error_x if args.tape_error_x is not None else read_float(
        "Enter physical X error relative to tape mark in mm (forward/overshoot + / undershoot -)"
    )
    err_y = args.tape_error_y if args.tape_error_y is not None else read_float(
        "Enter physical Y error relative to tape mark in mm (left + / right -)"
    )
    err_yaw = args.tape_error_yaw if args.tape_error_yaw is not None else read_float(
        "Enter physical Yaw error relative to tape mark in deg (CCW + / CW -)"
    )
    return err_x, err_y, err_yaw


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    plan = get_plan(args)
    plan.validate()
    config = MotionConfig.from_json(args.motion_config)
    OdomFreshnessGuard(timeout_s=args.odom_timeout)
    if args.dry_run:
        print(f"=== DRY-RUN: Evaluation Plan [{plan.pattern_name.upper()}] ===")
        print(f"Max displacement from origin: {plan.max_origin_distance_m:.3f} m (Limit <= {MAX_ALLOWED_ORIGIN_DISTANCE_M} m)")
        print(f"Nominal total distance: {plan.nominal_total_distance_m:.3f} m")
        print(f"Nominal total rotation: {math.degrees(plan.nominal_total_yaw_rad):.1f} deg")
        print("Segments:")
        for idx, seg in enumerate(plan.segments, 1):
            unit = "rad" if seg.is_rotation else "m"
            print(f"  {idx}. {seg.name:18s} | vx={seg.vx_mps:+.2f} vy={seg.vy_mps:+.2f} wz={seg.wz_rad_s:+.2f} | target={seg.target_displacement:.2f}{unit} | dwell={seg.dwell_after_s:.1f}s")
        print("Dry-run validation successful.")
        return 0
    if not HAVE_ROS:
        print("Run inside distrobox ros2-humble.", file=sys.stderr)
        return 1
    from datetime import datetime
    import fcntl
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output = args.output or args.output_dir / f"odom_eval_{plan.pattern_name}_{timestamp}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        print(f"Refusing to replace existing data: {output}", file=sys.stderr)
        return 1
    lock_file = open("/tmp/cleany_odom_eval.lock", "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        print("Another calibration motion tool is active.", file=sys.stderr)
        return 1
    rclpy.init()
    node = OdometryEvaluatorNode(
        plan=plan, use_lidar=not args.no_lidar, odom_timeout_s=args.odom_timeout,
        motion_config=config,
    )
    data = {
        "schema_version": 2, "timestamp": timestamp, "pattern_name": plan.pattern_name,
        "status": "failed", "is_valid": False, "failure_reason": "",
        "motion_config": config.to_dict(), "odom_timeout_s": args.odom_timeout,
        "residual_frame": "start_base",
        "error_rate_reference": "nominal_total_path_length_or_rotation",
    }
    exit_code = 1

    def interrupt(sig, frame):
        node.aborted = True
        node.last_failure = f"Interrupted by signal {sig}"
        raise KeyboardInterrupt()

    handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        results = node.run_plan()
        if results is None:
            raise RuntimeError(node.last_failure or "Motion or scan capture failed")
        node.stop_robot()
        start, end, start_points, end_points = results
        dx, dy, yaw = compute_relative_odom(start, end)
        physical = (None, None, None)
        method, lidar_info = "missing_measurement", None
        capture_files = []
        for label, points in (("start", start_points), ("end", end_points)):
            if points is not None:
                path = output.with_name(f"{output.stem}_{label}.npz")
                save_scan_capture(None if label == "start" else node, points, path)
                capture_files.append(str(path))
        data["capture_files"] = capture_files
        if not args.no_lidar and start_points is not None and end_points is not None:
            match = node.matcher.match(start_points, end_points)
            lidar_info = match_metadata(match)
            lidar_info["inlier_ratio_pct"] = match.inlier_ratio * 100
            if match.is_valid:
                bx, by, byaw = transform_lidar_to_base(
                    match.delta_x_m, match.delta_y_m, match.delta_yaw_rad,
                    extrinsic_yaw_rad=math.radians(-7.85),
                )
                physical = (bx * 1000, by * 1000, math.degrees(byaw))
                method = "lidar_icp"
                if not args.no_plots:
                    plot_scan_match(
                        start_points, end_points, match, str(output.with_suffix(".png")),
                        title=f"LiDAR ICP [{plan.pattern_name.upper()}]",
                    )
            else:
                method = "rejected_icp"
                print(f"Rejected ICP: {match.rejection_reason}", file=sys.stderr)
        if physical[0] is None:
            try:
                physical = prompt_physical_error(args)
                method = "manual_tape"
            except ValueError as error:
                data["failure_reason"] = str(error)
        result = GroundTruthResult(
            pattern_name=plan.pattern_name,
            nominal_distance_m=plan.nominal_total_distance_m,
            nominal_yaw_deg=math.degrees(plan.nominal_total_yaw_rad),
            odom_start=start, odom_end=end,
            odom_delta_x_m=dx, odom_delta_y_m=dy,
            odom_delta_yaw_deg=math.degrees(yaw),
            physical_error_x_mm=physical[0], physical_error_y_mm=physical[1],
            physical_error_yaw_deg=physical[2],
            measurement_method=method, lidar_metrics=lidar_info,
            diagnostic_hint="",
        )
        data.update(result.to_dict())
        data["capture_files"] = capture_files
        data["status"] = "success" if result.measurement_valid else "failed"
        data["extrinsic_calibration_applied"] = {
            "yaw_deg": -7.85, "lever_arm_x_m": .029, "lever_arm_y_m": 0,
        }
        exit_code = 0 if result.measurement_valid else 1
        print(result.format_summary_table())
    except (KeyboardInterrupt, Exception) as error:
        data["failure_reason"] = node.last_failure or str(error) or "Interrupted"
        data["status"], data["is_valid"] = "failed", False
        print(f"Evaluation failed: {data['failure_reason']}", file=sys.stderr)
    finally:
        try:
            node.stop_robot()
            if node.aborted:
                data["status"], data["is_valid"] = "failed", False
                data["failure_reason"] = node.last_failure or data["failure_reason"]
                exit_code = 1
            data["motion_runs"] = node.motion_results
            write_data_json(output, data)
            print(f"Data: {output}")
        except (OSError, TypeError, ValueError, RuntimeError) as error:
            print(f"Stop or data save failed: {error}", file=sys.stderr)
            exit_code = 1
        finally:
            node.destroy_node()
            rclpy.shutdown()
            lock_file.close()
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
