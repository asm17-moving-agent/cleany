#!/usr/bin/env python3
"""Automated One-Way (편도) Holonomic Odometry Evaluation via 2D LiDAR ICP Ground Truth.

Measures displacement differences during axial, diagonal and rotation motion
without error cancellation from round-trip return legs.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any, Sequence

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MATPLOTLIB = True
except ImportError:
    HAVE_MATPLOTLIB = False

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

from evaluator_core import OdomFreshnessGuard, OdomGuardError, Pose2D
from evaluation_results import build_leg_result, json_finite, match_metadata, write_data_json
from motion_control import MotionConfig
from motion_runtime import execute_motion, init_motion_runtime, receive_odometry, stop_motion
from planar_extrinsic import PlanarExtrinsic
from motion_patterns import motion_vectors


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
from lidar_scan_matcher import (
    LidarScanMatcher,
    ScanMatchResult,
    plot_scan_match,
    validate_scan_match,
)


class OneWayEvaluatorNode(Node if HAVE_ROS else object):  # type: ignore[misc]

    def __init__(
        self,
        min_range_m: float = 0.38,
        voxel_size_m: float = 0.015,
        odom_timeout_s: float = 0.15,
        motion_config: MotionConfig | None = None,
        extrinsic: PlanarExtrinsic | None = None,
        odom_topic: str = "/odom",
        min_inlier_ratio: float = 0.40,
        convergence_config: dict | None = None,
    ) -> None:
        if not HAVE_ROS:
            raise RuntimeError("ROS 2 (rclpy) is not available")
        super().__init__("oneway_odometry_evaluator")

        self.extrinsic = extrinsic or PlanarExtrinsic()
        self.matcher = LidarScanMatcher(
            min_range_m=min_range_m, voxel_size_m=voxel_size_m,
            min_inlier_ratio=min_inlier_ratio,
            convergence_config=convergence_config,
        )
        self.guard = OdomFreshnessGuard(timeout_s=odom_timeout_s)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.enable_client = self.create_client(SetBool, "/base/enable")
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self._on_scan,
            QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.current_pose: Pose2D | None = None
        self.recent_scans: list[LaserScan] = []
        self.aborted = False
        init_motion_runtime(self, motion_config)

    def _on_odom(self, msg: Odometry) -> None:
        receive_odometry(self, msg)

    def _on_scan(self, msg: LaserScan) -> None:
        self.recent_scans.append(msg)
        if len(self.recent_scans) > 30:
            self.recent_scans.pop(0)

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

    def execute_leg(
        self, vx: float, vy: float, wz: float, target_displacement: float,
        is_rotation: bool, dwell_after_s: float = 1.5, timeout_s: float = 30.0,
    ) -> tuple[bool, dict[str, Any]]:
        return execute_motion(
            self, vx, vy, wz, target_displacement, is_rotation, dwell_after_s, timeout_s
        )


def transform_lidar_to_base(
    dx_l: float,
    dy_l: float,
    dyaw_l: float,
    extrinsic_yaw_rad: float = -0.1370,
    lever_arm_x_m: float = 0.029,
    lever_arm_y_m: float = 0.0,
) -> tuple[float, float, float]:
    """Transform relative displacement from LiDAR frame to Base frame."""
    c_e = math.cos(extrinsic_yaw_rad)
    s_e = math.sin(extrinsic_yaw_rad)
    R_e = np.array([[c_e, -s_e], [s_e, c_e]])

    p_l = np.array([dx_l, dy_l])
    p_b_trans = R_e @ p_l

    # Lever arm displacement due to in-place yaw rotation
    c_y = math.cos(dyaw_l)
    s_y = math.sin(dyaw_l)
    R_yaw = np.array([[c_y, -s_y], [s_y, c_y]])
    arm = np.array([lever_arm_x_m, lever_arm_y_m])
    lever_shift = arm - (R_yaw @ arm)

    p_b = p_b_trans + lever_shift
    return float(p_b[0]), float(p_b[1]), float(dyaw_l)


def compute_relative_odom(p_from: Pose2D, p_to: Pose2D) -> tuple[float, float, float]:
    """Compute relative displacement (dx, dy, dyaw) in p_from robot coordinate frame."""
    dx_world = p_to.x_m - p_from.x_m
    dy_world = p_to.y_m - p_from.y_m
    c = math.cos(p_from.yaw_rad)
    s = math.sin(p_from.yaw_rad)
    # Rotate world delta into p_from robot frame
    dx_local = c * dx_world + s * dy_world
    dy_local = -s * dx_world + c * dy_world
    dyaw = (p_to.yaw_rad - p_from.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
    return dx_local, dy_local, dyaw


def save_scan_capture(node: Any, points: np.ndarray, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {"points": points}
    for index, msg in enumerate(getattr(node, "recent_scans", [])):
        arrays[f"ranges_{index}"] = np.asarray(msg.ranges, dtype=float)
        arrays[f"metadata_{index}"] = np.asarray([
            msg.header.stamp.sec, msg.header.stamp.nanosec, msg.angle_min,
            msg.angle_increment, msg.range_min, msg.range_max,
        ])
        arrays[f"frame_{index}"] = np.asarray(msg.header.frame_id)
    with path.open("xb") as stream:
        np.savez_compressed(stream, **arrays)


def match_with_base_guess(
    node: Any, reference: np.ndarray, target: np.ndarray,
    base_delta: tuple[float, float, float],
) -> ScanMatchResult:
    """ICP operates in laser axes, not the odometry/base coordinate frame."""
    return node.matcher.match(
        reference, target, initial_guess=node.extrinsic.base_to_lidar(*base_delta)
    )


def run_single_oneway_test(
    node: OneWayEvaluatorNode,
    pattern: str,
    speed: float,
    displacement: float,
    plot_prefix: Path,
) -> dict[str, Any] | None:
    """Execute one-way outbound and return inbound test, evaluating ICP for each leg."""
    is_rotation = pattern == "rotation"
    (out_vx, out_vy, out_wz), (in_vx, in_vy, in_wz) = motion_vectors(pattern, speed)
    # Wait for odom
    wait_deadline = time.monotonic() + 5.0
    while node.current_pose is None and time.monotonic() < wait_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.current_pose is None:
        node.get_logger().error("No odom received before test")
        return None

    # Step 1: Baseline Scan at Origin (S0)
    node.current_stage = "start_scan"
    node.get_logger().info("Capturing Start Scan (S0) at Origin...")
    s0_pts = node.capture_scan_burst(duration_s=1.2, min_scans=6)
    if s0_pts is None:
        node.get_logger().error("Failed to capture initial scan S0")
        return None
    save_scan_capture(node, s0_pts, plot_prefix.parent / f"{plot_prefix.name}_start.npz")
    p0_odom = node.current_pose
    assert p0_odom is not None

    # Step 2: Outbound Leg (편도 가는 길)
    node.current_stage = "outbound_motion"
    node.get_logger().info(f"--> Executing Outbound Leg [{pattern}]: speed={speed:.2f}, target={displacement:.3f}")
    out_ok, out_stop_stats = node.execute_leg(out_vx, out_vy, out_wz, displacement, is_rotation, dwell_after_s=1.5)
    if not out_ok:
        node.stop_robot()
        return None

    # Step 3: Destination Scan (S1)
    node.current_stage = "destination_scan"
    node.get_logger().info("Capturing Mid Scan (S1) at Destination...")
    s1_pts = node.capture_scan_burst(duration_s=1.2, min_scans=6)
    p1_odom = node.current_pose
    if s1_pts is None or p1_odom is None:
        node.stop_robot()
        return None
    save_scan_capture(node, s1_pts, plot_prefix.parent / f"{plot_prefix.name}_destination.npz")

    # Compute Outbound Odometry & ICP Ground Truth
    odom_out_dx, odom_out_dy, odom_out_dyaw = compute_relative_odom(p0_odom, p1_odom)
    node.get_logger().info(f"Outbound Odom Delta: dx={odom_out_dx*1000:+.1f}mm, dy={odom_out_dy*1000:+.1f}mm, dyaw={math.degrees(odom_out_dyaw):+.2f}°")

    # Match S1 to S0 with initial guess from odometry
    node.current_stage = "outbound_icp"
    out_icp = match_with_base_guess(
        node, s0_pts, s1_pts, (odom_out_dx, odom_out_dy, odom_out_dyaw)
    )
    if getattr(node, "plot_enabled", True):
        plot_scan_match(
            s0_pts, s1_pts, out_icp,
            str(plot_prefix.parent / f"{plot_prefix.name}_outbound.png"),
            title=f"One-Way Outbound [{pattern.upper()} {speed:.2f}]",
        )

    # Step 4: Inbound Leg (원점 복귀 편도)
    node.current_stage = "inbound_motion"
    node.get_logger().info(f"--> Executing Inbound Leg (Return): speed={speed:.2f}, target={displacement:.3f}")
    in_ok, in_stop_stats = node.execute_leg(in_vx, in_vy, in_wz, displacement, is_rotation, dwell_after_s=1.5)
    if not in_ok:
        node.stop_robot()
        return None
    node.stop_robot()

    # Step 5: Final Returned Scan (S2)
    node.current_stage = "returned_scan"
    node.get_logger().info("Capturing Final Scan (S2) after Return...")
    s2_pts = node.capture_scan_burst(duration_s=1.2, min_scans=6)
    p2_odom = node.current_pose
    if s2_pts is None or p2_odom is None:
        return None
    save_scan_capture(node, s2_pts, plot_prefix.parent / f"{plot_prefix.name}_returned.npz")

    # Compute Inbound Odometry & ICP Ground Truth
    node.current_stage = "inbound_and_loop_icp"
    odom_in_dx, odom_in_dy, odom_in_dyaw = compute_relative_odom(p1_odom, p2_odom)
    in_icp = match_with_base_guess(
        node, s1_pts, s2_pts, (odom_in_dx, odom_in_dy, odom_in_dyaw)
    )
    if getattr(node, "plot_enabled", True):
        plot_scan_match(
            s1_pts, s2_pts, in_icp,
            str(plot_prefix.parent / f"{plot_prefix.name}_inbound.png"),
            title=f"One-Way Inbound Return [{pattern.upper()} {speed:.2f}]",
        )

    # Full Round-Trip Loop Closure (S0 vs S2)
    odom_loop_dx, odom_loop_dy, odom_loop_dyaw = compute_relative_odom(p0_odom, p2_odom)
    loop_icp = match_with_base_guess(
        node, s0_pts, s2_pts, (odom_loop_dx, odom_loop_dy, odom_loop_dyaw)
    )

    transform = node.extrinsic.lidar_to_base
    outbound = build_leg_result(
        (odom_out_dx, odom_out_dy, odom_out_dyaw), out_icp,
        displacement, is_rotation, transform,
    )
    inbound = build_leg_result(
        (odom_in_dx, odom_in_dy, odom_in_dyaw), in_icp,
        displacement, is_rotation, transform,
    )
    loop = None
    if loop_icp.is_valid:
        dx, dy, yaw = transform(
            loop_icp.delta_x_m, loop_icp.delta_y_m, loop_icp.delta_yaw_rad
        )
        loop = {
            "error_x_mm": dx * 1000, "error_y_mm": dy * 1000,
            "error_yaw_deg": math.degrees(yaw),
            "residual_norm_mm": math.hypot(dx, dy) * 1000,
        }
    raw_path = plot_prefix.parent / f"{plot_prefix.name}_scans.npz"
    with raw_path.open("xb") as stream:
        np.savez_compressed(stream, start=s0_pts, destination=s1_pts, returned=s2_pts)
    return {
        "is_valid": outbound["is_valid"] and inbound["is_valid"] and loop_icp.is_valid,
        "pattern": pattern, "speed": speed,
        "nominal_displacement": displacement, "is_rotation": is_rotation,
        "control_stopping_performance": {
            "outbound": out_stop_stats, "inbound": in_stop_stats,
        },
        "extrinsic_calibration_applied": node.extrinsic.to_dict(),
        "evaluation_settings": node.evaluation_settings,
        "raw_scan_file": str(raw_path),
        "outbound": outbound, "inbound": inbound,
        "loop_closure": loop,
        "raw_loop_icp": match_metadata(loop_icp),
    }


def generate_oneway_comparison_plots(
    results_by_pattern: dict[str, list[dict[str, Any]]],
    save_path: Path,
) -> None:
    """Plot axial, optional diagonal and rotation odom-versus-ICP differences."""
    if not HAVE_MATPLOTLIB:
        return

    has_diagonals = any(p in results_by_pattern for p in ("diagonal_left", "diagonal_right"))
    fig, axes = plt.subplots(3 if has_diagonals else 2, 2,
                             figsize=(14, 15 if has_diagonals else 10), dpi=150)

    # Subplot 1: Surge One-Way Error vs Speed
    ax_surge = axes[0, 0]
    surge_runs = [r for r in results_by_pattern.get("surge", []) if r.get("is_valid", False)]
    if surge_runs:
        speeds = [r["speed"] for r in surge_runs]
        out_norms = [r["outbound"]["one_way_error"]["error_norm_mm"] for r in surge_runs]
        in_norms = [r["inbound"]["one_way_error"]["error_norm_mm"] for r in surge_runs]
        out_xs = [r["outbound"]["one_way_error"]["error_x_mm"] for r in surge_runs]
        ax_surge.plot(speeds, out_norms, "o-", color="#1f77b4", linewidth=2, label="Outbound Forward Norm (mm)")
        ax_surge.plot(speeds, in_norms, "s--", color="#ff7f0e", linewidth=1.8, label="Inbound Return Norm (mm)")
        ax_surge.plot(speeds, out_xs, "^:", color="#2ca02c", linewidth=1.5, label="Outbound ΔX Slip (mm)")
        ax_surge.set_title("SURGE (One-Way 0.5m) Error vs Speed", fontsize=11, fontweight="bold")
        ax_surge.set_xlabel("Linear Speed (m/s)")
        ax_surge.set_ylabel("Error (mm)")
        ax_surge.grid(True, linestyle=":", alpha=0.6)
        ax_surge.legend(fontsize=8)

    # Subplot 2: Sway One-Way Error vs Speed
    ax_sway = axes[0, 1]
    sway_runs = [r for r in results_by_pattern.get("sway", []) if r.get("is_valid", False)]
    if sway_runs:
        speeds = [r["speed"] for r in sway_runs]
        out_norms = [r["outbound"]["one_way_error"]["error_norm_mm"] for r in sway_runs]
        in_norms = [r["inbound"]["one_way_error"]["error_norm_mm"] for r in sway_runs]
        out_ys = [r["outbound"]["one_way_error"]["error_y_mm"] for r in sway_runs]
        ax_sway.plot(speeds, out_norms, "o-", color="#d62728", linewidth=2, label="Outbound Left Norm (mm)")
        ax_sway.plot(speeds, in_norms, "s--", color="#9467bd", linewidth=1.8, label="Inbound Right Norm (mm)")
        ax_sway.plot(speeds, out_ys, "v:", color="#8c564b", linewidth=1.5, label="Outbound dY Slip (mm)")
        ax_sway.set_title("SWAY (One-Way 0.5m) Error vs Speed", fontsize=11, fontweight="bold")
        ax_sway.set_xlabel("Linear Speed (m/s)")
        ax_sway.set_ylabel("Error (mm)")
        ax_sway.grid(True, linestyle=":", alpha=0.6)
        ax_sway.legend(fontsize=8)

    # Subplot 3: Rotation One-Way Error vs Angular Speed
    ax_rot = axes[1, 0]
    rot_runs = [r for r in results_by_pattern.get("rotation", []) if r.get("is_valid", False)]
    if rot_runs:
        speeds = [r["speed"] for r in rot_runs]
        out_yaws = [abs(r["outbound"]["one_way_error"]["error_yaw_deg"]) for r in rot_runs]
        in_yaws = [abs(r["inbound"]["one_way_error"]["error_yaw_deg"]) for r in rot_runs]
        ax_rot.plot(speeds, out_yaws, "o-", color="#e377c2", linewidth=2, label="Outbound CCW Yaw Error (deg)")
        ax_rot.plot(speeds, in_yaws, "s--", color="#7f7f7f", linewidth=1.8, label="Inbound CW Yaw Error (deg)")
        ax_rot.set_title("ROTATION (One-Way 90 deg) Error vs Angular Speed", fontsize=11, fontweight="bold")
        ax_rot.set_xlabel("Angular Speed (rad/s)")
        ax_rot.set_ylabel("Heading Error (deg)")
        ax_rot.grid(True, linestyle=":", alpha=0.6)
        ax_rot.legend(fontsize=8)

    # Subplot 4: Relative Error Rate (%) Comparison
    ax_bar = axes[1, 1]
    categories = []
    rates = []
    colors_list = []

    for r in surge_runs:
        categories.append(f"Surge {r['speed']:.2f}")
        rates.append(r["outbound"]["one_way_error"]["error_rate_pct"])
        colors_list.append("#1f77b4")
    for r in sway_runs:
        categories.append(f"Sway {r['speed']:.2f}")
        rates.append(r["outbound"]["one_way_error"]["error_rate_pct"])
        colors_list.append("#d62728")
    for r in rot_runs:
        categories.append(f"Rot {r['speed']:.2f}")
        rates.append(r["outbound"]["one_way_error"]["error_rate_pct"])
        colors_list.append("#e377c2")

    if has_diagonals:
        for col, pattern in enumerate(("diagonal_left", "diagonal_right")):
            ax = axes[2, col]
            runs = [r for r in results_by_pattern.get(pattern, []) if r.get("is_valid")]
            for leg, style in (("outbound", "o-"), ("inbound", "s--")):
                ax.plot([r["speed"] for r in runs],
                        [r[leg]["one_way_error"]["error_norm_mm"] for r in runs],
                        style, label=leg)
            ax.set_title(f"{pattern.upper()} one-way odom - ICP")
            ax.set_xlabel("Speed cap (m/s)")
            ax.set_ylabel("Translation difference norm (mm)")
            ax.grid(alpha=.3)
            ax.legend()
            for r in runs:
                categories.append(f"{pattern} {r['speed']:.2f}")
                rates.append(r["outbound"]["one_way_error"]["error_rate_pct"])
                colors_list.append("#2ca02c" if col == 0 else "#8c564b")

    if categories:
        x_indices = np.arange(len(categories))
        ax_bar.bar(x_indices, rates, color=colors_list, alpha=0.8, width=0.55)
        ax_bar.set_xticks(x_indices)
        ax_bar.set_xticklabels(categories, rotation=35, ha="right", fontsize=8)
        ax_bar.set_ylabel("Relative One-Way Error (%)")
        ax_bar.set_title("Relative One-Way Error Rate (%) across Modes & Speeds", fontsize=11, fontweight="bold")
        ax_bar.grid(True, axis="y", linestyle=":", alpha=0.6)
        ax_bar.legend(fontsize=8)

    fig.suptitle("One-Way Holonomic Motion: Wheel Odometry vs LiDAR ICP", fontsize=13, fontweight="bold")
    plt.tight_layout()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path)
    plt.close(fig)
    print(f"\n[One-Way Plot] Saved comparison figure to: {save_path}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Evaluate one-way motion using LiDAR ICP")
    parser.add_argument("--patterns", nargs="+", default=["surge", "sway", "rotation"],
                        choices=["surge", "sway", "diagonal_left", "diagonal_right", "rotation"])
    parser.add_argument("--linear-speeds", type=float, nargs="+", default=[0.20, 0.35, 0.50])
    parser.add_argument("--angular-speeds", type=float, nargs="+", default=[0.30, 0.60, 1.00])
    parser.add_argument("--distance", type=float, default=0.50)
    parser.add_argument("--rotation-angle-deg", type=float, default=90.0)
    parser.add_argument("--odom-timeout", type=float, default=0.15)
    parser.add_argument("--motion-config", type=Path)
    parser.add_argument("--extrinsic-config", type=Path,
                        help="Flat JSON: yaw_deg, lever_arm_x_m, lever_arm_y_m")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--lidar-min-range", type=float, default=0.38,
                        help="ICP-only minimum range in metres; excludes near body returns")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/odometry_evaluation"))
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = MotionConfig.from_json(args.motion_config)
        extrinsic = PlanarExtrinsic.from_json(args.extrinsic_config)
        OdomFreshnessGuard(timeout_s=args.odom_timeout)
        if not math.isfinite(args.lidar_min_range) or not 0 <= args.lidar_min_range < 10:
            raise ValueError("lidar minimum range must be finite and in [0, 10) metres")
        if not args.odom_topic or any(c.isspace() for c in args.odom_topic):
            raise ValueError("odom topic must be nonempty and contain no whitespace")
        evaluation_settings = {
            "odom_topic": args.odom_topic,
            "lidar_min_range_m": args.lidar_min_range,
            "extrinsic_calibration_applied": extrinsic.to_dict(),
            "extrinsic_config_file": str(args.extrinsic_config) if args.extrinsic_config else None,
            "extrinsic_config_sha256": (
                hashlib.sha256(args.extrinsic_config.read_bytes()).hexdigest()
                if args.extrinsic_config else None
            ),
        }
        if not math.isfinite(args.distance) or not 0 < args.distance <= 3:
            raise ValueError("distance must be in (0, 3] metres")
        if not math.isfinite(args.rotation_angle_deg) or args.rotation_angle_deg <= 0:
            raise ValueError("rotation angle must be finite and positive")
        if any(not math.isfinite(v) or not 0 < v <= 0.5 for v in args.linear_speeds):
            raise ValueError("linear speeds must be in (0, 0.5] m/s")
        if any(not math.isfinite(v) or not 0 < v <= 1 for v in args.angular_speeds):
            raise ValueError("angular speeds must be in (0, 1] rad/s")
    except (ValueError, OSError, TypeError) as error:
        parser.error(str(error))
    if args.dry_run:
        print("=== DRY-RUN: One-Way Motion Evaluation Plan ===")
        print(f"Patterns: {args.patterns}; linear speeds: {args.linear_speeds}; distance: {args.distance}")
        print(f"Angular speeds: {args.angular_speeds}; angle: {args.rotation_angle_deg}")
        print(f"Motion config: {config.to_dict()}")
        print(f"Evaluation settings: {evaluation_settings}")
        return 0
    if not HAVE_ROS:
        print("Run inside distrobox ros2-humble.", file=sys.stderr)
        return 1
    lock_file = open("/tmp/cleany_odom_eval.lock", "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        print("Another calibration motion tool is active.", file=sys.stderr)
        return 1
    rclpy.init()
    node = OneWayEvaluatorNode(
        odom_timeout_s=args.odom_timeout, motion_config=config,
        extrinsic=extrinsic, odom_topic=args.odom_topic,
        min_range_m=args.lidar_min_range,
    )
    node.plot_enabled = not args.no_plots
    node.evaluation_settings = evaluation_settings
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results_by_pattern: dict[str, list[dict[str, Any]]] = {}
    failure = ""
    exit_code = 0

    def interrupt(sig, frame):
        node.aborted = True
        node.last_failure = f"Interrupted by signal {sig}"
        raise KeyboardInterrupt()

    old_handlers = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for pattern in args.patterns:
            results_by_pattern[pattern] = []
            speeds = args.angular_speeds if pattern == "rotation" else args.linear_speeds
            displacement = math.radians(args.rotation_angle_deg) if pattern == "rotation" else args.distance
            for speed in speeds:
                print(f"Starting [{pattern}] speed={speed:.2f}, target={displacement:.3f}", flush=True)
                prefix = args.output_dir / f"oneway_{pattern}_v{int(speed*100):02d}_{timestamp}"
                result = run_single_oneway_test(node, pattern, speed, displacement, prefix)
                if result is None:
                    failure = node.last_failure or "Motion or scan capture failed"
                    exit_code = 1
                    result = {
                        "is_valid": False, "pattern": pattern, "speed": speed,
                        "nominal_displacement": displacement,
                        "failure_reason": failure,
                        "failed_stage": getattr(node, "current_stage", "initial_odometry"),
                        "evaluation_settings": evaluation_settings,
                        "capture_files": [str(p) for p in prefix.parent.glob(f"{prefix.name}_*.npz")],
                    }
                    results_by_pattern[pattern].append(result)
                    write_data_json(prefix.with_suffix(".json"), result)
                    break
                results_by_pattern[pattern].append(result)
                write_data_json(prefix.with_suffix(".json"), result)
                if not result["is_valid"]:
                    failure = "Rejected ICP observation; excluded from accuracy statistics"
                    exit_code = 1
                    break
                print(f"[{pattern} {speed:.2f}] outbound={result['outbound']['one_way_error']['error_norm_mm']:.2f}mm "
                      f"inbound={result['inbound']['one_way_error']['error_norm_mm']:.2f}mm", flush=True)
            if exit_code:
                break
    except (KeyboardInterrupt, Exception) as error:
        failure = node.last_failure or str(error) or "Interrupted"
        exit_code = 1
    finally:
        try:
            node.stop_robot()
        except Exception as error:
            exit_code = 1
            failure = f"Stop failed: {error}"
        if node.aborted:
            exit_code = 1
            failure = node.last_failure or failure
        summary = {
            "schema_version": 2, "timestamp": timestamp,
            "status": "success" if not exit_code else "failed",
            "failure_reason": failure, "patterns_evaluated": args.patterns,
            "linear_speeds": args.linear_speeds, "angular_speeds": args.angular_speeds,
            "translation_distance_m": args.distance, "rotation_angle_deg": args.rotation_angle_deg,
            "motion_config": config.to_dict(), "odom_timeout_s": args.odom_timeout,
            "evaluation_settings": evaluation_settings,
            "results": results_by_pattern, "motion_runs": node.motion_results,
        }
        summary_path = args.output_dir / f"oneway_evaluation_summary_{timestamp}.json"
        try:
            write_data_json(summary_path, summary)
            print(f"Data: {summary_path}", flush=True)
        except (OSError, TypeError, ValueError) as error:
            exit_code = 1
            print(f"Data save failed: {error}", file=sys.stderr)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            lock_file.close()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
    if not args.no_plots:
        generate_oneway_comparison_plots(
            results_by_pattern, args.output_dir / f"oneway_evaluation_comparison_{timestamp}.png"
        )
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
