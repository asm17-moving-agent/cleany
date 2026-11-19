#!/usr/bin/env python3
"""Yaw P-gain Response Calibration Tool for Cleany Mobile Base.

Executes step heading changes for multiple Kp candidates, logs high-frequency
time-series response (heading, velocity, tracking error), evaluates settling time
and overshoot, and generates publication-grade response plots.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MATPLOTLIB = True
except ImportError:
    HAVE_MATPLOTLIB = False

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_srvs.srv import SetBool
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

from evaluator_core import OdomFreshnessGuard, OdomGuardError
from motion_runtime import (
    init_motion_runtime, receive_odometry, stop_motion, await_fresh_pose, request_abort,
)


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class ResponseSample:
    t_sec: float
    current_yaw_deg: float
    target_yaw_deg: float
    error_yaw_deg: float
    cmd_wz_rad_s: float
    odom_wz_rad_s: float


@dataclass
class GainMetrics:
    kp: float
    target_deg: float
    max_wz_rad_s: float
    rise_time_s: float
    peak_yaw_deg: float
    overshoot_deg: float
    overshoot_pct: float
    settling_time_s: float  # time to enter and stay within tolerance
    steady_state_error_deg: float
    samples: list[ResponseSample]


class YawCalibrationNode(Node if HAVE_ROS else object):  # type: ignore[misc]

    def __init__(self, odom_timeout_s: float = 0.15) -> None:
        if not HAVE_ROS:
            raise RuntimeError("ROS 2 is not available")
        super().__init__("yaw_p_tuner")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.enable_client = self.create_client(SetBool, "/base/enable")
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 1)
        self.guard = OdomFreshnessGuard(timeout_s=odom_timeout_s)
        self.current_yaw_rad: float | None = None
        self.current_wz_rad_s: float = 0.0
        self.aborted = False
        self.current_pose = None
        init_motion_runtime(self)

    def _on_odom(self, msg: Odometry) -> None:
        receive_odometry(self, msg)
        if self.current_pose is not None:
            self.current_yaw_rad = self.current_pose.yaw_rad
            self.current_wz_rad_s = self.motion_velocity[1]

    def set_base_enabled(self, enable: bool) -> bool:
        if not self.enable_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Service /base/enable not available")
            return False
        req = SetBool.Request()
        req.data = enable
        future = self.enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            return future.result().success
        return False

    def stop_robot(self) -> None:
        stop_motion(self)

    def step_turn(
        self,
        target_displacement_deg: float,
        kp: float,
        max_wz: float = 0.5,
        min_wz: float = 0.08,
        tolerance_deg: float = 1.0,
        settle_hold_s: float = 0.4,
        timeout_s: float = 10.0,
    ) -> list[ResponseSample]:
        rate_hz = 50.0
        dt = 1.0 / rate_hz

        try:
            await_fresh_pose(self, timeout_s=5.0)
            if not self.set_base_enabled(True):
                raise OdomGuardError("Failed to enable base")
            self.guard.check_freshness()
        except OdomGuardError as error:
            request_abort(self, str(error))
            self.stop_robot()
            return []

        start_time = time.monotonic()
        last_yaw_rad = self.current_yaw_rad
        accumulated_yaw_rad = 0.0
        target_rad = math.radians(target_displacement_deg)
        target_deg = target_displacement_deg

        samples: list[ResponseSample] = []
        settled_start_time: float | None = None

        twist = Twist()

        while rclpy.ok() and not self.aborted:
            rclpy.spin_once(self, timeout_sec=0.005)
            now = time.monotonic()
            elapsed = now - start_time

            if elapsed > timeout_s:
                self.get_logger().warn(f"Step turn timed out after {timeout_s:.1f}s")
                request_abort(self, "Yaw response timed out before settling")
                break

            try:
                self.guard.check_freshness()
            except OdomGuardError as e:
                self.get_logger().error(f"[ODOM_FRESHNESS_FAIL] Stale odom in step turn: {e}")
                self.aborted = True
                self.stop_robot()
                break

            curr_yaw = self.current_yaw_rad
            if curr_yaw is not None:
                diff = (curr_yaw - last_yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
                accumulated_yaw_rad += diff
                last_yaw_rad = curr_yaw

            curr_deg = math.degrees(accumulated_yaw_rad)
            error_deg = target_deg - curr_deg
            error_rad = target_rad - accumulated_yaw_rad

            # Log sample
            samples.append(
                ResponseSample(
                    t_sec=elapsed,
                    current_yaw_deg=curr_deg,
                    target_yaw_deg=target_deg,
                    error_yaw_deg=error_deg,
                    cmd_wz_rad_s=twist.angular.z,
                    odom_wz_rad_s=self.current_wz_rad_s,
                )
            )

            # Settling condition: must stay within tolerance band for settle_hold_s
            if abs(error_deg) <= tolerance_deg and abs(self.current_wz_rad_s) <= self.motion_config.stop_angular_speed_rad_s:
                if settled_start_time is None:
                    settled_start_time = now
                elif now - settled_start_time >= settle_hold_s:
                    self.get_logger().info(
                        f"Settled at {curr_deg:.2f}deg (err={error_deg:.2f}deg) after {elapsed:.2f}s"
                    )
                    break
            else:
                settled_start_time = None

            # P controller
            target_wz = kp * error_rad
            if abs(target_wz) > max_wz:
                target_wz = math.copysign(max_wz, target_wz)
            elif abs(target_wz) < min_wz and abs(error_deg) > tolerance_deg:
                target_wz = math.copysign(min_wz, target_wz)
            elif abs(error_deg) <= tolerance_deg:
                target_wz = 0.0

            twist.angular.z = target_wz
            self.cmd_vel_pub.publish(twist)
            time.sleep(dt)

        # Stop and dwell
        zero_twist = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(zero_twist)
            time.sleep(0.02)

        self.stop_robot()
        return samples if not self.aborted else []


def calculate_metrics(samples: list[ResponseSample], kp: float, target_deg: float, max_wz: float) -> GainMetrics:
    if not samples:
        return GainMetrics(kp, target_deg, max_wz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [])

    yaws = [s.current_yaw_deg for s in samples]
    times = [s.t_sec for s in samples]
    is_positive = target_deg >= 0

    # Peak & Overshoot
    if is_positive:
        peak_yaw = max(yaws)
        overshoot_deg = max(0.0, peak_yaw - target_deg)
    else:
        peak_yaw = min(yaws)
        overshoot_deg = max(0.0, target_deg - peak_yaw)
    overshoot_pct = (overshoot_deg / abs(target_deg)) * 100.0 if target_deg != 0 else 0.0

    # Rise time (10% to 90%)
    t_90_target = abs(target_deg) * 0.90
    rise_time = 0.0
    for s in samples:
        if abs(s.current_yaw_deg) >= t_90_target:
            rise_time = s.t_sec
            break

    # Settling time (stays permanently within +-1.0 deg of target)
    tol = 1.0
    settling_time = times[-1]
    for i in range(len(samples) - 1, -1, -1):
        if abs(samples[i].current_yaw_deg - target_deg) > tol:
            settling_time = samples[min(i + 1, len(samples) - 1)].t_sec
            break

    # Steady state error (average of last 5 samples)
    tail = samples[-5:] if len(samples) >= 5 else samples
    steady_state_error = sum(s.error_yaw_deg for s in tail) / len(tail)

    return GainMetrics(
        kp=kp,
        target_deg=target_deg,
        max_wz_rad_s=max_wz,
        rise_time_s=rise_time,
        peak_yaw_deg=peak_yaw,
        overshoot_deg=overshoot_deg,
        overshoot_pct=overshoot_pct,
        settling_time_s=settling_time,
        steady_state_error_deg=steady_state_error,
        samples=samples,
    )


def plot_gain_comparison(metrics_list: list[GainMetrics], output_path: Path) -> None:
    if not HAVE_MATPLOTLIB:
        return

    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(10, 8), sharex=True, dpi=150)

    target_deg = metrics_list[0].target_deg
    colors = ["#1f77b4", "#2ca02c", "#ff7f0e", "#d62728", "#9467bd"]

    # Target line & tolerance band
    ax_pos.axhline(target_deg, color="black", linestyle="--", linewidth=1.5, label=f"Target ({target_deg:.1f}°)")
    ax_pos.axhspan(target_deg - 1.0, target_deg + 1.0, color="gray", alpha=0.15, label="±1.0° Tolerance")

    for idx, m in enumerate(metrics_list):
        c = colors[idx % len(colors)]
        times = [s.t_sec for s in m.samples]
        yaws = [s.current_yaw_deg for s in m.samples]
        label = f"Kp={m.kp:.1f} (Settling={m.settling_time_s:.2f}s, Over={m.overshoot_deg:.2f}°)"
        ax_pos.plot(times, yaws, label=label, color=c, linewidth=2.0)

        cmd_wz = [s.cmd_wz_rad_s for s in m.samples]
        odom_wz = [s.odom_wz_rad_s for s in m.samples]
        ax_vel.plot(times, cmd_wz, linestyle="--", color=c, alpha=0.6, label=f"Kp={m.kp:.1f} Cmd")
        ax_vel.plot(times, odom_wz, linestyle="-", color=c, linewidth=1.5, label=f"Kp={m.kp:.1f} Odom")

    ax_pos.set_ylabel("Heading Angle (deg)", fontsize=11, fontweight="bold")
    ax_pos.set_title(f"Step Response Comparison (Target = {target_deg:.1f}°)", fontsize=13, fontweight="bold")
    ax_pos.legend(loc="lower right", fontsize=9)
    ax_pos.grid(True, linestyle=":", alpha=0.6)

    ax_vel.set_xlabel("Time (seconds)", fontsize=11, fontweight="bold")
    ax_vel.set_ylabel("Angular Velocity (rad/s)", fontsize=11, fontweight="bold")
    ax_vel.set_title("Angular Velocity Profile (Command vs Measured)", fontsize=11, fontweight="bold")
    ax_vel.legend(loc="upper right", fontsize=8, ncol=len(metrics_list))
    ax_vel.grid(True, linestyle=":", alpha=0.6)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path)
    plt.close()


def simulate_synthetic_response(kp: float, target_deg: float, max_wz: float) -> list[ResponseSample]:
    """Simulates realistic 2nd-order mobile base response for dry-run/testing."""
    dt = 0.02
    target_rad = math.radians(target_deg)
    yaw_rad = 0.0
    actual_wz = 0.0
    cmd_wz = 0.0
    slew_rate = 0.50 * 5.0  # max wz slew
    tau = 0.12  # motor response time constant

    samples: list[ResponseSample] = []
    settled_count = 0

    for step in range(250):  # 5 seconds
        t = step * dt
        error_rad = target_rad - yaw_rad
        error_deg = math.degrees(error_rad)
        curr_deg = math.degrees(yaw_rad)

        samples.append(
            ResponseSample(
                t_sec=t,
                current_yaw_deg=curr_deg,
                target_yaw_deg=target_deg,
                error_yaw_deg=error_deg,
                cmd_wz_rad_s=cmd_wz,
                odom_wz_rad_s=actual_wz,
            )
        )

        if abs(error_deg) <= 1.0:
            settled_count += 1
            if settled_count >= 25:
                break
        else:
            settled_count = 0

        desired_wz = kp * error_rad
        if abs(desired_wz) > max_wz:
            desired_wz = math.copysign(max_wz, desired_wz)
        elif abs(desired_wz) < 0.08 and abs(error_deg) > 1.0:
            desired_wz = math.copysign(0.08, desired_wz)
        elif abs(error_deg) <= 1.0:
            desired_wz = 0.0

        # Slew limit
        wz_diff = desired_wz - cmd_wz
        max_step = slew_rate * dt
        if abs(wz_diff) > max_step:
            cmd_wz += math.copysign(max_step, wz_diff)
        else:
            cmd_wz = desired_wz

        # Motor 1st-order response
        actual_wz += (cmd_wz - actual_wz) * (dt / (tau + dt))
        yaw_rad += actual_wz * dt

    return samples


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Calibrate Heading Yaw P-gain via Step Response Graphs")
    parser.add_argument("--kp-list", type=float, nargs="+", default=[1.0, 2.0, 3.0, 4.0],
                        help="List of Kp gains to evaluate (default: 1.0 2.0 3.0 4.0)")
    parser.add_argument("--angle-deg", type=float, default=90.0,
                        help="Step turn angle in degrees (default: 90.0)")
    parser.add_argument("--max-wz", type=float, default=0.5,
                        help="Maximum angular speed rad/s (default: 0.5)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run synthetic simulation without connecting to real ROS")
    parser.add_argument("--odom-timeout", type=float, default=0.15,
                        help="Odometry watchdog freshness timeout in seconds (default: 0.15s)")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/odometry_evaluation"),
                        help="Output directory for results and plots")
    args = parser.parse_args(argv)

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    plot_file = output_dir / f"yaw_p_response_{timestamp}.png"
    json_file = output_dir / f"yaw_p_calibration_{timestamp}.json"

    metrics_list: list[GainMetrics] = []

    if args.dry_run or not HAVE_ROS:
        print(f"=== DRY-RUN / SIMULATION MODE (Testing Kp = {args.kp_list}) ===")
        for kp in args.kp_list:
            samples = simulate_synthetic_response(kp, args.angle_deg, args.max_wz)
            m = calculate_metrics(samples, kp, args.angle_deg, args.max_wz)
            metrics_list.append(m)
    else:
        import fcntl
        lock_file = open("/tmp/cleany_odom_eval.lock", "w")
        try:
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock_file.close()
            print("Another calibration motion tool is active.", file=sys.stderr)
            return 1
        rclpy.init()
        node = YawCalibrationNode(odom_timeout_s=args.odom_timeout)
        print(f"=== PHYSICAL ROBOT STEP RESPONSE CALIBRATION ===")
        print(f"Angle: {args.angle_deg}° | Max speed: {args.max_wz} rad/s | Gains: {args.kp_list} | Odom Timeout: {args.odom_timeout:.3f}s")

        try:
            for kp in args.kp_list:
                if node.aborted:
                    print("[ABORT] Guard tripped, skipping remaining gain tests.", file=sys.stderr)
                    break
                print(f"\n[Test] Evaluating Kp = {kp:.1f} (Step +{args.angle_deg:.1f}°)...")
                samples = node.step_turn(args.angle_deg, kp, max_wz=args.max_wz)
                if node.aborted:
                    print("[ABORT] Step turn aborted due to guard trip.", file=sys.stderr)
                    break
                m = calculate_metrics(samples, kp, args.angle_deg, args.max_wz)
                metrics_list.append(m)
                print(f"  -> Rise time: {m.rise_time_s:.2f}s, Settling: {m.settling_time_s:.2f}s, Overshoot: {m.overshoot_deg:.2f}°")

                # Return turn back to 0° to maintain workspace
                print(f"  -> Returning to initial heading...")
                node.step_turn(-args.angle_deg, kp=2.0, max_wz=args.max_wz)
                if node.aborted:
                    print("[ABORT] Return turn aborted due to guard trip.", file=sys.stderr)
                    break
                time.sleep(1.0)
        finally:
            node.stop_robot()
            node.destroy_node()
            rclpy.shutdown()
            lock_file.close()

    if not metrics_list:
        print("No valid completed yaw response; no gain recommendation.", file=sys.stderr)
        return 1

    # Generate plot
    plot_gain_comparison(metrics_list, plot_file)
    print(f"\nSaved response plot to: {plot_file}")

    # Summary table
    print("\n" + "=" * 78)
    print(f"{'Kp':<6} | {'Rise Time (s)':<14} | {'Settling Time (s)':<18} | {'Overshoot (deg)':<16} | {'SS Error (deg)':<14}")
    print("-" * 78)
    best_m = metrics_list[0]
    best_score = float("inf")

    for m in metrics_list:
        # Score = settling_time + 3.0 * overshoot
        score = m.settling_time_s + 3.0 * m.overshoot_deg
        if score < best_score:
            best_score = score
            best_m = m
        print(f"{m.kp:<6.1f} | {m.rise_time_s:<14.2f} | {m.settling_time_s:<18.2f} | {m.overshoot_deg:<16.2f} | {m.steady_state_error_deg:<14.2f}")
    print("=" * 78)
    print(f"Optimal Gain Recommendation: Kp = {best_m.kp:.1f} (Settling: {best_m.settling_time_s:.2f}s, Overshoot: {best_m.overshoot_deg:.2f}°)")

    # Save JSON
    json_data = {
        "target_deg": args.angle_deg,
        "max_wz_rad_s": args.max_wz,
        "recommended_kp": best_m.kp,
        "results": [
            {
                "kp": m.kp,
                "rise_time_s": m.rise_time_s,
                "peak_yaw_deg": m.peak_yaw_deg,
                "overshoot_deg": m.overshoot_deg,
                "overshoot_pct": m.overshoot_pct,
                "settling_time_s": m.settling_time_s,
                "steady_state_error_deg": m.steady_state_error_deg,
            }
            for m in metrics_list
        ],
    }
    with open(json_file, "w") as f:
        json.dump(json_data, f, indent=2)
    print(f"Saved calibration data to: {json_file}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
