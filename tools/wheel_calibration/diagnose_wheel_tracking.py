#!/usr/bin/env python3
"""Diagnose per-wheel velocity tracking and estimate ground-loaded PI+FF parameters.

Supports round-trip 0.5m surge/sway tests, plots 4-wheel tracking responses,
and computes optimal FF scales and PI gains to eliminate wheel speed asymmetry.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import hashlib
import signal
import sys
import time
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
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_srvs.srv import SetBool
    from cleany_base_interfaces.msg import WheelState
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

REPO_ROOT = Path(__file__).resolve().parents[2]
ODOM_EVAL_DIR = REPO_ROOT / "tools" / "odometry_evaluation"
if str(ODOM_EVAL_DIR) not in sys.path:
    sys.path.insert(0, str(ODOM_EVAL_DIR))

try:
    from evaluator_core import OdomFreshnessGuard, OdomGuardError
except ImportError:
    OdomFreshnessGuard = None  # type: ignore[misc,assignment]
    OdomGuardError = RuntimeError  # type: ignore[misc,assignment]

from motion_runtime import execute_motion, init_motion_runtime, receive_odometry, stop_motion
from evaluation_results import json_finite

def quaternion_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


WHEEL_NAMES = ("FL", "FR", "RL", "RR")
MIN_STABLE_TARGET_DURATION_S = 0.3
TARGET_ACTIVE_THRESHOLD_RAD_S = 0.5
COMMAND_SETTLED_TOLERANCE_RAD_S = 0.05
MAX_SAMPLE_GAP_S = 0.1

DEFAULT_BASELINE = Path(__file__).with_name("controller_baseline.json")

def load_baseline(path: Path = DEFAULT_BASELINE) -> dict[str, dict[str, float]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    wheels = payload["wheels"]
    if set(wheels) != set(WHEEL_NAMES):
        raise ValueError("Baseline must define exactly FL, FR, RL, RR")
    configs = {name: {key: float(wheels[name][key]) for key in ("ff_scale", "kp", "ki")}
               for name in WHEEL_NAMES}
    if any(not math.isfinite(value) or value <= 0
           for config in configs.values() for value in config.values()):
        raise ValueError("Controller baseline values must be finite and positive")
    return configs

CURRENT_CONFIGS = load_baseline()

def split_direction_samples(samples: list[WheelSample]) -> dict[str, list[WheelSample]]:
    active = [s for s in samples if any(abs(v) > TARGET_ACTIVE_THRESHOLD_RAD_S
                                        for v in s.target_rad_s)]
    if not active:
        return {"forward": [], "reverse": []}
    reference = next((s.target_rad_s for s in active
                      if math.sqrt(sum(v * v for v in s.target_rad_s)) > 0.0), None)
    if reference is None:
        return {"forward": [], "reverse": []}
    result = {"forward": [], "reverse": []}
    for sample in samples:
        projection = sum(a * b for a, b in zip(sample.target_rad_s, reference))
        if any(abs(v) > TARGET_ACTIVE_THRESHOLD_RAD_S for v in sample.target_rad_s):
            result["forward" if projection >= 0 else "reverse"].append(sample)
    return result

def has_complete_directional_results(
    stats_by_pattern: dict[str, dict[str, list[WheelStats]]],
) -> bool:
    return all(
        len(stats_by_pattern.get(pattern, {}).get(direction, [])) == len(WHEEL_NAMES)
        and all(
            item.has_sufficient_steady_state
            and item.suggested_kp is not None
            and item.suggested_ki is not None
            and item.ground_ff_scale_est is not None
            for item in stats_by_pattern[pattern][direction]
        )
        for pattern in ("surge", "sway")
        for direction in ("forward", "reverse")
    )


@dataclass
class WheelSample:
    time_s: float
    target_rad_s: list[float]
    commanded_rad_s: list[float]
    velocity_rad_s: list[float]
    pwm_percent: list[int]


@dataclass
class WheelStats:
    name: str
    target_mean: float
    commanded_mean: float
    actual_mean: float
    steady_error_mean: float
    steady_error_rmse: float
    steady_rel_err_pct: float
    transient_delay_ms: float
    pwm_mean: float
    current_ff_scale: float
    ground_ff_scale_est: float | None
    suggested_kp: float | None
    suggested_ki: float | None
    has_sufficient_steady_state: bool = True
    status: str = "VALID"


def analyze_samples(
    samples: list[WheelSample],
    configs: dict[str, dict[str, float]] | None = None,
    target_sign: int | None = None,
) -> list[WheelStats]:
    if not samples:
        return []

    results = []
    for idx, name in enumerate(WHEEL_NAMES):
        current_cfg = (configs or CURRENT_CONFIGS)[name]

        # Steady state condition:
        # 1. Target speed is active (|target| > 0.5 rad/s)
        # 2. Commanded speed has reached target within tolerance (slew phase finished): |commanded - target| < 0.05 rad/s
        ordered = sorted(samples, key=lambda sample: sample.time_s)
        steady_samples: list[WheelSample] = []
        transient_duration_s = 0.0
        run: list[WheelSample] = []

        def finish_run() -> None:
            nonlocal transient_duration_s
            if not run:
                return
            start = run[0].time_s
            settled_segments: list[list[WheelSample]] = []
            segment: list[WheelSample] = []
            first_settled_time = None
            for sample in run:
                settled = abs(sample.commanded_rad_s[idx] - sample.target_rad_s[idx]) < COMMAND_SETTLED_TOLERANCE_RAD_S
                if not settled:
                    if segment:
                        settled_segments.append(segment)
                    segment = []
                    continue
                if first_settled_time is None:
                    first_settled_time = sample.time_s
                if segment and sample.time_s - segment[-1].time_s > MAX_SAMPLE_GAP_S:
                    settled_segments.append(segment)
                    segment = [sample]
                else:
                    segment.append(sample)
            if segment:
                settled_segments.append(segment)
            for segment in settled_segments:
                steady_samples.extend(
                    sample for sample in segment
                    if sample.time_s - start >= MIN_STABLE_TARGET_DURATION_S
                    and sample.time_s - segment[0].time_s >= MIN_STABLE_TARGET_DURATION_S
                )
            transient_duration_s += max(
                0.0, (first_settled_time if first_settled_time is not None else run[-1].time_s) - start
            )
            run.clear()

        for sample in ordered:
            target = sample.target_rad_s[idx]
            active = abs(target) > TARGET_ACTIVE_THRESHOLD_RAD_S
            sign_matches = target_sign is None or (target_sign > 0 and target > 0) or (target_sign < 0 and target < 0)
            same_run = bool(run) and abs(target - run[-1].target_rad_s[idx]) < 1e-6 and sample.time_s - run[-1].time_s <= MAX_SAMPLE_GAP_S
            if not active or not sign_matches or (run and not same_run):
                finish_run()
            if active and sign_matches:
                run.append(sample)
        finish_run()

        if len(steady_samples) < 2:
            results.append(WheelStats(
                name=name,
                target_mean=0.0,
                commanded_mean=0.0,
                actual_mean=0.0,
                steady_error_mean=0.0,
                steady_error_rmse=0.0,
                steady_rel_err_pct=0.0,
                transient_delay_ms=0.0,
                pwm_mean=0.0,
                current_ff_scale=current_cfg["ff_scale"],
                ground_ff_scale_est=None,
                suggested_kp=None,
                suggested_ki=None,
                has_sufficient_steady_state=False,
                status="INSUFFICIENT_STEADY_STATE_DATA",
            ))
            continue

        targets = [abs(s.target_rad_s[idx]) for s in steady_samples]
        cmds = [abs(s.commanded_rad_s[idx]) for s in steady_samples]
        actuals = [abs(s.velocity_rad_s[idx]) for s in steady_samples]
        pwms = [abs(s.pwm_percent[idx]) for s in steady_samples]

        t_mean = sum(targets) / len(targets)
        c_mean = sum(cmds) / len(cmds)
        a_mean = sum(actuals) / len(actuals)
        pwm_mean = sum(pwms) / len(pwms)

        # Evaluate tracking error against slew-filtered commanded_rad_s (not the step target!)
        errors = [abs(s.commanded_rad_s[idx] - s.velocity_rad_s[idx]) for s in steady_samples]
        err_mean = sum(errors) / len(errors)
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
        rel_err_pct = (err_mean / c_mean * 100.0) if c_mean > 0.1 else 0.0

        # Transient response delay estimate (time spent slewing before reaching steady-state)
        transient_delay_ms = transient_duration_s * 1000.0

        # FF estimate based strictly on steady-state PWM
        if pwm_mean > 1.0 and a_mean > 0.1:
            ground_ff = (a_mean / pwm_mean) * 100.0
        else:
            ground_ff = None

        # If steady-state tracking error is negligible (< 2.0% and < 0.10 rad/s), KEEP CURRENT GAINS!
        # Do NOT mistakenly increase gains due to normal transient acceleration lag!
        # Tracking error alone is not enough evidence to retune closed-loop PI gains.
        adj_factor = 1.0

        sug_kp = round(current_cfg["kp"] * adj_factor, 3)
        sug_ki = round(current_cfg["ki"] * adj_factor, 3)

        results.append(WheelStats(
            name=name,
            target_mean=round(t_mean, 3),
            commanded_mean=round(c_mean, 3),
            actual_mean=round(a_mean, 3),
            steady_error_mean=round(err_mean, 3),
            steady_error_rmse=round(rmse, 3),
            steady_rel_err_pct=round(rel_err_pct, 2),
            transient_delay_ms=round(transient_delay_ms, 1),
            pwm_mean=round(pwm_mean, 1),
            current_ff_scale=current_cfg["ff_scale"],
            ground_ff_scale_est=round(ground_ff, 4) if ground_ff is not None else None,
            suggested_kp=sug_kp,
            suggested_ki=sug_ki,
            has_sufficient_steady_state=True,
            status="VALID",
        ))

    return results


def format_table(stats: list[WheelStats], title: str = "STEADY-STATE WHEEL TRACKING & GROUND-LOAD PI+FF DIAGNOSIS") -> str:
    lines = [
        "=" * 105,
        f"                             {title}                             ",
        "=" * 105,
        f"{'Wheel':<6} | {'Cmd(rad/s)':<11} | {'Actual(rad/s)':<13} | {'Steady RMSE':<11} | {'Rel Err%':<8} | {'Slew Lag(ms)':<12} | {'Current FF':<10} | {'Suggested Kp/Ki':<15}",
        "-" * 105,
    ]
    for s in stats:
        if not s.has_sufficient_steady_state:
            lines.append(f"{s.name:<6} | {'INSUFFICIENT STEADY-STATE SAMPLES (NO RECOMMENDATION)':<90}")
        else:
            lines.append(
                f"{s.name:<6} | {s.commanded_mean:<11.3f} | {s.actual_mean:<13.3f} | {s.steady_error_rmse:<11.3f} | {s.steady_rel_err_pct:<7.2f}% | "
                f"{s.transient_delay_ms:<12.1f} | {s.current_ff_scale:<10.4f} | "
                f"Kp={s.suggested_kp if s.suggested_kp is not None else 'N/A'}, "
                f"Ki={s.suggested_ki if s.suggested_ki is not None else 'N/A'}"
            )
    lines.append("-" * 105)

    # Assess asymmetry
    actual_speeds = [s.actual_mean for s in stats if s.actual_mean > 0.1]
    if len(actual_speeds) == 4:
        diff_max = max(actual_speeds) - min(actual_speeds)
        avg_speed = sum(actual_speeds) / 4.0
        pct_diff = (diff_max / avg_speed) * 100.0 if avg_speed > 0 else 0.0
        lines.append(f"Wheel Speed Asymmetry: max spread = {diff_max:.3f} rad/s ({pct_diff:.1f}% deviation between fastest & slowest wheel)")
        if pct_diff > 3.0:
            lines.append("--> Speed asymmetry detected; collect validated direction-specific data before considering calibration.")
            fastest = max(stats, key=lambda s: s.actual_mean).name
            slowest = min(stats, key=lambda s: s.actual_mean).name
            lines.append(f"    Fastest wheel: [{fastest}], Slowest wheel: [{slowest}].")
        else:
            lines.append("--> Wheel speeds are well balanced (< 3% deviation).")
    lines.append("=" * 100)
    return "\n".join(lines)


def plot_wheel_tracking(samples: list[WheelSample], stats: list[WheelStats], pattern: str, output_path: Path) -> None:
    if not HAVE_MATPLOTLIB or not samples:
        return

    t0 = samples[0].time_s
    times = [s.time_s - t0 for s in samples]

    fig, axes = plt.subplots(2, 2, figsize=(12, 9), dpi=150, sharex=True)
    axes_flat = axes.flatten()

    colors = {"target": "#1f77b4", "actual": "#2ca02c", "pwm": "#d62728"}

    for idx, name in enumerate(WHEEL_NAMES):
        ax = axes_flat[idx]
        targets = [s.target_rad_s[idx] for s in samples]
        actuals = [s.velocity_rad_s[idx] for s in samples]
        pwms = [s.pwm_percent[idx] for s in samples]

        # Left axis: Velocities (rad/s)
        ax.plot(times, targets, label="Target (rad/s)", color=colors["target"], linestyle="--", linewidth=1.5)
        ax.plot(times, actuals, label="Actual (rad/s)", color=colors["actual"], linewidth=1.8)
        ax.set_ylabel("Speed (rad/s)", fontsize=10, fontweight="bold")
        ff_text = f"{stats[idx].ground_ff_scale_est:.2f}" if stats[idx].ground_ff_scale_est is not None else "N/A"
        ax.set_title(f"Wheel {name} (RMSE={stats[idx].steady_error_rmse:.2f}, Ground FF={ff_text})",
                     fontsize=11, fontweight="bold")
        ax.grid(True, linestyle=":", alpha=0.6)

        # Right axis: PWM (%)
        ax_pwm = ax.twinx()
        ax_pwm.plot(times, pwms, label="PWM (%)", color=colors["pwm"], linestyle=":", alpha=0.5, linewidth=1.2)
        ax_pwm.set_ylabel("PWM (%)", color=colors["pwm"], fontsize=9)
        ax_pwm.tick_params(axis="y", labelcolor=colors["pwm"])

        # Legends
        if idx == 0:
            lines_1, labels_1 = ax.get_legend_handles_labels()
            lines_2, labels_2 = ax_pwm.get_legend_handles_labels()
            ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper right", fontsize=8)

    for ax in axes[1]:
        ax.set_xlabel("Time (seconds)", fontsize=10, fontweight="bold")

    fig.suptitle(f"Per-Wheel Dynamic Tracking ({pattern.upper()} 0.5m Round-Trip)", fontsize=13, fontweight="bold")
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path)
    plt.close()


class WheelDiagnosticNode(Node if HAVE_ROS else object):  # type: ignore[misc]

    def __init__(
        self,
        pattern: str = "surge",
        distance_m: float = 0.5,
        speed_mps: float = 0.3,
        round_trip: bool = True,
        odom_timeout_s: float = 0.15,
    ) -> None:
        if not HAVE_ROS:
            raise RuntimeError("ROS 2 environment is not available")
        super().__init__("wheel_diagnostic_node")
        self.pattern = pattern
        self.distance_m = distance_m
        self.speed_mps = speed_mps
        self.round_trip = round_trip

        self.samples: list[WheelSample] = []
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.enable_client = self.create_client(SetBool, "/base/enable")
        wheel_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.wheel_sub = self.create_subscription(
            WheelState, "/base/wheel_state", self._on_wheel_state, wheel_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, 1
        )
        self.guard = OdomFreshnessGuard(timeout_s=odom_timeout_s) if OdomFreshnessGuard is not None else None
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_pos: tuple[float, float] | None = None
        self.aborted = False
        self.current_pose = None
        init_motion_runtime(self)

    def stop_robot(self) -> None:
        stop_motion(self)

    def _on_odom(self, msg: Odometry) -> None:
        receive_odometry(self, msg)
        if self.current_pose is not None:
            self.current_x, self.current_y = self.current_pose.x_m, self.current_pose.y_m
            if self.start_pos is None:
                self.start_pos = (self.current_x, self.current_y)

    def _on_wheel_state(self, msg: WheelState) -> None:
        self.samples.append(WheelSample(
            time_s=time.monotonic(),
            target_rad_s=list(msg.target_rad_s),
            commanded_rad_s=list(msg.commanded_rad_s),
            velocity_rad_s=list(msg.velocity_rad_s),
            pwm_percent=list(msg.pwm_percent),
        ))

    def set_base_enabled(self, enable: bool) -> bool:
        if not self.enable_client.wait_for_service(timeout_sec=3.0):
            return False
        req = SetBool.Request()
        req.data = enable
        future = self.enable_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return future.done() and future.result() is not None and future.result().success

    def _drive_leg(self, vx: float, vy: float, target_dist: float) -> bool:
        success, _ = execute_motion(self, vx, vy, 0.0, target_dist, False, 1.2, 30.0)
        return success

    def run_diagnostic(self) -> list[WheelSample]:
        self.get_logger().info("Waiting for /base/wheel_state and /odom...")
        start_wait = time.monotonic()
        while (not self.samples or self.start_pos is None) and time.monotonic() - start_wait < 5.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        if not self.samples:
            self.get_logger().error("No WheelState received from MCU")
            return []

        self.samples.clear()



        vx = self.speed_mps if self.pattern == "surge" else 0.0
        vy = self.speed_mps if self.pattern == "sway" else 0.0

        self.get_logger().info(f"Leg 1: Driving {self.pattern} +{self.distance_m}m at {self.speed_mps}m/s...")
        ok = self._drive_leg(vx, vy, self.distance_m)
        if not ok or self.aborted:
            self.get_logger().error("Leg 1 failed or guard tripped! Aborting remaining legs.")
            self.stop_robot()
            return []

        if self.round_trip:
            self.get_logger().info(f"Leg 2: Returning {self.pattern} -{self.distance_m}m to origin...")
            ok = self._drive_leg(-vx, -vy, self.distance_m)
            if not ok or self.aborted:
                self.get_logger().error("Leg 2 failed or guard tripped! Aborting.")
                self.stop_robot()
                return []

        self.stop_robot()
        self.get_logger().info(f"Recorded {len(self.samples)} wheel state samples.")
        return self.samples


def run_single_pattern(
    pattern: str,
    distance_m: float,
    speed_mps: float,
    round_trip: bool,
    odom_timeout_s: float = 0.15,
    output_dir: Path = Path("artifacts/wheel_calibration"),
    configs: dict[str, dict[str, float]] | None = None,
) -> tuple[list[WheelSample], list[WheelStats], Path, Path]:
    node = WheelDiagnosticNode(
        pattern=pattern,
        distance_m=distance_m,
        speed_mps=speed_mps,
        round_trip=round_trip,
        odom_timeout_s=odom_timeout_s,
    )

    def sig_handler(sig, frame):
        node.aborted = True
        node.stop_robot()
        sys.exit(130)

    signal.signal(signal.SIGINT, sig_handler)

    try:
        completed = bool(node.run_diagnostic()) and not node.aborted
        samples = list(node.samples)
    finally:
        node.set_base_enabled(False)
        node.destroy_node()

    stats = analyze_samples(samples, configs) if completed else []

    out_dir = output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_json = out_dir / f"wheel_diag_{pattern}_{ts}.json"
    out_png = out_dir / f"wheel_diag_{pattern}_{ts}.png"
    suffix = 1
    while out_json.exists() or out_png.exists():
        out_json = out_dir / f"wheel_diag_{pattern}_{ts}_{suffix}.json"
        out_png = out_dir / f"wheel_diag_{pattern}_{ts}_{suffix}.png"
        suffix += 1

    if stats:
        plot_wheel_tracking(samples, stats, pattern, out_png)

    source_path = REPO_ROOT / "esp32/src/main.cpp"
    data = {
        "status": "success" if completed else "failed",
        "failure_reason": node.last_failure,
        "motion_runs": node.motion_results,
        "motion_config": node.motion_config.to_dict(),
        "pattern": pattern,
        "distance_m": distance_m,
        "speed_mps": speed_mps,
        "round_trip": round_trip,
        "controller_baseline": configs or CURRENT_CONFIGS,
        "firmware_source": str(source_path),
        "firmware_source_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
        "samples": [
            {"time_s": s.time_s, "target_rad_s": s.target_rad_s,
             "commanded_rad_s": s.commanded_rad_s, "velocity_rad_s": s.velocity_rad_s,
             "pwm_percent": s.pwm_percent}
            for s in samples
        ],
        "wheel_stats": [asdict(s) for s in stats],
        "direction_stats": {
            direction: [asdict(s) for s in analyze_samples(direction_samples, configs)]
            for direction, direction_samples in split_direction_samples(samples).items()
        },
    }
    with out_json.open("x", encoding="utf-8") as stream:
        json.dump(json_finite(data), stream, indent=2, allow_nan=False)

    return samples, stats, out_json, out_png


def main() -> int:
    parser = argparse.ArgumentParser(description="Per-wheel tracking and ground-load PI+FF diagnosis")
    parser.add_argument("--pattern", choices=["surge", "sway", "both"], default="both",
                        help="Motion direction: surge, sway, or both (default: both)")
    parser.add_argument("--distance", type=float, default=0.5, help="Drive distance in meters per leg (default: 0.5)")
    parser.add_argument("--speed", type=float, default=0.3, help="Drive speed in m/s (default: 0.3)")
    parser.add_argument("--no-round-trip", action="store_true", help="Only drive one way (default: round-trip)")
    parser.add_argument("--odom-timeout", type=float, default=0.15, help="Odometry watchdog freshness timeout in seconds (default: 0.15s)")
    parser.add_argument("--dry-run", action="store_true", help="Test logic without ROS")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/wheel_calibration"))
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    args = parser.parse_args()
    try:
        configs = load_baseline(args.baseline)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"Invalid controller baseline: {exc}", file=sys.stderr)
        return 2

    round_trip = not args.no_round_trip

    if args.dry_run:
        print(f"=== DRY-RUN / SIMULATION MODE ===")
        print(f"Pattern: {args.pattern}, Distance: {args.distance}m, Speed: {args.speed}m/s, RoundTrip: {round_trip}, OdomTimeout: {args.odom_timeout:.3f}s")
        dummy = [WheelSample(
            time_s=float(i) * 0.02,
            target_rad_s=[4.72] * 4,
            commanded_rad_s=[4.72] * 4,
            velocity_rad_s=[4.70, 4.65, 4.75, 4.60],
            pwm_percent=[40, 42, 39, 43],
        ) for i in range(100)]
        stats = analyze_samples(dummy, configs)
        print(format_table(stats))
        return 0

    if OdomFreshnessGuard is None:
        print("Cannot run wheel diagnostic: odometry freshness guard import failed.", file=sys.stderr)
        return 2
    if not HAVE_ROS:
        print("Run inside distrobox ros2-humble with workspace sourced.", file=sys.stderr)
        return 2
    import fcntl
    lock_file = open("/tmp/cleany_odom_eval.lock", "w")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        print("Another calibration motion tool is active.", file=sys.stderr)
        return 1
    rclpy.init()

    patterns = ["surge", "sway"] if args.pattern == "both" else [args.pattern]
    combined_stats: dict[str, list[WheelStats]] = {}
    combined_samples: dict[str, list[WheelSample]] = {}

    for pat in patterns:
        print(f"\n>>> Running {pat.upper()} 0.5m {'Round-Trip' if round_trip else 'One-Way'} Evaluation...")
        samples, stats, json_p, png_p = run_single_pattern(
            pat, args.distance, args.speed, round_trip, odom_timeout_s=args.odom_timeout,
            output_dir=args.output_dir, configs=configs
        )
        combined_stats[pat] = stats
        combined_samples[pat] = samples
        if not samples or len(stats) != len(WHEEL_NAMES):
            print(f"DIAGNOSTIC FAILED [{pat}]: no complete measurements; no combined recommendation.")
            rclpy.shutdown()
            return 1
        print("\n" + format_table(stats, title=f"DIAGNOSIS [{pat.upper()} 0.5m]"))
        print(f"Saved plot to: {png_p}")
        print(f"Saved JSON to: {json_p}")
        time.sleep(1.5)

    rclpy.shutdown()

    # If both patterns were evaluated, compute combined weighted average FF and PI
    direction_stats = {
        pattern: {
            direction: analyze_samples(direction_samples, configs)
            for direction, direction_samples in split_direction_samples(samples).items()
        }
        for pattern, samples in combined_samples.items()
    }
    recommendation_ready = (
        len(patterns) == 2 and not args.no_round_trip
        and has_complete_directional_results(direction_stats)
    )
    if recommendation_ready:
        print("\n" + "=" * 100)
        print("                  COMBINED (SURGE + SWAY) COMPREHENSIVE PI+FF TUNING RECOMMENDATION")
        print("=" * 100)
        print(f"{'Wheel':<6} | {'Current FF':<11} | {'Surge FF':<10} | {'Sway FF':<10} | {'Balanced FF Est':<16} | {'Recommended Kp / Ki':<20}")
        print("-" * 100)

        surge_map = {s.name: s for s in direction_stats["surge"]["forward"]}
        sway_map = {s.name: s for s in direction_stats["sway"]["forward"]}

        recommended_configs = {}

        for name in WHEEL_NAMES:
            s_surge = surge_map[name]
            s_sway = sway_map[name]
            # Measurements are diagnostic evidence, not sufficient grounds for automatic retuning.
            balanced_ff = s_surge.current_ff_scale
            avg_kp, avg_ki = s_surge.suggested_kp, s_surge.suggested_ki

            recommended_configs[name] = {"ff_scale": balanced_ff, "kp": avg_kp, "ki": avg_ki}
            print(f"{name:<6} | {s_surge.current_ff_scale:<11.4f} | {s_surge.ground_ff_scale_est:<10.4f} | {s_sway.ground_ff_scale_est:<10.4f} | {balanced_ff:<16.4f} | Kp={avg_kp:.3f}, Ki={avg_ki:.3f}")

        print("=" * 100)
        print("\nC++ Configuration Snippet for esp32/src/main.cpp:")
        print(f"constexpr cleany::WheelVelocityControllerConfig kFlControllerConfig{{{recommended_configs['FL']['ff_scale']:.4f}F, {recommended_configs['FL']['kp']:.3f}F, {recommended_configs['FL']['ki']:.3f}F, 100.0F}};")
        print(f"constexpr cleany::WheelVelocityControllerConfig kFrControllerConfig{{{recommended_configs['FR']['ff_scale']:.4f}F, {recommended_configs['FR']['kp']:.3f}F, {recommended_configs['FR']['ki']:.3f}F, 100.0F}};")
        print(f"constexpr cleany::WheelVelocityControllerConfig kRlControllerConfig{{{recommended_configs['RL']['ff_scale']:.4f}F, {recommended_configs['RL']['kp']:.3f}F, {recommended_configs['RL']['ki']:.3f}F, 100.0F}};")
        print(f"constexpr cleany::WheelVelocityControllerConfig kRrControllerConfig{{{recommended_configs['RR']['ff_scale']:.4f}F, {recommended_configs['RR']['kp']:.3f}F, {recommended_configs['RR']['ki']:.3f}F, 100.0F}};")
        print()
    elif args.pattern == "both":
        print("Combined recommendation unavailable: both target directions require sufficient steady-state data for every wheel in surge and sway.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
