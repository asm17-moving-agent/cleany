from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
from typing import Any


class OdomGuardError(RuntimeError):
    """Raised when odometry freshness, timestamp monotonicity, or numeric validity fails."""
    pass


@dataclass
class OdomFreshnessGuard:
    """Strict watchdog protecting against stale, frozen, or invalid odometry frames.

    Ensures that odometry is received within timeout_s, timestamp is not frozen across
    consecutive incoming messages, and pose coordinates contain no NaN or Infinite values.
    """
    timeout_s: float = 0.15
    max_consecutive_frozen_stamps: int = 1

    _last_mono_time: float | None = None
    _last_stamp_sec: int | None = None
    _last_stamp_nanosec: int | None = None
    _frozen_stamp_count: int = 0
    _last_pose: Pose2D | None = None
    _is_tripped: bool = False
    _trip_reason: str = ""

    def __post_init__(self) -> None:
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("odom timeout must be finite and positive")
        if self.max_consecutive_frozen_stamps < 1:
            raise ValueError("max_consecutive_frozen_stamps must be positive")

    def update(
        self,
        stamp_sec: int,
        stamp_nanosec: int,
        x: float,
        y: float,
        yaw: float,
        mono_time: float | None = None,
    ) -> Pose2D:
        now = time.monotonic() if mono_time is None else mono_time

        if self._is_tripped:
            raise OdomGuardError(f"Odom guard already tripped: {self._trip_reason}")
        if not math.isfinite(now):
            raise OdomGuardError("Invalid monotonic timestamp")
        if stamp_sec < 0 or not 0 <= stamp_nanosec < 1_000_000_000:
            self._is_tripped = True
            self._trip_reason = "Invalid odometry timestamp"
            raise OdomGuardError(self._trip_reason)
        stamp = (stamp_sec, stamp_nanosec)
        if self._last_stamp_sec is not None and stamp < (
            self._last_stamp_sec, self._last_stamp_nanosec
        ):
            self._is_tripped = True
            self._trip_reason = "Odometry timestamp moved backwards"
            raise OdomGuardError(self._trip_reason)

        # Check NaN or Inf
        if any(math.isnan(v) or math.isinf(v) for v in (x, y, yaw)):
            self._is_tripped = True
            self._trip_reason = f"NaN or Inf pose detected: x={x}, y={y}, yaw={yaw}"
            raise OdomGuardError(self._trip_reason)

        # Check frozen repeated stamps
        if (
            self._last_stamp_sec is not None
            and stamp_sec == self._last_stamp_sec
            and stamp_nanosec == self._last_stamp_nanosec
        ):
            self._frozen_stamp_count += 1
            if self._frozen_stamp_count >= self.max_consecutive_frozen_stamps:
                self._is_tripped = True
                self._trip_reason = f"Repeated frozen odom stamp detected ({stamp_sec}.{stamp_nanosec:09d})"
                raise OdomGuardError(self._trip_reason)
        else:
            self._frozen_stamp_count = 0

        self._last_stamp_sec = stamp_sec
        self._last_stamp_nanosec = stamp_nanosec
        self._last_mono_time = now
        self._last_pose = Pose2D(x_m=x, y_m=y, yaw_rad=yaw)
        return self._last_pose

    def check_freshness(
        self, mono_time: float | None = None, *, trip_on_timeout: bool = True
    ) -> Pose2D:
        if self._is_tripped:
            raise OdomGuardError(f"Odom guard already tripped: {self._trip_reason}")

        now = time.monotonic() if mono_time is None else mono_time
        if self._last_mono_time is None or self._last_pose is None:
            raise OdomGuardError("No odometry message received yet")

        elapsed = now - self._last_mono_time
        if elapsed > self.timeout_s:
            reason = f"Odometry timeout: {elapsed:.3f}s elapsed (limit {self.timeout_s:.3f}s)"
            if trip_on_timeout:
                self._is_tripped = True
                self._trip_reason = reason
            raise OdomGuardError(reason)

        return self._last_pose

    @property
    def is_tripped(self) -> bool:
        return self._is_tripped

    @property
    def trip_reason(self) -> str:
        return self._trip_reason

    def reset(self) -> None:
        self._last_mono_time = None
        self._last_stamp_sec = None
        self._last_stamp_nanosec = None
        self._frozen_stamp_count = 0
        self._last_pose = None
        self._is_tripped = False
        self._trip_reason = ""


MAX_ALLOWED_ORIGIN_DISTANCE_M = 3.0
MAX_LINEAR_VELOCITY_MPS = 0.5
MAX_ANGULAR_VELOCITY_RAD_S = 1.0


@dataclass(frozen=True)
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    @property
    def yaw_deg(self) -> float:
        return math.degrees(self.yaw_rad)

    def distance_to(self, other: Pose2D) -> float:
        return math.hypot(self.x_m - other.x_m, self.y_m - other.y_m)


@dataclass(frozen=True)
class MotionSegment:
    name: str
    vx_mps: float
    vy_mps: float
    wz_rad_s: float
    target_displacement: float
    is_rotation: bool = False
    dwell_after_s: float = 1.5


@dataclass
class EvaluationPlan:
    pattern_name: str
    segments: list[MotionSegment]
    max_origin_distance_m: float
    nominal_total_distance_m: float
    nominal_total_yaw_rad: float

    def validate(self) -> None:
        values = (self.max_origin_distance_m, self.nominal_total_distance_m,
                  self.nominal_total_yaw_rad)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError("Plan dimensions must be finite and nonnegative")
        if self.max_origin_distance_m > MAX_ALLOWED_ORIGIN_DISTANCE_M + 1e-6:
            raise ValueError(
                f"Trajectory maximum distance {self.max_origin_distance_m:.3f}m "
                f"exceeds allowed boundary {MAX_ALLOWED_ORIGIN_DISTANCE_M}m"
            )
        for seg in self.segments:
            values = (seg.vx_mps, seg.vy_mps, seg.wz_rad_s,
                      seg.target_displacement, seg.dwell_after_s)
            if not all(math.isfinite(v) for v in values):
                raise ValueError("Segment parameters must be finite")
            if seg.target_displacement <= 0 or seg.dwell_after_s < 0:
                raise ValueError("Segment target must be positive and dwell nonnegative")
            if (seg.is_rotation and (seg.wz_rad_s == 0 or seg.vx_mps != 0 or seg.vy_mps != 0)
                    or not seg.is_rotation and (seg.wz_rad_s != 0 or math.hypot(seg.vx_mps, seg.vy_mps) == 0)):
                raise ValueError("Segment requires a nonzero pure rotation or translation")
            if abs(seg.vx_mps) > MAX_LINEAR_VELOCITY_MPS + 1e-6:
                raise ValueError(f"vx {seg.vx_mps} exceeds limit {MAX_LINEAR_VELOCITY_MPS}")
            if abs(seg.vy_mps) > MAX_LINEAR_VELOCITY_MPS + 1e-6:
                raise ValueError(f"vy {seg.vy_mps} exceeds limit {MAX_LINEAR_VELOCITY_MPS}")
            if abs(seg.wz_rad_s) > MAX_ANGULAR_VELOCITY_RAD_S + 1e-6:
                raise ValueError(f"wz {seg.wz_rad_s} exceeds limit {MAX_ANGULAR_VELOCITY_RAD_S}")


class PatternBuilder:
    """Build holonomic test trajectories respecting origin distance limit <= 3.0m."""

    @staticmethod
    def build_surge(distance_m: float = 2.5, speed_mps: float = 0.5) -> EvaluationPlan:
        if distance_m <= 0 or distance_m > MAX_ALLOWED_ORIGIN_DISTANCE_M:
            raise ValueError(f"Surge distance must be within (0, {MAX_ALLOWED_ORIGIN_DISTANCE_M}]m")
        speed = min(abs(speed_mps), MAX_LINEAR_VELOCITY_MPS)
        segments = [
            MotionSegment(name="forward_x", vx_mps=speed, vy_mps=0.0, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=2.0),
            MotionSegment(name="return_x", vx_mps=-speed, vy_mps=0.0, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="surge",
            segments=segments,
            max_origin_distance_m=distance_m,
            nominal_total_distance_m=distance_m * 2.0,
            nominal_total_yaw_rad=0.0,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_return_x(distance_m: float = 2.5, speed_mps: float = 0.5) -> EvaluationPlan:
        if distance_m <= 0 or distance_m > MAX_ALLOWED_ORIGIN_DISTANCE_M:
            raise ValueError(f"Return distance must be within (0, {MAX_ALLOWED_ORIGIN_DISTANCE_M}]m")
        speed = min(abs(speed_mps), MAX_LINEAR_VELOCITY_MPS)
        segments = [
            MotionSegment(name="return_x", vx_mps=-speed, vy_mps=0.0, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="return_x",
            segments=segments,
            max_origin_distance_m=distance_m,
            nominal_total_distance_m=distance_m,
            nominal_total_yaw_rad=0.0,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_sway(distance_m: float = 2.0, speed_mps: float = 0.5) -> EvaluationPlan:
        if distance_m <= 0 or distance_m > MAX_ALLOWED_ORIGIN_DISTANCE_M:
            raise ValueError(f"Sway distance must be within (0, {MAX_ALLOWED_ORIGIN_DISTANCE_M}]m")
        speed = min(abs(speed_mps), MAX_LINEAR_VELOCITY_MPS)
        segments = [
            MotionSegment(name="strafe_left_y", vx_mps=0.0, vy_mps=speed, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=2.0),
            MotionSegment(name="strafe_right_y", vx_mps=0.0, vy_mps=-speed, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="sway",
            segments=segments,
            max_origin_distance_m=distance_m,
            nominal_total_distance_m=distance_m * 2.0,
            nominal_total_yaw_rad=0.0,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_return_y(distance_m: float = 2.0, speed_mps: float = 0.5) -> EvaluationPlan:
        if distance_m <= 0 or distance_m > MAX_ALLOWED_ORIGIN_DISTANCE_M:
            raise ValueError(f"Return distance must be within (0, {MAX_ALLOWED_ORIGIN_DISTANCE_M}]m")
        speed = min(abs(speed_mps), MAX_LINEAR_VELOCITY_MPS)
        segments = [
            MotionSegment(name="return_y", vx_mps=0.0, vy_mps=-speed, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="return_y",
            segments=segments,
            max_origin_distance_m=distance_m,
            nominal_total_distance_m=distance_m,
            nominal_total_yaw_rad=0.0,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_yaw(turns: float = 2.0, speed_rad_s: float = 1.0) -> EvaluationPlan:
        if turns <= 0:
            raise ValueError("Turns must be positive")
        speed = min(abs(speed_rad_s), MAX_ANGULAR_VELOCITY_RAD_S)
        target_rad = turns * 2.0 * math.pi
        segments = [
            MotionSegment(name="spin_ccw", vx_mps=0.0, vy_mps=0.0, wz_rad_s=speed,
                          target_displacement=target_rad, is_rotation=True, dwell_after_s=2.0),
            MotionSegment(name="spin_cw", vx_mps=0.0, vy_mps=0.0, wz_rad_s=-speed,
                          target_displacement=target_rad, is_rotation=True, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="yaw",
            segments=segments,
            max_origin_distance_m=0.0,
            nominal_total_distance_m=0.0,
            nominal_total_yaw_rad=target_rad * 2.0,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_yaw_cw(turns: float = 1.0, speed_rad_s: float = 1.0) -> EvaluationPlan:
        if turns <= 0:
            raise ValueError("Turns must be positive")
        speed = min(abs(speed_rad_s), MAX_ANGULAR_VELOCITY_RAD_S)
        target_rad = turns * 2.0 * math.pi
        segments = [
            MotionSegment(name="spin_cw", vx_mps=0.0, vy_mps=0.0, wz_rad_s=-speed,
                          target_displacement=target_rad, is_rotation=True, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="yaw_cw",
            segments=segments,
            max_origin_distance_m=0.0,
            nominal_total_distance_m=0.0,
            nominal_total_yaw_rad=target_rad,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_yaw_ccw(turns: float = 1.0, speed_rad_s: float = 1.0) -> EvaluationPlan:
        if turns <= 0:
            raise ValueError("Turns must be positive")
        speed = min(abs(speed_rad_s), MAX_ANGULAR_VELOCITY_RAD_S)
        target_rad = turns * 2.0 * math.pi
        segments = [
            MotionSegment(name="spin_ccw", vx_mps=0.0, vy_mps=0.0, wz_rad_s=speed,
                          target_displacement=target_rad, is_rotation=True, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="yaw_ccw",
            segments=segments,
            max_origin_distance_m=0.0,
            nominal_total_distance_m=0.0,
            nominal_total_yaw_rad=target_rad,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_box(side_m: float = 2.0, speed_mps: float = 0.5) -> EvaluationPlan:
        # Diagonal from origin is sqrt(2)*side_m. Must be <= 3.0m
        diagonal = math.hypot(side_m, side_m)
        if side_m <= 0 or diagonal > MAX_ALLOWED_ORIGIN_DISTANCE_M:
            raise ValueError(
                f"Box side {side_m}m yields diagonal {diagonal:.3f}m, "
                f"exceeding max radius {MAX_ALLOWED_ORIGIN_DISTANCE_M}m"
            )
        speed = min(abs(speed_mps), MAX_LINEAR_VELOCITY_MPS)
        segments = [
            MotionSegment(name="leg_forward_x", vx_mps=speed, vy_mps=0.0, wz_rad_s=0.0,
                          target_displacement=side_m, is_rotation=False, dwell_after_s=1.5),
            MotionSegment(name="leg_left_y", vx_mps=0.0, vy_mps=speed, wz_rad_s=0.0,
                          target_displacement=side_m, is_rotation=False, dwell_after_s=1.5),
            MotionSegment(name="leg_backward_x", vx_mps=-speed, vy_mps=0.0, wz_rad_s=0.0,
                          target_displacement=side_m, is_rotation=False, dwell_after_s=1.5),
            MotionSegment(name="leg_right_y", vx_mps=0.0, vy_mps=-speed, wz_rad_s=0.0,
                          target_displacement=side_m, is_rotation=False, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="box",
            segments=segments,
            max_origin_distance_m=diagonal,
            nominal_total_distance_m=side_m * 4.0,
            nominal_total_yaw_rad=0.0,
        )
        plan.validate()
        return plan

    @staticmethod
    def build_diagonal(distance_m: float = 2.0, speed_mps: float = 0.5) -> EvaluationPlan:
        if distance_m <= 0 or distance_m > MAX_ALLOWED_ORIGIN_DISTANCE_M:
            raise ValueError(f"Diagonal distance must be within (0, {MAX_ALLOWED_ORIGIN_DISTANCE_M}]m")
        speed = min(abs(speed_mps), MAX_LINEAR_VELOCITY_MPS)
        # 45 deg translation: vx = vy = speed * cos(45)
        v_comp = speed / math.sqrt(2.0)
        segments = [
            MotionSegment(name="diagonal_forward", vx_mps=v_comp, vy_mps=v_comp, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=2.0),
            MotionSegment(name="diagonal_return", vx_mps=-v_comp, vy_mps=-v_comp, wz_rad_s=0.0,
                          target_displacement=distance_m, is_rotation=False, dwell_after_s=1.5),
        ]
        plan = EvaluationPlan(
            pattern_name="diagonal",
            segments=segments,
            max_origin_distance_m=distance_m,
            nominal_total_distance_m=distance_m * 2.0,
            nominal_total_yaw_rad=0.0,
        )
        plan.validate()
        return plan


@dataclass
class GroundTruthResult:
    pattern_name: str
    nominal_distance_m: float
    nominal_yaw_deg: float
    odom_start: Pose2D
    odom_end: Pose2D
    odom_delta_x_m: float
    odom_delta_y_m: float
    odom_delta_yaw_deg: float
    # Physical measured error relative to floor origin (tape or LiDAR ICP)
    physical_error_x_mm: float | None
    physical_error_y_mm: float | None
    physical_error_yaw_deg: float | None
    measurement_method: str = "manual_tape"
    lidar_metrics: dict[str, Any] | None = None
    physical_position_error_norm_mm: float | None = field(init=False)
    error_rate_pct: float | None = field(init=False)
    diagnostic_hint: str = ""
    measurement_valid: bool = True

    def __post_init__(self) -> None:
        values = (self.physical_error_x_mm, self.physical_error_y_mm,
                  self.physical_error_yaw_deg)
        if not self.measurement_valid or any(
            v is None or not math.isfinite(v) for v in values
        ):
            self.measurement_valid = False
            self.physical_error_x_mm = None
            self.physical_error_y_mm = None
            self.physical_error_yaw_deg = None
            self.physical_position_error_norm_mm = None
            self.error_rate_pct = None
            return
        self.physical_position_error_norm_mm = math.hypot(
            self.physical_error_x_mm, self.physical_error_y_mm
        )
        if self.nominal_distance_m > 1e-3:
            self.error_rate_pct = (
                (self.physical_position_error_norm_mm / 1000.0) / self.nominal_distance_m
            ) * 100.0
        elif abs(self.nominal_yaw_deg) > 1e-3:
            self.error_rate_pct = (
                abs(self.physical_error_yaw_deg) / abs(self.nominal_yaw_deg)
            ) * 100.0
        else:
            self.error_rate_pct = 0.0

    def to_dict(self) -> dict[str, Any]:
        data: dict[str, Any] = {
            "pattern_name": self.pattern_name,
            "nominal_distance_m": self.nominal_distance_m,
            "nominal_yaw_deg": self.nominal_yaw_deg,
            "measurement_method": self.measurement_method,
            "is_valid": self.measurement_valid,
            "odom_start": {
                "x_m": self.odom_start.x_m,
                "y_m": self.odom_start.y_m,
                "yaw_deg": self.odom_start.yaw_deg,
            },
            "odom_end": {
                "x_m": self.odom_end.x_m,
                "y_m": self.odom_end.y_m,
                "yaw_deg": self.odom_end.yaw_deg,
            },
            "odom_residual": {
                "delta_x_m": self.odom_delta_x_m,
                "delta_y_m": self.odom_delta_y_m,
                "delta_yaw_deg": self.odom_delta_yaw_deg,
            },
            "physical_ground_truth": {
                "error_x_mm": self.physical_error_x_mm,
                "error_y_mm": self.physical_error_y_mm,
                "error_yaw_deg": self.physical_error_yaw_deg,
                "position_norm_mm": self.physical_position_error_norm_mm,
                "error_rate_pct": self.error_rate_pct,
            },
            "diagnostic_hint": self.diagnostic_hint,
        }
        if self.lidar_metrics is not None:
            data["lidar_metrics"] = self.lidar_metrics
        return data

    def format_summary_table(self) -> str:
        if not self.measurement_valid:
            reason = (self.lidar_metrics or {}).get("rejection_reason", "No explicit measurement")
            return (
                f"=== Odometry Verification Summary: [{self.pattern_name.upper()}] ===\n"
                f"Measurement: INVALID ({reason})\n"
                "Physical error and error rate: unavailable"
            )
        method_title = (
            "2D LiDAR ICP Scan Matching"
            if self.measurement_method == "lidar_icp"
            else "Floor Tape Manual Measurement"
        )
        lines = [
            f"=== Odometry Verification Summary: [{self.pattern_name.upper()}] ===",
            f"Nominal Total Distance: {self.nominal_distance_m:.2f} m | Nominal Yaw: {self.nominal_yaw_deg:.1f} deg",
            f"Measurement Method: {method_title}",
            "----------------------------------------------------------------",
            f"Odom Residual (Loop Closure Reported by ROS):",
            f"  delta_x: {self.odom_delta_x_m * 1000.0:+.1f} mm, delta_y: {self.odom_delta_y_m * 1000.0:+.1f} mm, delta_yaw: {self.odom_delta_yaw_deg:+.2f} deg",
            "----------------------------------------------------------------",
            f"Physical Ground-Truth Error ({method_title}):",
            f"  error_x: {self.physical_error_x_mm:+.1f} mm",
            f"  error_y: {self.physical_error_y_mm:+.1f} mm",
            f"  error_yaw: {self.physical_error_yaw_deg:+.2f} deg",
            f"  2D Position Norm: {self.physical_position_error_norm_mm:.1f} mm",
            f"  Relative Error Rate: {self.error_rate_pct:.2f} %",
        ]
        if self.lidar_metrics:
            rmse = self.lidar_metrics.get("rmse_mm", 0.0)
            inliers = self.lidar_metrics.get("inlier_ratio_pct", 0.0)
            cond = self.lidar_metrics.get("condition_number", 0.0)
            lines.append("----------------------------------------------------------------")
            lines.append(f"LiDAR Match Confidence: RMSE {rmse:.2f} mm | Inliers {inliers:.1f}% | Cond {cond:.1f}")
        lines.extend([
            "----------------------------------------------------------------",
            f"Diagnosis: {self.diagnostic_hint}",
            "================================================================",
        ])
        return "\n".join(lines)


def diagnose_error(
    pattern: str,
    err_x_mm: float,
    err_y_mm: float,
    err_yaw_deg: float,
    nom_dist_m: float,
) -> str:
    hints = []
    if pattern == "surge":
        if abs(err_x_mm) > 30.0:
            if err_x_mm < 0:
                hints.append("Robot stopped short of tape origin: effective wheel radius may be smaller than 0.0635m, or longitudinal drive slip occurred during max speed.")
            else:
                hints.append("Robot overshot tape origin: effective wheel radius may be larger than nominal, or deceleration skid occurred.")
        if abs(err_y_mm) > 30.0:
            hints.append("Lateral deviation detected during pure surge: check left/right wheel diameter balance or normal load asymmetry.")
    elif pattern == "sway":
        if abs(err_y_mm) > 50.0:
            hints.append("Lateral slip detected on Mecanum rollers (expected at 0.5 m/s): roller contact friction coefficient or lateral kinematics scale requires calibration factor.")
        if abs(err_x_mm) > 30.0:
            hints.append("Longitudinal coupling detected during pure sway: roller angle asymmetry or chassis diagonal alignment error.")
    elif pattern == "yaw":
        if abs(err_yaw_deg) > 3.0:
            if err_yaw_deg < 0:
                hints.append("Rotated less than commanded: effective (wheelbase + wheel_separation)/2 is larger than nominal (0.480m) or rotational scrubbing slip occurred.")
            else:
                hints.append("Rotated more than commanded: effective base footprint factor is smaller than nominal.")
    elif pattern in ("box", "diagonal"):
        norm_mm = math.hypot(err_x_mm, err_y_mm)
        if norm_mm > 60.0:
            hints.append(f"Combined planar drift ({norm_mm:.1f}mm) after closed loop: combine individual surge, sway, and yaw calibration factors.")
    if not hints:
        hints.append("Odometry error is within typical indoor floor tolerances (< 2%).")
    return " | ".join(hints)
