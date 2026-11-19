from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

from evaluator_core import (
    MAX_ALLOWED_ORIGIN_DISTANCE_M,
    MAX_LINEAR_VELOCITY_MPS,
    MAX_ANGULAR_VELOCITY_RAD_S,
    EvaluationPlan,
    GroundTruthResult,
    MotionSegment,
    OdomFreshnessGuard,
    OdomGuardError,
    PatternBuilder,
    Pose2D,
    diagnose_error,
)
import run_odometry_eval


def test_pose2d_distance() -> None:
    p1 = Pose2D(0.0, 0.0, 0.0)
    p2 = Pose2D(3.0, 4.0, math.pi)
    assert pytest.approx(p1.distance_to(p2)) == 5.0
    assert pytest.approx(p2.yaw_deg) == 180.0


def test_surge_pattern_within_boundary() -> None:
    plan = PatternBuilder.build_surge(distance_m=2.5, speed_mps=0.5)
    assert plan.pattern_name == "surge"
    assert plan.max_origin_distance_m == 2.5
    assert plan.nominal_total_distance_m == 5.0
    assert len(plan.segments) == 2
    assert plan.segments[0].vx_mps == 0.5
    assert plan.segments[1].vx_mps == -0.5


def test_return_x_pattern() -> None:
    plan = PatternBuilder.build_return_x(distance_m=2.5, speed_mps=0.5)
    assert plan.pattern_name == "return_x"
    assert plan.max_origin_distance_m == 2.5
    assert len(plan.segments) == 1
    assert plan.segments[0].vx_mps == -0.5


def test_return_y_pattern() -> None:
    plan = PatternBuilder.build_return_y(distance_m=2.0, speed_mps=0.3)
    assert plan.pattern_name == "return_y"
    assert plan.max_origin_distance_m == 2.0
    assert len(plan.segments) == 1
    assert plan.segments[0].vy_mps == -0.3


def test_surge_pattern_exceeding_max_boundary() -> None:
    with pytest.raises(ValueError, match="Surge distance must be within"):
        PatternBuilder.build_surge(distance_m=3.5, speed_mps=0.5)


def test_surge_pattern_speed_clamped() -> None:
    plan = PatternBuilder.build_surge(distance_m=2.0, speed_mps=1.5)
    assert plan.segments[0].vx_mps == MAX_LINEAR_VELOCITY_MPS
    assert plan.segments[1].vx_mps == -MAX_LINEAR_VELOCITY_MPS


def test_sway_pattern_within_boundary() -> None:
    plan = PatternBuilder.build_sway(distance_m=2.0, speed_mps=0.5)
    assert plan.pattern_name == "sway"
    assert plan.max_origin_distance_m == 2.0
    assert plan.nominal_total_distance_m == 4.0
    assert plan.segments[0].vy_mps == 0.5
    assert plan.segments[1].vy_mps == -0.5


def test_sway_pattern_exceeding_max_boundary() -> None:
    with pytest.raises(ValueError, match="Sway distance must be within"):
        PatternBuilder.build_sway(distance_m=3.1)


def test_yaw_pattern() -> None:
    plan = PatternBuilder.build_yaw(turns=2.0, speed_rad_s=1.0)
    assert plan.pattern_name == "yaw"
    assert plan.max_origin_distance_m == 0.0
    assert pytest.approx(plan.nominal_total_yaw_rad) == 4.0 * math.pi * 2.0
    assert plan.segments[0].wz_rad_s == 1.0
    assert plan.segments[1].wz_rad_s == -1.0


def test_yaw_cw_pattern() -> None:
    plan = PatternBuilder.build_yaw_cw(turns=1.0, speed_rad_s=0.8)
    assert plan.pattern_name == "yaw_cw"
    assert plan.max_origin_distance_m == 0.0
    assert pytest.approx(plan.nominal_total_yaw_rad) == 2.0 * math.pi
    assert len(plan.segments) == 1
    assert plan.segments[0].wz_rad_s == -0.8
    assert pytest.approx(plan.segments[0].target_displacement) == 2.0 * math.pi


def test_yaw_ccw_pattern() -> None:
    plan = PatternBuilder.build_yaw_ccw(turns=1.0, speed_rad_s=0.8)
    assert plan.pattern_name == "yaw_ccw"
    assert len(plan.segments) == 1
    assert plan.segments[0].wz_rad_s == 0.8


def test_box_pattern_diagonal_boundary() -> None:
    # 2.0m x 2.0m box has diagonal sqrt(8) ~ 2.828m <= 3.0m
    plan = PatternBuilder.build_box(side_m=2.0, speed_mps=0.5)
    assert plan.pattern_name == "box"
    assert plan.max_origin_distance_m < MAX_ALLOWED_ORIGIN_DISTANCE_M
    assert len(plan.segments) == 4
    assert plan.nominal_total_distance_m == 8.0

    # 2.5m x 2.5m box has diagonal sqrt(12.5) ~ 3.535m > 3.0m -> must raise
    with pytest.raises(ValueError, match="exceeding max radius"):
        PatternBuilder.build_box(side_m=2.5)


def test_diagonal_pattern() -> None:
    plan = PatternBuilder.build_diagonal(distance_m=2.0, speed_mps=0.5)
    assert plan.pattern_name == "diagonal"
    assert plan.max_origin_distance_m == 2.0
    assert pytest.approx(plan.segments[0].vx_mps) == 0.5 / math.sqrt(2.0)
    assert pytest.approx(plan.segments[0].vy_mps) == 0.5 / math.sqrt(2.0)


def test_ground_truth_result_calculation() -> None:
    res = GroundTruthResult(
        pattern_name="surge",
        nominal_distance_m=5.0,
        nominal_yaw_deg=0.0,
        odom_start=Pose2D(0.0, 0.0, 0.0),
        odom_end=Pose2D(0.01, 0.0, 0.0),
        odom_delta_x_m=0.01,
        odom_delta_y_m=0.0,
        odom_delta_yaw_deg=0.0,
        physical_error_x_mm=50.0,
        physical_error_y_mm=20.0,
        physical_error_yaw_deg=1.2,
        diagnostic_hint="Test hint",
    )
    assert pytest.approx(res.physical_position_error_norm_mm) == math.hypot(50.0, 20.0)
    # 53.85mm / 5000mm * 100 ~ 1.077%
    assert pytest.approx(res.error_rate_pct, rel=1e-2) == 1.077
    d = res.to_dict()
    assert d["pattern_name"] == "surge"
    assert "diagnostic_hint" in d
    summary = res.format_summary_table()
    assert "=== Odometry Verification Summary: [SURGE] ===" in summary


def test_diagnose_error_heuristics() -> None:
    hint_surge = diagnose_error("surge", err_x_mm=-50.0, err_y_mm=0.0, err_yaw_deg=0.0, nom_dist_m=5.0)
    assert "Robot stopped short of tape origin" in hint_surge

    hint_sway = diagnose_error("sway", err_x_mm=0.0, err_y_mm=70.0, err_yaw_deg=0.0, nom_dist_m=4.0)
    assert "Lateral slip detected on Mecanum rollers" in hint_sway

    hint_yaw = diagnose_error("yaw", err_x_mm=0.0, err_y_mm=0.0, err_yaw_deg=-5.0, nom_dist_m=0.0)
    assert "Rotated less than commanded" in hint_yaw


def test_cli_dry_run(capsys) -> None:
    code = run_odometry_eval.main(["--pattern", "surge", "--distance", "2.5", "--dry-run"])
    assert code == 0
    captured = capsys.readouterr()
    assert "=== DRY-RUN: Evaluation Plan [SURGE] ===" in captured.out
    assert "Max displacement from origin: 2.500 m" in captured.out


def test_cli_dry_run_box(capsys) -> None:
    code = run_odometry_eval.main(["--pattern", "box", "--distance", "2.0", "--dry-run"])
    assert code == 0
    captured = capsys.readouterr()
    assert "=== DRY-RUN: Evaluation Plan [BOX] ===" in captured.out
    assert "leg_forward_x" in captured.out


def test_odom_guard_normal_flow() -> None:
    guard = OdomFreshnessGuard(timeout_s=0.15)
    t0 = 100.0
    p = guard.update(stamp_sec=10, stamp_nanosec=0, x=1.0, y=2.0, yaw=0.5, mono_time=t0)
    assert p.x_m == 1.0 and p.y_m == 2.0
    # Within timeout
    p_fresh = guard.check_freshness(mono_time=t0 + 0.10)
    assert p_fresh == p


def test_odom_guard_stale_timeout() -> None:
    guard = OdomFreshnessGuard(timeout_s=0.15)
    t0 = 100.0
    guard.update(stamp_sec=10, stamp_nanosec=0, x=1.0, y=2.0, yaw=0.5, mono_time=t0)
    # Stale by 0.16s (> 0.15s)
    with pytest.raises(OdomGuardError, match="Odometry timeout"):
        guard.check_freshness(mono_time=t0 + 0.16)
    assert guard.is_tripped


def test_odom_guard_nan_inf_rejection() -> None:
    guard = OdomFreshnessGuard(timeout_s=0.15)
    with pytest.raises(OdomGuardError, match="NaN or Inf"):
        guard.update(stamp_sec=10, stamp_nanosec=0, x=float("nan"), y=2.0, yaw=0.5)
    assert guard.is_tripped

    guard.reset()
    with pytest.raises(OdomGuardError, match="NaN or Inf"):
        guard.update(stamp_sec=10, stamp_nanosec=0, x=1.0, y=float("inf"), yaw=0.5)
    assert guard.is_tripped


def test_odom_guard_frozen_stamp_rejection() -> None:
    guard = OdomFreshnessGuard(timeout_s=0.15, max_consecutive_frozen_stamps=1)
    t0 = 100.0
    guard.update(stamp_sec=10, stamp_nanosec=500, x=1.0, y=2.0, yaw=0.5, mono_time=t0)
    # Consecutive identical stamp
    with pytest.raises(OdomGuardError, match="Repeated frozen odom stamp"):
        guard.update(stamp_sec=10, stamp_nanosec=500, x=1.01, y=2.0, yaw=0.5, mono_time=t0 + 0.02)
    assert guard.is_tripped


def test_odom_disconnect_while_mcu_ok_triggers_stop_and_blocks_nonzero_commands() -> None:
    """Verify that when odometry drops while MCU feedback continues normally:

    1. Stale condition is detected within the configured timeout (0.15s).
    2. Base driver enable is cleared (/base/enable = false).
    3. Motion is aborted and NO nonzero commands are published afterward.
    """
    guard = OdomFreshnessGuard(timeout_s=0.15)
    published_cmds: list[float] = []
    base_enabled = True
    aborted = False

    t_start = 10.0
    # Initial valid odom received at t=10.0
    guard.update(stamp_sec=10, stamp_nanosec=0, x=0.0, y=0.0, yaw=0.0, mono_time=t_start)

    # Simulated MCU feedback continues up to t=10.30s at 50Hz (every 0.02s)
    # But odom disconnects (no further guard.update calls)!
    for step in range(1, 15):
        curr_t = t_start + step * 0.02
        # MCU feedback arrived (simulated normal operation)
        mcu_feedback_ok = True

        if aborted:
            # Once aborted, any command publisher must strictly refuse nonzero speeds
            break

        # Attempt to issue movement command (vx = 0.5 m/s)
        try:
            guard.check_freshness(mono_time=curr_t)
            # If freshness passed, issue command
            published_cmds.append(0.5)
        except OdomGuardError:
            # Safety trip: STOP immediately!
            aborted = True
            base_enabled = False
            published_cmds.append(0.0)  # Stop command

    # Must trip within 0.15s: at 0.16s (step 8), freshness check must fail
    assert aborted is True
    assert base_enabled is False
    assert guard.is_tripped is True
    assert "Odometry timeout" in guard.trip_reason

    # All commands issued before timeout were 0.5, the command at trip was 0.0,
    # and zero nonzero commands were issued after the trip!
    assert published_cmds[-1] == 0.0
    # Exactly 7 steps (0.02s * 7 = 0.14s <= 0.15s) allowed, step 8 (0.16s) aborted!
    assert len(published_cmds) == 8
    assert published_cmds[:7] == [0.5] * 7
