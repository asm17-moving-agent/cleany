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
    # 2.0×2.0m 상자 대각선은 sqrt(8) ≈ 2.828m로 3.0m 이하입니다.
    plan = PatternBuilder.build_box(side_m=2.0, speed_mps=0.5)
    assert plan.pattern_name == "box"
    assert plan.max_origin_distance_m < MAX_ALLOWED_ORIGIN_DISTANCE_M
    assert len(plan.segments) == 4
    assert plan.nominal_total_distance_m == 8.0

    # 2.5×2.5m 상자 대각선은 sqrt(12.5) ≈ 3.535m로 3.0m를 넘어 예외가 필요합니다.
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
    # 53.85mm / 5000mm × 100 ≈ 1.077%
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
    # 시간 제한 안입니다.
    p_fresh = guard.check_freshness(mono_time=t0 + 0.10)
    assert p_fresh == p


def test_odom_guard_stale_timeout() -> None:
    guard = OdomFreshnessGuard(timeout_s=0.15)
    t0 = 100.0
    guard.update(stamp_sec=10, stamp_nanosec=0, x=1.0, y=2.0, yaw=0.5, mono_time=t0)
    # 0.16초로 제한 0.15초를 초과했습니다.
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
    # 연속으로 같은 stamp입니다.
    with pytest.raises(OdomGuardError, match="Repeated frozen odom stamp"):
        guard.update(stamp_sec=10, stamp_nanosec=500, x=1.01, y=2.0, yaw=0.5, mono_time=t0 + 0.02)
    assert guard.is_tripped


def test_odom_disconnect_while_mcu_ok_triggers_stop_and_blocks_nonzero_commands() -> None:
    """MCU 피드백은 계속되지만 odometry가 끊긴 상황을 모사합니다.

    설정된 0.15초 제한 이후 첫 검사에서 오래된 입력을 검출하고,
    구동 해제 및 정지 뒤에는 0이 아닌 명령을 보내지 않는지 확인합니다.
    실제 하드웨어 정지 검증은 아닙니다.
    """
    guard = OdomFreshnessGuard(timeout_s=0.15)
    published_cmds: list[float] = []
    base_enabled = True
    aborted = False

    t_start = 10.0
    # t=10.0에 최초 유효 odom을 받습니다.
    guard.update(stamp_sec=10, stamp_nanosec=0, x=0.0, y=0.0, yaw=0.0, mono_time=t_start)

    # 합성 MCU 피드백은 t=10.30초까지 50Hz(0.02초마다)로 계속되지만
    # odom은 끊겨 추가 guard.update 호출이 없습니다.
    for step in range(1, 15):
        curr_t = t_start + step * 0.02
        # 정상 동작을 모사하는 MCU 피드백이 도착했습니다.
        mcu_feedback_ok = True

        if aborted:
            # 중단 이후 모든 명령 publisher는 0이 아닌 속도를 거부해야 합니다.
            break

        # vx=0.5m/s 이동 명령을 시도합니다.
        try:
            guard.check_freshness(mono_time=curr_t)
            # 최신성을 통과하면 명령을 보냅니다.
            published_cmds.append(0.5)
        except OdomGuardError:
            # 안전 조건 위반 시 즉시 STOP합니다.
            aborted = True
            base_enabled = False
            published_cmds.append(0.0)  # 정지 명령입니다.

    # 0.15초 제한 이후 첫 검사인 0.16초(step 8)에는 최신성 검사가 실패해야 합니다.
    assert aborted is True
    assert base_enabled is False
    assert guard.is_tripped is True
    assert "Odometry timeout" in guard.trip_reason

    # 시간 초과 전 명령은 모두 0.5, 실패 시 명령은 0.0이며
    # 그 이후 0이 아닌 명령은 없어야 합니다.
    assert published_cmds[-1] == 0.0
    # 0.02초 × 7 = 0.14초까지 일곱 step만 허용하고 step 8인 0.16초에 중단합니다.
    assert len(published_cmds) == 8
    assert published_cmds[:7] == [0.5] * 7
