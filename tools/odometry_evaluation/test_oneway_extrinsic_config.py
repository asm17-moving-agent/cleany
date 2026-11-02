"""Current-mount evaluation settings and laser-frame ICP seeds."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from planar_extrinsic import PlanarExtrinsic
from evaluator_core import Pose2D
from lidar_scan_matcher import LidarScanMatcher, ScanMatchResult
import run_oneway_eval as runner


CONFIG = Path(__file__).resolve().parents[2] / "ros2_ws/src/cleany_base_driver/config/lidar_extrinsic_20261015.json"


def test_current_mount_dry_run_reports_wheel_input_and_extrinsic(capsys):
    status = runner.main([
        "--patterns", "surge", "sway", "rotation",
        "--linear-speeds", ".2", ".35", ".5",
        "--angular-speeds", ".3", ".6", "1",
        "--distance", ".5", "--rotation-angle-deg", "90",
        "--extrinsic-config", str(CONFIG),
        "--odom-topic", "/wheel/odom", "--lidar-min-range", ".45", "--dry-run",
    ])
    output = capsys.readouterr().out
    assert status == 0
    assert "/wheel/odom" in output
    assert "'lidar_min_range_m': 0.45" in output
    assert "79.77007003626845" in output


@pytest.mark.parametrize("value", ["nan", "inf", "-0.1", "10"])
def test_invalid_lidar_filter_rejected_before_motion(value):
    with pytest.raises(SystemExit) as error:
        runner.main(["--lidar-min-range", value, "--dry-run"])
    assert error.value.code == 2


def test_icp_seed_uses_inverse_extrinsic_and_both_lever_arms():
    extrinsic = PlanarExtrinsic.from_json(CONFIG)
    base = (.5, -.2, math.pi / 2)
    captured = []
    sentinel = object()

    def match(reference, target, *, initial_guess):
        captured.append(initial_guess)
        return sentinel

    node = SimpleNamespace(extrinsic=extrinsic, matcher=SimpleNamespace(match=match))
    assert runner.match_with_base_guess(node, np.zeros((1, 2)), np.zeros((1, 2)), base) is sentinel
    assert captured[0] == pytest.approx(extrinsic.base_to_lidar(*base))
    assert captured[0] != pytest.approx(base)
    assert extrinsic.lidar_to_base(*captured[0]) == pytest.approx(base)


def test_shared_conversion_matches_legacy_transform_formula():
    extrinsic = PlanarExtrinsic(yaw_deg=79.77, lever_arm_x_m=-.0708, lever_arm_y_m=-.0382)
    delta = (.31, -.27, .8)
    expected = runner.transform_lidar_to_base(
        *delta, extrinsic_yaw_rad=math.radians(extrinsic.yaw_deg),
        lever_arm_x_m=extrinsic.lever_arm_x_m, lever_arm_y_m=extrinsic.lever_arm_y_m,
    )
    assert extrinsic.lidar_to_base(*delta) == pytest.approx(expected)


def test_invalid_config_does_not_become_legacy_default(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text(json.dumps({"yaw_deg": 79.77}))
    with pytest.raises(SystemExit) as error:
        runner.main(["--extrinsic-config", str(path), "--dry-run"])
    assert error.value.code == 2


def test_complete_round_trip_records_config_and_transforms_all_icp_guesses(tmp_path):
    extrinsic = PlanarExtrinsic.from_json(CONFIG)
    seeds = []

    def match(reference, target, *, initial_guess):
        seeds.append(initial_guess)
        x, y, yaw = initial_guess
        return ScanMatchResult(
            delta_x_m=x, delta_y_m=y, delta_yaw_rad=yaw,
            delta_yaw_deg=math.degrees(yaw), rmse_m=0, rmse_mm=0,
            inlier_ratio=1, num_inliers=100, num_total_points=100,
            converged=True, iterations=1, condition_number=1,
        )

    node = SimpleNamespace(
        current_pose=Pose2D(), extrinsic=extrinsic,
        matcher=SimpleNamespace(match=match), recent_scans=[],
        capture_scan_burst=lambda **kwargs: np.zeros((40, 2)),
        plot_enabled=False, evaluation_settings={"odom_topic": "/wheel/odom"},
        stop_robot=lambda: None,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )

    def execute(vx, vy, wz, target, is_rotation, **kwargs):
        node.current_pose = Pose2D(.5 if vx > 0 else 0, 0, 0)
        return True, {"is_valid": True}

    node.execute_leg = execute
    result = runner.run_single_oneway_test(node, "surge", .2, .5, tmp_path / "trial")
    assert result["is_valid"]
    assert result["extrinsic_calibration_applied"] == extrinsic.to_dict()
    assert result["evaluation_settings"]["odom_topic"] == "/wheel/odom"
    assert len(seeds) == 3
    assert seeds[0] == pytest.approx(extrinsic.base_to_lidar(.5, 0, 0))
    assert seeds[1] == pytest.approx(extrinsic.base_to_lidar(-.5, 0, 0))
    assert seeds[2] == pytest.approx((0, 0, 0))
    for leg in ("outbound", "inbound"):
        assert result[leg]["one_way_error"]["error_norm_mm"] == pytest.approx(0, abs=1e-9)


def test_current_mount_range_filter_excludes_observed_profile_distances():
    matcher = LidarScanMatcher(min_range_m=.45)
    points = matcher.laserscan_to_points(
        [.234, .2995, .3345, .3805, 1.0], 0, .1, .15, 12,
    )
    assert len(points) == 1
    assert np.linalg.norm(points[0]) == pytest.approx(1)
