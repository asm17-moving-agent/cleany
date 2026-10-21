import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from evaluator_core import Pose2D
from motion_patterns import motion_vectors
from planar_extrinsic import PlanarExtrinsic
import run_oneway_eval as runner


@pytest.mark.parametrize("pattern", ["diagonal_left", "diagonal_right"])
def test_diagonal_speed_normalization_and_return(pattern):
    out, back = motion_vectors(pattern, 0.5)
    assert math.hypot(out[0], out[1]) == pytest.approx(0.5)
    assert out[0] == pytest.approx(0.5 / math.sqrt(2))
    assert abs(out[1]) == pytest.approx(0.5 / math.sqrt(2))
    assert back == tuple(-v for v in out)


@pytest.mark.parametrize("speed", [0, -1, math.inf, math.nan])
def test_invalid_speed_rejected(speed):
    with pytest.raises(ValueError):
        motion_vectors("diagonal_left", speed)


def test_invalid_pattern_rejected_before_scan(tmp_path):
    node = SimpleNamespace(current_pose=None)
    with pytest.raises(ValueError, match="Unknown pattern"):
        runner.run_single_oneway_test(node, "bad", 0.2, 0.5, Path(tmp_path / "x"))


@pytest.mark.parametrize("pattern,sign", [("diagonal_left", 1), ("diagonal_right", -1)])
def test_diagonal_full_fake_round_trip_uses_normalized_vectors(tmp_path, pattern, sign):
    calls = []
    extrinsic = PlanarExtrinsic(yaw_deg=20, lever_arm_x_m=.1, lever_arm_y_m=-.05)
    node = SimpleNamespace(
        current_pose=Pose2D(), extrinsic=extrinsic,
        matcher=SimpleNamespace(match=lambda reference, target, *, initial_guess:
            runner.ScanMatchResult(
                delta_x_m=initial_guess[0], delta_y_m=initial_guess[1],
                delta_yaw_rad=initial_guess[2],
                delta_yaw_deg=math.degrees(initial_guess[2]), rmse_m=0,
                rmse_mm=0, inlier_ratio=1, num_inliers=40, num_total_points=40,
                converged=True, iterations=1, condition_number=1)),
        recent_scans=[], capture_scan_burst=lambda **kwargs: __import__("numpy").zeros((40, 2)),
        plot_enabled=False, evaluation_settings={}, stop_robot=lambda: None,
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )

    def execute(vx, vy, wz, target, is_rotation, **kwargs):
        calls.append((vx, vy, wz, target, is_rotation))
        node.current_pose = Pose2D(.5 / math.sqrt(2), sign * .5 / math.sqrt(2), 0) if vx > 0 else Pose2D()
        return True, {"is_valid": True}

    node.execute_leg = execute
    result = runner.run_single_oneway_test(node, pattern, .2, .5, tmp_path / "trial")
    assert result["is_valid"]
    assert calls[0][:3] == pytest.approx((.2 / math.sqrt(2), sign * .2 / math.sqrt(2), 0))
    assert calls[1][:3] == pytest.approx((-.2 / math.sqrt(2), -sign * .2 / math.sqrt(2), 0))
    assert result["extrinsic_calibration_applied"] == extrinsic.to_dict()
