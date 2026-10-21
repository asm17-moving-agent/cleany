from types import SimpleNamespace
import json
import math

import pytest
import numpy as np

from evaluator_core import GroundTruthResult, Pose2D
from evaluation_results import build_leg_result, json_finite, write_data_json
from lidar_scan_matcher import ScanMatchResult
from run_odometry_eval import prompt_physical_error


def match(valid):
    return ScanMatchResult(
        delta_x_m=.5, delta_y_m=0, delta_yaw_rad=0, delta_yaw_deg=0,
        rmse_m=0, rmse_mm=0, inlier_ratio=1, num_inliers=100,
        num_total_points=100, converged=True, iterations=1,
        condition_number=1 if valid else math.inf, is_valid=valid,
        rejection_reason="" if valid else "Unobservable geometry",
    )


def test_rejected_icp_never_becomes_numeric_calibration_error():
    def forbidden(*args):
        raise AssertionError("Rejected transform was consumed")

    result = build_leg_result((.5, 0, 0), match(False), .5, False, forbidden)
    assert result["one_way_error"] is None
    assert result["calibrated_base_ground_truth"] is None
    assert result["raw_lidar_icp"]["condition_number"] is None
    json.dumps(result, allow_nan=False)


def test_valid_observation_computes_error():
    result = build_leg_result((.51, 0, 0), match(True), .5, False, lambda x, y, yaw: (x, y, yaw))
    assert result["one_way_error"]["error_norm_mm"] == pytest.approx(10)
    assert result["one_way_error"]["error_rate_pct"] == pytest.approx(2)


def test_missing_physical_measurement_has_no_zero_accuracy():
    result = GroundTruthResult("box", 2, 0, Pose2D(), Pose2D(), 0, 0, 0, None, None, None)
    assert not result.measurement_valid
    assert result.error_rate_pct is None
    assert result.to_dict()["physical_ground_truth"]["position_norm_mm"] is None
    json.dumps(result.to_dict(), allow_nan=False)


@pytest.mark.parametrize("text", ["", "not-a-number", "nan", "inf"])
def test_interactive_missing_or_invalid_measurement_is_rejected(monkeypatch, text):
    monkeypatch.setattr("builtins.input", lambda _: text)
    args = SimpleNamespace(tape_error_x=None, tape_error_y=None, tape_error_yaw=None, manual_fallback=True)
    with pytest.raises(ValueError):
        prompt_physical_error(args)


def test_eof_and_automatic_manual_fallback_are_rejected(monkeypatch):
    def eof(_):
        raise EOFError()

    monkeypatch.setattr("builtins.input", eof)
    args = SimpleNamespace(tape_error_x=None, tape_error_y=None, tape_error_yaw=None, manual_fallback=True)
    with pytest.raises(ValueError):
        prompt_physical_error(args)
    args.manual_fallback = False
    with pytest.raises(ValueError, match="fallback is disabled"):
        prompt_physical_error(args)


def test_explicit_zero_is_a_valid_measurement():
    args = SimpleNamespace(tape_error_x=0, tape_error_y=0, tape_error_yaw=0)
    assert prompt_physical_error(args) == (0, 0, 0)


def test_strict_json_preserves_failure_without_nonstandard_infinity():
    result = json_finite({"invalid": [math.inf, -math.inf, math.nan], "error": None})
    assert result == {"invalid": [None, None, None], "error": None}
    json.dumps(result, allow_nan=False)


def test_ros_numpy_arrays_and_scalars_have_standard_json_types():
    result = json_finite({
        "counts": np.asarray([1, 2], dtype=np.int32),
        "speed": [np.float32(.5), np.float32("inf")],
        "sequence": np.uint32(4), "enabled": True,
    })
    assert result == {"counts": [1, 2], "speed": [.5, None], "sequence": 4, "enabled": True}
    json.dumps(result, allow_nan=False)


def test_atomic_data_writer_never_replaces_existing_run(tmp_path):
    path = tmp_path / "run.json"
    write_data_json(path, {"is_valid": False, "error": None})
    with pytest.raises(FileExistsError):
        write_data_json(path, {"is_valid": True, "error": 0})
    assert json.loads(path.read_text()) == {"is_valid": False, "error": None}
    assert list(tmp_path.iterdir()) == [path]


def test_serialization_failure_does_not_publish_partial_json(tmp_path):
    path = tmp_path / "run.json"
    with pytest.raises(TypeError):
        write_data_json(path, {"unsupported": object()})
    assert not path.exists()


@pytest.mark.parametrize("failure", ["motion", "icp"])
def test_regular_runner_preserves_failure_and_cleans_up(monkeypatch, tmp_path, failure):
    import run_odometry_eval as runner

    calls = []
    def fail_match(*args):
        raise RuntimeError("Synthetic ICP exception")

    node = SimpleNamespace(
        motion_results=[{"is_valid": False}], aborted=False, last_failure="",
        stop_robot=lambda: calls.append("stop"), destroy_node=lambda: calls.append("destroy"),
        matcher=SimpleNamespace(match=fail_match),
        run_plan=lambda: (
            None if failure == "motion" else
            (Pose2D(), Pose2D(), np.zeros((40, 2)), np.zeros((40, 2)))
        ),
    )
    monkeypatch.setattr(runner, "OdometryEvaluatorNode", lambda **kwargs: node)
    monkeypatch.setattr(runner, "rclpy", SimpleNamespace(
        init=lambda: calls.append("init"), shutdown=lambda: calls.append("shutdown"),
    ))
    output = tmp_path / "failure.json"
    status = runner.main(["--pattern", "box", "--distance", ".5", "--speed", ".2",
                          "--no-plots", "--output", str(output)])
    data = json.loads(output.read_text())
    assert status == 1 and data["status"] == "failed" and not data["is_valid"]
    assert data["failure_reason"]
    assert "stop" in calls and calls[-2:] == ["destroy", "shutdown"]


def test_oneway_motion_failure_has_individual_and_summary_data(monkeypatch, tmp_path):
    import run_oneway_eval as runner

    calls = []
    node = SimpleNamespace(
        motion_results=[{"is_valid": False}], aborted=False,
        last_failure="Synthetic stale odometry", current_stage="outbound_motion",
        stop_robot=lambda: calls.append("stop"), destroy_node=lambda: calls.append("destroy"),
    )
    monkeypatch.setattr(runner, "OneWayEvaluatorNode", lambda **kwargs: node)
    monkeypatch.setattr(runner, "run_single_oneway_test", lambda *args: None)
    monkeypatch.setattr(runner, "rclpy", SimpleNamespace(
        init=lambda: None, shutdown=lambda: calls.append("shutdown"),
    ))
    status = runner.main(["--patterns", "surge", "--linear-speeds", ".2",
                          "--no-plots", "--output-dir", str(tmp_path)])
    summary = list(tmp_path.glob("oneway_evaluation_summary_*.json"))
    individual = list(tmp_path.glob("oneway_surge_*.json"))
    assert status == 1 and len(summary) == len(individual) == 1
    assert json.loads(individual[0].read_text())["failed_stage"] == "outbound_motion"
    assert json.loads(summary[0].read_text())["status"] == "failed"
    assert calls[-2:] == ["destroy", "shutdown"]
