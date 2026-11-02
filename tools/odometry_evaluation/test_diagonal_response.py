"""Offline regression checks for diagonal tuning, geometry and reference validity."""
from copy import deepcopy
import json
import math

import pytest

from evaluator_core import Pose2D
from motion_control import MotionConfig, MotionController
from motion_response import endpoint_errors, plot_responses, translation_direction
from run_motion_response import ROOT, bound_violation, candidate_config, load_config, main


def test_diagonal_config_and_dry_run(capsys):
    assert main(["--config", "configs/mission/diagonal_response_tuning.json", "--dry-run"]) == 0
    assert '"directional_legs": 16' in capsys.readouterr().out
    assert main(["--config", "configs/mission/diagonal_response_validation.json", "--dry-run"]) == 0
    assert '"directional_legs": 36' in capsys.readouterr().out


def test_wheel_batch_accepts_diagonal_patterns_without_motion(tmp_path, capsys):
    from run_wheel_only_eval import main as batch_main
    config = json.loads((ROOT / "configs/mission/wheel_odometry_evaluation.json").read_text())
    config["patterns"] = ["diagonal_left", "diagonal_right"]
    config["output_dir"] = str(tmp_path / "not_created")
    source = tmp_path / "diagonal.json"
    source.write_text(json.dumps(config))
    assert batch_main(["--config", str(source), "--dry-run"]) == 0
    assert "directional legs: 36" in capsys.readouterr().out
    assert not (tmp_path / "not_created").exists()


@pytest.mark.parametrize("axis", ["diagonal_left", "diagonal_right"])
@pytest.mark.parametrize("sign", [-1., 1.])
@pytest.mark.parametrize("heading", [0., 1.7, -2.1])
def test_diagonal_guards_are_along_and_normal_to_command(axis, sign, heading):
    guards = load_config(ROOT / "configs/mission/diagonal_response_tuning.json")["guards"]
    start = Pose2D(.03, -.04, heading)
    ux, uy = translation_direction(axis)
    def pose(along, cross):
        bx, by = ux*along - uy*cross, uy*along + ux*cross
        return Pose2D(start.x_m + math.cos(heading)*bx - math.sin(heading)*by,
                      start.y_m + math.sin(heading)*bx + math.cos(heading)*by, heading)
    assert not bound_violation(pose(sign*.5, .01), start, axis, .5, guards, start)
    assert "cross-track" in bound_violation(pose(sign*.3, .081), start, axis, .5, guards, start)
    assert "excursion" in bound_violation(pose(sign*.526, 0), start, axis, .5, guards, start)


@pytest.mark.parametrize("axis", ["diagonal_left", "diagonal_right"])
def test_diagonal_candidate_uses_fallback_and_preserves_axial_gains(axis):
    base = MotionConfig(linear_kp=1, linear_x_kp=6, linear_y_kp=6, yaw_kp=6)
    candidate = candidate_config(base, axis, 4)
    assert (candidate.linear_kp, candidate.linear_x_kp, candidate.linear_y_kp, candidate.yaw_kp) == (4, 6, 6, 6)
    ux, uy = translation_direction(axis)
    ctrl = MotionController(Pose2D(0, 0, 1.), ux, uy, 0, .01, False, candidate)
    step = ctrl.step(Pose2D(0, 0, 1.), 0, 0, 0)
    assert math.hypot(step.vx_mps, step.vy_mps) == pytest.approx(.04)


@pytest.mark.parametrize("axis", ["diagonal_left", "diagonal_right"])
def test_endpoint_errors_separate_goal_error_and_odometry_bias(axis):
    ux, uy = translation_direction(axis)
    result = {"nominal_displacement": .5}
    for leg, sign in (("outbound", 1), ("inbound", -1)):
        def pose(along, cross):
            return {"dx_m": sign*(ux*along - uy*cross), "dy_m": sign*(uy*along + ux*cross),
                    "dyaw_deg": 1.}
        result[leg] = {"is_valid": True, "odom_reported": pose(.496, .003),
                       "calibrated_base_ground_truth": pose(.510, .008),
                       "one_way_error": {"error_norm_mm": math.hypot(14, 5)}}
    errors = endpoint_errors(result, axis)
    for leg in ("outbound", "inbound"):
        assert errors[leg]["wheel_goal"]["shortfall_mm"] == pytest.approx(4.)
        assert errors[leg]["icp_goal"]["shortfall_mm"] == pytest.approx(-10.)
        assert errors[leg]["icp_goal"]["cross_track_mm"] == pytest.approx(8.)
        assert errors[leg]["odom_minus_icp"]["along_mm"] == pytest.approx(-14.)
        assert errors[leg]["odom_minus_icp"]["cross_mm"] == pytest.approx(-5.)
    rejected = deepcopy(result)
    rejected["outbound"].update(is_valid=False, calibrated_base_ground_truth=None, one_way_error=None)
    invalid = endpoint_errors(rejected, axis)["outbound"]
    assert invalid["wheel_goal"] is not None
    assert invalid["icp_goal"] is None and invalid["odom_minus_icp"] is None


def test_diagonal_plot_single_column(tmp_path):
    # Aborted trials still have a valid named axis, but no fabricated trace.
    output = tmp_path / "response.png"
    plot_responses([{"axis": "diagonal_left", "result": None}], output)
    assert output.is_file()


def test_oneway_diagonal_comparison_has_data_axes(tmp_path, monkeypatch):
    import run_oneway_eval as runner
    saved = []
    close = runner.plt.close
    monkeypatch.setattr(runner.plt, "close", lambda fig: saved.append(fig))
    leg = {"one_way_error": {"error_norm_mm": 12., "error_rate_pct": 2.4}}
    result = {"is_valid": True, "speed": .2, "outbound": leg, "inbound": leg}
    runner.generate_oneway_comparison_plots({"diagonal_left": [result]}, tmp_path / "oneway.png")
    figure = saved[-1]
    assert len(figure.axes) == 6
    assert "DIAGONAL_LEFT" in figure.axes[4].get_title()
    assert list(figure.axes[4].lines[0].get_ydata()) == [12.]
    close(figure)
