from copy import deepcopy
import math

import pytest

from evaluator_core import Pose2D
from motion_control import MotionConfig
from motion_response import response_metrics
from run_motion_response import ROOT, bound_violation, candidate_config, close_resources, load_config, main, trial_validity


def test_default_plan_and_dry_run(capsys):
    assert main(["--dry-run"]) == 0
    assert '"directional_legs": 90' in capsys.readouterr().out


def test_candidate_changes_one_axis_only():
    base = MotionConfig()
    assert candidate_config(base, "x", 3).linear_x_kp == 3
    assert candidate_config(base, "y", 2).linear_y_kp == 2
    assert candidate_config(base, "yaw", 4).yaw_kp == 4
    assert candidate_config(base, "x", 3).linear_deceleration_mps2 == base.linear_deceleration_mps2


def test_bound_checks_cover_both_signs_and_world_heading():
    guards = load_config(ROOT / "configs/mission/motion_response_tuning.json")["guards"]
    origin = Pose2D(0, 0, math.pi/2)
    assert not bound_violation(Pose2D(0, -.5, math.pi/2), origin, "x", .5, guards, origin)
    assert "cross-track" in bound_violation(Pose2D(.09, .2, math.pi/2), origin, "x", .5, guards, origin)
    assert "excursion" in bound_violation(Pose2D(0, -.53, math.pi/2), origin, "x", .5, guards, origin)
    assert "rotation" in bound_violation(Pose2D(0, 0, math.pi/2+math.radians(96)),
                                        origin, "yaw", math.pi/2, guards, origin)


def test_metrics_reject_invalid_and_use_position_and_speed_settling():
    assert response_metrics({"is_valid": False}, "x") == {"valid": False}
    result = {
        "is_valid": True, "reached_tolerance": True, "motion_config": MotionConfig().to_dict(),
        "target_displacement": .5, "final_error": .003, "max_overshoot": 0,
        "trace": [
            {"elapsed_s": t, "position_error": error, "linear_speed_mps": speed}
            for t, error, speed in [(0, .5, 0), (1, .4, .2), (2, .05, .05),
                                    (3, .003, .01), (3.2, .003, 0), (3.8, .003, 0)]
        ]}
    metrics = response_metrics(result, "x")
    assert metrics["settling_entry_s"] == 3.2
    assert metrics["rise_10_90_s"] == 1
    assert metrics["valid"]


def test_response_only_can_exclude_icp_but_never_failed_motion():
    result = {"is_valid": False, "control_stopping_performance": {
        "outbound": {"is_valid": True}, "inbound": {"is_valid": True}}}
    assert trial_validity(result, True) == (True, False, False)
    assert trial_validity(result, False) == (True, False, True)
    result["control_stopping_performance"]["inbound"]["is_valid"] = False
    assert trial_validity(result, False) == (False, False, False)
    assert trial_validity(None, False) == (False, False, False)


@pytest.mark.parametrize("stop_raises", [False, True])
def test_cleanup_attempts_all_resources_and_reports_unconfirmed_disable(stop_raises):
    from types import SimpleNamespace
    calls = []
    node = SimpleNamespace(aborted=False, last_failure="",
                           destroy_node=lambda: calls.append("destroy"))
    def stop():
        calls.append("stop")
        if stop_raises:
            raise RuntimeError("stop transport failed")
        node.aborted = True
        node.last_failure = "Base disable was not acknowledged"
    node.stop_robot = stop
    errors = close_resources(
        node, SimpleNamespace(close=lambda: calls.append("stack")),
        SimpleNamespace(close=lambda: calls.append("probe")),
        lambda: calls.append("shutdown"))
    assert calls == ["stop", "destroy", "shutdown", "stack", "probe"]
    assert errors
    assert ("transport" if stop_raises else "not acknowledged") in errors[0]


def test_cleanup_shuts_down_ros_after_node_construction_failure():
    calls = []
    assert close_resources(None, None, None, lambda: calls.append("shutdown")) == []
    assert calls == ["shutdown"]
