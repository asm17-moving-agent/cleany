from collections import defaultdict
from copy import deepcopy
import json

import pytest

from motion_control import MotionConfig
import run_motion_response as response
from run_wheel_only_eval import load_config as load_evaluation_config


CONFIG = response.ROOT / "configs/mission/all_axis_validation.json"


def test_all_axis_validation_matrix_uses_p6_and_requires_external_comparison():
    config = response.load_config(CONFIG)
    speeds = defaultdict(list)
    for trial in config["trials"]:
        assert trial["kp"] == 6.0
        speeds[trial["axis"]].append(trial["speed"])
    assert set(speeds) == {"x", "y", "yaw", "diagonal_left", "diagonal_right"}
    for axis, values in speeds.items():
        assert values == ([.3, .6, 1.] if axis == "yaw" else [.2, .35, .5])
    assert [trial["axis"] for trial in config["trials"][:5]] == [
        "x", "y", "yaw", "diagonal_left", "diagonal_right",
    ]
    assert config["require_valid_icp"] is True
    assert config["icp_min_inlier_ratio"] == .30
    assert config["icp_convergence"]["translation_tolerance_m"] == 1e-5
    assert config["icp_convergence"]["rotation_tolerance_rad"] == 1e-5
    assert config["icp_convergence"]["max_translation_span_m"] == .001
    assert config["repetitions"] == 3
    assert len(config["trials"]) * config["repetitions"] == 45
    assert config["translation_distance_m"] == .5
    assert config["rotation_angle_deg"] == 90.


def test_all_axis_dry_run_from_other_directory_has_no_runtime_or_output(
    tmp_path, monkeypatch, capsys,
):
    config = response.load_config(CONFIG)
    config["output_dir"] = str(tmp_path / "not_created")
    source = tmp_path / "validation.json"
    source.write_text(json.dumps(config))
    monkeypatch.chdir(tmp_path)

    def no_run(*args, **kwargs):
        raise AssertionError("dry-run must not initialize the runtime")

    monkeypatch.setattr(response, "run", no_run)
    assert response.main(["--config", str(source), "--dry-run"]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["config"] == config
    assert plan["directional_legs"] == 90
    assert not (tmp_path / "not_created").exists()


def test_trial_parameter_tuning_is_ephemeral_and_preserves_other_settings(tmp_path):
    config = response.load_config(CONFIG)
    evaluation, _ = load_evaluation_config(response.ROOT / config["evaluation_config"])
    base_path = response.ROOT / evaluation["motion_config"]
    before = base_path.read_bytes()
    base = MotionConfig.from_json(base_path)
    changed = deepcopy(config)
    changed["repetitions"] = 1
    changed["trials"] = [{"axis": "diagonal_right", "kp": 4., "speed": .15}]
    source = tmp_path / "tuning.json"
    source.write_text(json.dumps(changed))

    trial = response.load_config(source)["trials"][0]
    candidate = response.candidate_config(base, trial["axis"], trial["kp"])
    expected = base.to_dict()
    expected["linear_kp"] = 4.
    assert candidate.to_dict() == expected
    assert base.linear_kp == 6.
    assert base_path.read_bytes() == before
    assert source.read_text() == json.dumps(changed)


@pytest.mark.parametrize("value", [0, -.1, 1.01, float("nan"), float("inf"), True, "0.3"])
def test_invalid_icp_inlier_ratio_rejected_before_runtime(tmp_path, value):
    config = response.load_config(CONFIG)
    config["icp_min_inlier_ratio"] = value
    source = tmp_path / "invalid.json"
    source.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="icp_min_inlier_ratio"):
        response.load_config(source)


@pytest.mark.parametrize("value", [
    [], None, {"unexpected": .1}, {"translation_tolerance_m": 0},
    {"rotation_tolerance_rad": float("nan")}, {"max_period": True},
    {"max_translation_span_m": 1e-6},
])
def test_invalid_convergence_config_rejected_before_runtime(tmp_path, value):
    config = response.load_config(CONFIG)
    config["icp_convergence"] = value
    source = tmp_path / "invalid.json"
    source.write_text(json.dumps(config))
    with pytest.raises(ValueError, match="icp_convergence"):
        response.load_config(source)
