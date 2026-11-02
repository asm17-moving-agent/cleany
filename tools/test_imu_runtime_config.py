import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location("imu_runtime_config", Path(__file__).with_name("imu_runtime_config.py"))
config_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config_tool)


@pytest.fixture
def calibration_config():
    return {
        "settling_s": .2,
        "calibration_samples": 12,
        "validation_samples": 8,
        "timeout_s": 1.,
        "noise_factor": 3.5,
        "fallback_bias_rad_s": [.01, -.02, .03],
        "fallback_covariance_rad2_s2": [
            1e-6, 1e-7, 0., 1e-7, 2e-6, 0., 0., 0., 3e-6,
        ],
    }


def _assert_header_matches(config, header):
    assert f"c.calibrationSamples = {config['calibration_samples']};" in header
    assert f"c.validationSamples = {config['validation_samples']};" in header
    assert f"c.settlingUs = {round(config['settling_s'] * 1_000_000)}LL;" in header
    assert f"c.timeoutUs = {round(config['timeout_s'] * 1_000_000)}LL;" in header
    assert f"c.noiseFactor = {config['noise_factor']:.17g};" in header
    for field, key in (("fallbackBias", "fallback_bias_rad_s"),
                       ("fallbackCovariance", "fallback_covariance_rad2_s2")):
        values = ", ".join(format(value, ".17g") for value in config[key])
        assert f"c.{field} = " + "{{" + values + "}};" in header


def test_config_generation_and_incremental_refresh(tmp_path, calibration_config):
    source = tmp_path / "config.json"
    config = calibration_config
    source.write_text(json.dumps(config))
    output = tmp_path / "generated/imu_startup_config.hpp"
    config_tool.generate(source, output)
    original = output.read_text()
    _assert_header_matches(config, original)
    timestamp = output.stat().st_mtime_ns
    config_tool.generate(source, output)
    assert output.stat().st_mtime_ns == timestamp
    config["calibration_samples"] += 1
    config["fallback_bias_rad_s"][2] = .035
    config["fallback_covariance_rad2_s2"][0] = 4e-6
    source.write_text(json.dumps(config))
    config_tool.generate(source, output)
    _assert_header_matches(config, output.read_text())
    assert output.read_text() != original


@pytest.mark.parametrize("change", [
    {"calibration_samples": 1},
    {"timeout_s": .1},
    {"noise_factor": -1.},
    {"fallback_bias_rad_s": [0., 0.]},
    {"fallback_covariance_rad2_s2": [0.] * 9},
])
def test_invalid_config_is_not_compiled(change, calibration_config):
    config = calibration_config
    config.update(change)
    with pytest.raises(ValueError):
        config_tool.render(config)


def test_robot_config_can_be_compiled():
    config_tool.render(json.loads(config_tool.SOURCE.read_text()))
