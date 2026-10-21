#!/usr/bin/env python3
"""Generate the MCU boot-calibration configuration from the robot JSON."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "configs/robot/imu_calibration.json"


def render(config: dict) -> str:
    for name in ("calibration_samples", "validation_samples"):
        n = config[name]
        if not isinstance(n, int) or isinstance(n, bool) or n < 2:
            raise ValueError(f"{name} must be an integer >= 2")
    for name in ("settling_s", "timeout_s", "noise_factor"):
        x = config[name]
        if isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x):
            raise ValueError(f"{name} must be finite")
    if (config["settling_s"] < 0 or config["timeout_s"] <= config["settling_s"]
            or config["noise_factor"] <= 0):
        raise ValueError("invalid calibration timing or noise factor")
    for name, count in (("fallback_bias_rad_s", 3), ("fallback_covariance_rad2_s2", 9)):
        values = config[name]
        if (not isinstance(values, list) or len(values) != count
                or any(isinstance(x, bool) or not isinstance(x, (int, float))
                       or not math.isfinite(x) for x in values)):
            raise ValueError(f"{name} must contain {count} finite numbers")
    a, b, c, d, e, f, g, h, i = config["fallback_covariance_rad2_s2"]
    if (not all(math.isclose(x, y, rel_tol=1e-9, abs_tol=1e-15)
                for x, y in ((b, d), (c, g), (f, h)))
            or a <= 0 or a * e - b * d <= 0
            or a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g) <= 0):
        raise ValueError("fallback covariance must be symmetric positive definite")
    bias = ", ".join(format(x, ".17g") for x in config["fallback_bias_rad_s"])
    cov = ", ".join(format(x, ".17g") for x in config["fallback_covariance_rad2_s2"])
    return f"""// Generated from configs/robot/imu_calibration.json; do not edit.
#pragma once
#include "imu_startup_calibration.hpp"
namespace cleany {{
inline ImuStartupConfig configuredImuStartup() {{
  ImuStartupConfig c{{}};
  c.calibrationSamples = {config['calibration_samples']};
  c.validationSamples = {config['validation_samples']};
  c.settlingUs = {round(config['settling_s'] * 1_000_000)}LL;
  c.timeoutUs = {round(config['timeout_s'] * 1_000_000)}LL;
  c.noiseFactor = {config['noise_factor']:.17g};
  c.fallbackBias = {{{{{bias}}}}};
  c.fallbackCovariance = {{{{{cov}}}}};
  return c;
}}
}}  // namespace cleany
"""


def generate(source: Path, output: Path) -> None:
    content = render(json.loads(source.read_text()))
    output.parent.mkdir(parents=True, exist_ok=True)
    if not output.exists() or output.read_text() != content:
        output.write_text(content)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output)
