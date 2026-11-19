#!/usr/bin/env python3
"""Tests for Wheel Tracking Diagnosis and Gain Recommendation Logic."""

import pytest
import re
from pathlib import Path
from diagnose_wheel_tracking import (
    CURRENT_CONFIGS,
    WheelSample,
    analyze_samples,
    load_baseline,
    format_table,
    split_direction_samples,
    has_complete_directional_results,
)


def test_perfect_tracking_synthetic_input_no_gain_increase():
    """Verify that a robot perfectly tracking slew-filtered commands does NOT trigger false gain increases.

    Prevents the bug where normal acceleration slew lag was misclassified as steady-state tracking error,
    falsely recommending a 25.7% Kp increase.
    """
    samples: list[WheelSample] = []
    target = 6.0  # 6.0 rad/s step target

    # 100 samples (2.0s at 50Hz)
    # First 25 samples (0.5s): ramp up from 0 to 6.0 rad/s (slew phase)
    # Remaining 75 samples (1.5s): steady-state at 6.0 rad/s
    for i in range(100):
        t = i * 0.02
        if i < 25:
            cmd = target * (i / 25.0)  # ramping
        else:
            cmd = target  # steady state

        # Actual velocity perfectly tracks commanded_rad_s!
        vel = cmd
        # PWM proportional to speed (FF scale = 0.16)
        pwm = int((vel / 0.16) * 0.01 * 100)

        samples.append(WheelSample(
            time_s=t,
            target_rad_s=[target] * 4,
            commanded_rad_s=[cmd] * 4,
            velocity_rad_s=[vel] * 4,
            pwm_percent=[pwm] * 4,
        ))

    stats = analyze_samples(samples)

    assert len(stats) == 4
    for s in stats:
        assert s.has_sufficient_steady_state
        assert s.status == "VALID"
        # Steady-state error against commanded must be zero!
        assert s.steady_error_rmse < 0.001
        assert s.steady_rel_err_pct < 0.01
        # Transient slew delay must be captured (~500ms)
        assert abs(s.transient_delay_ms - 500.0) < 50.0

        # CRITICAL: Suggested Kp and Ki must NOT increase! Must be identical to current configs!
        cfg = CURRENT_CONFIGS[s.name]
        assert s.suggested_kp == pytest.approx(cfg["kp"], abs=1e-4)
        assert s.suggested_ki == pytest.approx(cfg["ki"], abs=1e-4)


def test_insufficient_steady_state_samples_rejects_gain_change():
    """Verify that insufficient data (<15 settled samples) yields no speculative gain changes."""
    samples: list[WheelSample] = []
    # Only 5 samples
    for i in range(5):
        samples.append(WheelSample(
            time_s=i * 0.02,
            target_rad_s=[5.0] * 4,
            commanded_rad_s=[5.0] * 4,
            velocity_rad_s=[4.0] * 4,
            pwm_percent=[30] * 4,
        ))

    stats = analyze_samples(samples)
    assert len(stats) == 4
    for s in stats:
        assert not s.has_sufficient_steady_state
        assert s.status == "INSUFFICIENT_STEADY_STATE_DATA"
        assert s.ground_ff_scale_est is None
        assert s.suggested_kp is None
        assert s.suggested_ki is None
    assert "NO RECOMMENDATION" in format_table(stats)


def test_brief_matching_command_after_target_change_is_not_steady_state():
    samples = [
        WheelSample(i * 0.02, [5.0] * 4, [5.0] * 4,
                    [5.0] * 4, [30] * 4)
        for i in range(10)
    ]
    stats = analyze_samples(samples)
    assert all(not item.has_sufficient_steady_state for item in stats)
    assert all(item.suggested_kp is None for item in stats)


def test_transient_spans_sum_across_direction_target_runs():
    samples = []
    for i in range(100):
        t = i * 0.02
        target = 5.0 if i < 50 else -5.0
        run_start = 0 if i < 50 else 50
        command = target if i - run_start >= 20 else target * 0.5
        samples.append(WheelSample(t, [target] * 4, [command] * 4,
                                   [command] * 4, [30] * 4))
    stats = analyze_samples(samples)
    assert all(item.has_sufficient_steady_state for item in stats)
    assert all(item.transient_delay_ms == pytest.approx(800.0) for item in stats)
    split = split_direction_samples(samples)
    assert len(split["forward"]) == len(split["reverse"]) == 50


def test_combined_readiness_requires_each_direction_for_both_patterns():
    both_directions = [
        WheelSample(i * 0.02, [5.0] * 4, [5.0] * 4, [5.0] * 4, [30] * 4)
        for i in range(40)
    ] + [
        WheelSample(1.0 + i * 0.02, [-5.0] * 4, [-5.0] * 4,
                    [-5.0] * 4, [30] * 4)
        for i in range(40)
    ]
    per_direction = {
        pattern: {
            direction: analyze_samples(samples, target_sign=sign)
            for direction, sign in (("forward", 1), ("reverse", -1))
        }
        for pattern, samples in {
            "surge": both_directions,
            "sway": both_directions,
        }.items()
    }
    assert has_complete_directional_results(per_direction)
    del per_direction["sway"]["reverse"]
    assert not has_complete_directional_results(per_direction)


def test_tracking_lag_does_not_automatically_retune_pi():
    """A tracking ratio is insufficient evidence for changing closed-loop PI gains."""
    samples: list[WheelSample] = []
    target = 6.0

    for i in range(100):
        t = i * 0.02
        cmd = target if i >= 20 else target * (i / 20.0)
        # Real steady-state lag: motor runs at 90% of command due to friction
        vel = cmd * 0.90 if i >= 20 else cmd * 0.85
        pwm = 35

        samples.append(WheelSample(
            time_s=t,
            target_rad_s=[target] * 4,
            commanded_rad_s=[cmd] * 4,
            velocity_rad_s=[vel] * 4,
            pwm_percent=[pwm] * 4,
        ))

    stats = analyze_samples(samples)
    for s in stats:
        assert s.has_sufficient_steady_state
        assert s.steady_rel_err_pct > 5.0
        cfg = CURRENT_CONFIGS[s.name]
        assert s.suggested_kp == cfg["kp"]
        assert s.suggested_ki == cfg["ki"]


def test_default_baseline_matches_firmware_controller_configs():
    configs = load_baseline()
    source = (Path(__file__).resolve().parents[2] / "esp32/src/main.cpp").read_text()
    expected = {
        "FL": (11.9352, 6.058, 12.118),
        "FR": (12.2596, 6.030, 12.058),
        "RL": (11.7678, 6.246, 12.492),
        "RR": (11.3872, 6.154, 12.309),
    }
    for wheel, values in expected.items():
        assert tuple(configs[wheel][key] for key in ("ff_scale", "kp", "ki")) == values
        match = re.search(
            rf"k{wheel.title()}ControllerConfig\{{\s*([\d.]+)F,\s*([\d.]+)F,\s*([\d.]+)F",
            source,
        )
        assert match
        assert tuple(map(float, match.groups())) == values


def test_perfect_constant_command_has_no_slew_delay():
    samples = [WheelSample(i * .02, [5.] * 4, [5.] * 4, [5.] * 4, [30] * 4)
               for i in range(100)]
    assert all(s.transient_delay_ms == 0 for s in analyze_samples(samples))


def test_opposite_velocity_sign_is_not_perfect_tracking():
    samples = [WheelSample(i * .02, [5.] * 4, [5.] * 4, [-5.] * 4, [30] * 4)
               for i in range(100)]
    assert all(s.steady_error_rmse == pytest.approx(10) for s in analyze_samples(samples))


def test_interrupted_settling_is_not_merged_into_valid_window():
    samples = [WheelSample(i * .02, [5.] * 4, [4. if i % 10 == 0 else 5.] * 4,
                           [5.] * 4, [30] * 4) for i in range(100)]
    assert all(not s.has_sufficient_steady_state for s in analyze_samples(samples))
