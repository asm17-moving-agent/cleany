import math

import pytest

from evaluator_core import Pose2D
from motion_control import MotionConfig, MotionController
from motion_patterns import motion_vectors


def command(vx, vy, config, yaw=0.0, rotation=False):
    controller = MotionController(
        Pose2D(0, 0, yaw), vx, vy, 1 if rotation else 0, 1, rotation, config
    )
    step = controller.step(Pose2D(0, 0, yaw), 0, 0, 0)
    return step.wz_rad_s if rotation else math.hypot(step.vx_mps, step.vy_mps)


@pytest.mark.parametrize("sign", [-1, 1])
@pytest.mark.parametrize("yaw", [0.0, 0.7])
def test_pure_axes_select_override_independent_of_sign_and_heading(sign, yaw):
    config = MotionConfig(linear_kp=0.2, linear_x_kp=0.4, linear_y_kp=0.8)
    assert command(sign, 0, config, yaw) == pytest.approx(0.4)
    assert command(0, sign, config, yaw) == pytest.approx(0.8)


def test_legacy_fallback_diagonal_and_rotation_gain():
    legacy = MotionConfig(linear_kp=0.3, yaw_kp=0.6)
    overrides = MotionConfig(
        linear_kp=0.3, linear_x_kp=0.9, linear_y_kp=1.2, yaw_kp=0.6
    )
    assert command(1, 0, legacy) == pytest.approx(command(1, 0, MotionConfig(linear_kp=0.3)))
    assert command(1, 1, overrides) == pytest.approx(command(1, 1, legacy))
    assert command(0, 0, overrides, rotation=True) == pytest.approx(0.6)


def test_none_defaults_and_dict_round_trip():
    config = MotionConfig()
    assert config.linear_x_kp is None and config.linear_y_kp is None
    assert MotionConfig(**config.to_dict()) == config


@pytest.mark.parametrize("pattern", [
    "surge", "sway", "rotation", "diagonal_left", "diagonal_right",
])
@pytest.mark.parametrize("leg", [0, 1], ids=["outbound", "inbound"])
@pytest.mark.parametrize("heading", [0.0, 0.7])
def test_current_evaluation_settings_apply_p6_in_every_direction(pattern, leg, heading):
    config = MotionConfig.from_json()
    assert (config.linear_kp, config.linear_x_kp, config.linear_y_kp, config.yaw_kp) == (6.,) * 4
    assert config.candidate_settings is True
    speed, target, error = .3, 1., .018
    vx, vy, wz = motion_vectors(pattern, speed)[leg]
    rotation = pattern == "rotation"
    start = Pose2D(0, 0, heading)
    controller = MotionController(start, vx, vy, wz, target, rotation, config)
    actual = target - error
    if rotation:
        pose = Pose2D(0, 0, heading + math.copysign(actual, wz))
    else:
        bx, by = vx / speed * actual, vy / speed * actual
        c, s = math.cos(heading), math.sin(heading)
        pose = Pose2D(c * bx - s * by, s * bx + c * by, heading)
    step = controller.step(pose, 0, 0, 0)
    # Outside the stop tolerance, with neither speed nor braking-envelope clipping:
    # assert the applied P, not just the serialized settings.
    scale = 6. * error / speed
    assert (step.vx_mps, step.vy_mps, step.wz_rad_s) == pytest.approx(
        (vx * scale, vy * scale, wz * scale)
    )
    assert step.error == pytest.approx(error)
    assert not step.done


@pytest.mark.parametrize("field", ["linear_x_kp", "linear_y_kp"])
@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), True, "1"])
def test_invalid_axis_gain_rejected(field, value):
    with pytest.raises(ValueError):
        MotionConfig(**{field: value})
