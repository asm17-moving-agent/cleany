import math

import pytest

from evaluator_core import Pose2D
from motion_control import MotionConfig, MotionController


def ctl(rotation=False, target=.55, vx=1.0, wz=1.0):
    return MotionController(Pose2D(0, 0, .4), 0 if rotation else vx, 0, wz if rotation else 0,
                            target, rotation, MotionConfig())


def test_translation_projection_overshoot_and_hold():
    c = ctl(target=.5)
    direction = (math.cos(.4), math.sin(.4))
    at = lambda distance: Pose2D(distance * direction[0], distance * direction[1], .4)
    step = c.step(at(.498), 0, 0, 0)
    assert not step.done
    assert step.vx_mps == pytest.approx(0)
    over = c.step(at(.55), 0, 0, .1)
    assert not over.done and over.overshoot == pytest.approx(.05)
    assert not c.step(at(.5), 0, 0, .7).done
    assert c.step(at(.5), 0, 0, 1.2).done


@pytest.mark.parametrize("direction", [1, -1])
def test_yaw_accumulates_full_turn(direction):
    c = ctl(True, 2 * math.pi, wz=direction)
    result = None
    for i in range(1, 9):
        yaw = .4 + direction * i * math.pi / 4
        wrapped = (yaw + math.pi) % (2 * math.pi) - math.pi
        result = c.step(Pose2D(0, 0, wrapped), 0, 0, i * .1)
    assert result.actual_displacement == pytest.approx(2 * math.pi)
    assert not result.done


def test_backward_sway_diagonal_and_heading_transform():
    for vx, vy in [(-1, 0), (0, 1), (1, 1)]:
        c = MotionController(Pose2D(0, 0, math.pi / 2), vx, vy, 0, .7, False, MotionConfig())
        ux, uy = -vy / math.hypot(vx, vy), vx / math.hypot(vx, vy)
        distance = .5
        result = c.step(Pose2D(ux * distance, uy * distance, math.pi / 2), 0, 0, 0)
        assert result.actual_displacement == pytest.approx(distance)
        assert math.isfinite(result.vx_mps + result.vy_mps)


@pytest.mark.parametrize("args", [
    (Pose2D(0, 0, 0), 0, 0, 1, .5, False),
    (Pose2D(0, 0, 0), 1, 0, 0, 0, False),
])
def test_reject_invalid_goals(args):
    with pytest.raises(ValueError):
        MotionController(*args)


def test_config_rejects_bad_values():
    with pytest.raises(ValueError):
        MotionConfig(linear_kp=float("nan"))
