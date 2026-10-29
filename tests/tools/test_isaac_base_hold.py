import importlib.util
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('base_hold', Path(__file__).resolve().parents[2]/
                                            'tools/validation/isaac/base_hold.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
StationaryBaseHold = module.StationaryBaseHold


def test_brake_rejects_invalid_gains_and_does_not_change_driving_commands():
    for value in (0., -1., math.nan, math.inf):
        with pytest.raises(ValueError):
            StationaryBaseHold(value, 5.)
    hold = StationaryBaseHold(5., 5.)
    hold.command(0., 0., 0., (0., 0., 0.))
    assert hold.command(.2, .1, .3, (.1, -.02, .2)) == (.1, -.02, .2)
    assert hold.command(.3, .2, .4, (0., 0., 0.)) == (0., 0., 0.)


def test_brake_uses_shortest_angle_and_respects_existing_actuator_limits():
    hold = StationaryBaseHold(5., 5.)
    hold.command(0., 0., math.pi-.0005, (0., 0., 0.))
    velocity = hold.command(1., -1., -math.pi+.0005, (0., 0., 0.))
    assert max(abs(v) for v in velocity[:2]) <= .25
    assert velocity[2] == pytest.approx(-.005)


def test_brake_bounds_accumulated_arm_reaction_drift_without_pose_resets():
    hold = StationaryBaseHold(5., 5.)
    yaw = 0.
    for _ in range(20000):
        _, _, velocity = hold.command(0., 0., yaw, (0., 0., 0.))
        yaw += (velocity+.001)*.001
    assert 0. < yaw <= .000201
    assert hold.report()['samples'] == 20000
