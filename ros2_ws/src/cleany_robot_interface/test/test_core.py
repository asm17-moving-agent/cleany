import copy
import math
from pathlib import Path

import pytest
import yaml

from cleany_robot_interface.core import (
    ENCODER_HALF_TICK_RAD, JOINTS, JointMapping, assess_joint_limits, mappings,
)


def fixture():
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/arm_preview.yaml').read_text())
    cal = {side: {key: dict(id=first + i, drive_mode=0, homing_offset=123,
                           range_min=500, range_max=3500)
                  for i, key in enumerate(JOINTS)}
           for side, first in [('left', 1), ('right', 7)]}
    return config, cal


def test_sides_and_angles():
    config, cal = fixture()
    joints = mappings(config, cal)
    assert [j.motor_id for j in joints] == list(range(1, 13))
    assert joints[0].name == 'left_shoulder_yaw_joint'
    assert joints[6].name == 'right_shoulder_yaw_joint'
    assert joints[0].radians(2044) == 0  # EEPROM offset not applied twice
    for index, zero in ((0, 2044), (6, 2043)):
        assert joints[index].radians(zero) == 0
        assert joints[index].radians(zero + 1024) == pytest.approx(-math.pi / 2)
        assert joints[index].radians(zero - 1024) == pytest.approx(math.pi / 2)
    assert joints[5].radians(1345) == pytest.approx(-0.37453297762778586)
    assert joints[11].radians(1359) == pytest.approx(-0.37453297762778586)


def test_closed_jaw_overrides_and_range_min_fallback():
    config, cal = fixture()
    joints = mappings(config, cal)
    # Both closure references were measured again after reassembly on 2026-10-05/06.
    for index, closed, direction in ((5, 1345, 1), (11, 1359, 1)):
        assert joints[index].radians(closed + 256) - joints[index].radians(closed) == pytest.approx(direction * math.pi / 8)
    config['overrides'] = {}
    joints = mappings(config, cal)
    assert joints[5].radians(500) == pytest.approx(-0.37453297762778586)
    assert joints[11].radians(500) == pytest.approx(-0.37453297762778586)


def test_override():
    config, cal = fixture()
    config['overrides'] = {'right': {'shoulder_pan': {'direction': 1}}}
    assert mappings(config, cal)[6].radians(3072) == pytest.approx(math.pi / 2)
    assert mappings(config, cal)[0].radians(3072) == pytest.approx(-math.pi / 2)


def test_shoulder_lifts_reverse_about_their_reference_poses():
    config, cal = fixture()
    joints = {joint.motor_id: joint for joint in mappings(config, cal)}
    assert joints[2].radians(3104) == pytest.approx(0)
    assert joints[2].radians(2080) == pytest.approx(math.pi / 2)
    assert joints[8].radians(3108) == pytest.approx(0)
    assert joints[8].radians(2084) == pytest.approx(math.pi / 2)
    assert joints[8].radians(1060) == pytest.approx(math.pi)


def test_both_wrist_recentered_references_and_directions():
    config, cal = fixture()
    joints = {joint.motor_id: joint for joint in mappings(config, cal)}
    assert joints[5].radians(2513) == pytest.approx(0)
    assert joints[11].radians(2467) == pytest.approx(0)
    assert joints[5].radians(2769) == pytest.approx(-math.pi / 8)
    assert joints[11].radians(2723) == pytest.approx(-math.pi / 8)


def test_right_elbow_folded_parallel_reference():
    config, cal = fixture()
    joint = mappings(config, cal)[8]
    # The operator aligned the upper/lower long sections in opposite directions.
    assert joint.radians(3100) == pytest.approx(math.pi)
    assert joint.radians(2076) == pytest.approx(math.pi / 2)
    assert joint.radians(1052) == pytest.approx(0)


@pytest.mark.parametrize('side,motor_id,shift,observed_ticks', [
    ('left', 5, 611, (1200, 1981, 3124, 4095, 4096, 4118)),
    ('right', 11, 674, (1251, 2140, 3141, 4095, 4096, 4193)),
])
def test_wrist_recenter_preserves_physical_angle_across_old_wrap(
    side, motor_id, shift, observed_ticks,
):
    config, cal = fixture()
    new = mappings(config, cal)
    old_config = copy.deepcopy(config)
    old_config['overrides'][side]['wrist_roll']['zero_tick'] = 3124 if side == 'left' else 3141
    old = mappings(old_config, cal)
    for continuous in observed_ticks:
        before = old[motor_id - 1].radians(continuous % 4096)
        after = new[motor_id - 1].radians(continuous - shift)
        assert math.sin(after) == pytest.approx(math.sin(before), abs=1e-12)
        assert math.cos(after) == pytest.approx(math.cos(before), abs=1e-12)
    assert [j for j in new if j.motor_id != motor_id] == [j for j in old if j.motor_id != motor_id]


@pytest.mark.parametrize('field,value', [('id', 7), ('drive_mode', 1), ('range_max', 499)])
def test_bad_calibration(field, value):
    config, cal = fixture()
    cal = copy.deepcopy(cal)
    cal['left']['shoulder_pan'][field] = value
    with pytest.raises(ValueError):
        mappings(config, cal)


def test_invalid_encoder():
    config, cal = fixture()
    with pytest.raises(ValueError):
        mappings(config, cal)[0].radians(65535)


@pytest.mark.parametrize('motor,raw,motor_bounds,model_bounds,status,excess_deg', [
    (2, 863, (855, 3256), (-0.22, 3.37), 'outside_model_limits', 3.876113666),
    (3, 3096, (908, 3102), (-0.22, 3.14), 'encoder_boundary', 0.003361704),
    (4, 825, (811, 3226), (-1.65806284946, 1.65806272933),
     'outside_model_limits', 8.974605934),
    (8, 858, (850, 3239), (-0.22, 3.37), 'outside_model_limits', 4.667129291),
])
def test_observed_motor_normal_model_excess(
    motor, raw, motor_bounds, model_bounds, status, excess_deg,
):
    config, cal = fixture()
    side, index = ('left', motor - 1) if motor <= 6 else ('right', motor - 7)
    cal[side][JOINTS[index]].update(range_min=motor_bounds[0], range_max=motor_bounds[1])
    joint = mappings(config, cal)[motor - 1]
    result = assess_joint_limits(joint, raw, model_bounds)
    assert result.within_motor_range
    assert not result.within_model_limits
    assert result.status == status
    assert math.degrees(result.model_limit_excess_rad) == pytest.approx(excess_deg, abs=1e-8)
    assert result.position_rad == joint.radians(raw)  # Never clamp the observed angle.


@pytest.mark.parametrize('raw,inside', [(500, True), (3500, True), (499, False), (3501, False)])
def test_motor_calibration_bounds_checked_independently(raw, inside):
    joint = JointMapping('test_joint', 1, 777, 1000, 1, 0, 0, 500, 3500)
    result = assess_joint_limits(joint, raw, (-math.tau, math.tau))
    assert result.within_motor_range == inside
    assert result.within_model_limits
    assert result.status == ('within_limits' if inside else 'outside_motor_limits')


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('half_ticks,status', [
    (0.0, 'within_limits'), (1.0, 'encoder_boundary'), (1.001, 'outside_model_limits'),
])
def test_encoder_boundary_label_preserves_strict_model_check(direction, half_ticks, status):
    joint = JointMapping('test_joint', 1, 777, 1000, 1, 0, 0, 500, 3500)
    excess = half_ticks * ENCODER_HALF_TICK_RAD
    bounds = (-1.0, -excess) if direction == 1 else (excess, 1.0)
    result = assess_joint_limits(joint, 1000, bounds)
    assert result.status == status
    assert result.position_rad == 0.0
    assert result.within_model_limits == (half_ticks == 0.0)
    assert result.model_limit_excess_rad == excess


@pytest.mark.parametrize('bounds', [(1.0, -1.0), (math.nan, 1.0), (-1.0, math.inf)])
def test_invalid_model_limits_rejected(bounds):
    joint = JointMapping('test_joint', 1, 777, 1000, 1, 0, 0, 500, 3500)
    with pytest.raises(ValueError, match='invalid model limits'):
        assess_joint_limits(joint, 1000, bounds)
