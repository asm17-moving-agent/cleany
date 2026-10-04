import math
import pytest
from cleany_navigation.pan_motion_gate import PanGate


def test_side_motion_waits_for_stop_pan_settle_and_new_depth():
    gate = PanGate(allow_lateral=True)
    def step(t, pan, v=(0.,0.,0.), depth=None):
        return gate.step(t,(0.,.1,0.),v,pan,0.,t,t,t if depth is None else depth)
    assert step(1,0.,(.1,0.,0.))[2] == 'STOP_BEFORE_PAN'
    assert step(2,0.)[0] == math.pi/2
    assert step(3,math.pi/2)[2] == 'SETTLING'
    assert step(3.3,math.pi/2,depth=3.1)[2] == 'WAIT_FRESH_DEPTH'
    assert step(3.4,math.pi/2)[1] == (0.,.1,0.)
    assert step(3.5,math.pi/2,(0.,.1,0.))[2] == 'PASS'
    assert step(4.2,math.pi/2,depth=3.4)[1] == (0.,0.,0.)


def test_reverse_is_opt_in_mixed_and_stale_feedback_stop():
    gate=PanGate(allow_reverse=False)
    args=((0.,0.,0.),0.,0.,1.,1.,1.)
    assert gate.step(1.,(-.1,0.,0.),*args)[2]=='REVERSE_NOT_ENABLED'
    assert gate.step(1.,(.1,.1,0.),*args)[2]=='MIXED_AXIS'
    assert gate.step(2.,(.1,0.,0.),*args)[2]=='STALE_FEEDBACK'
    gate.allow_reverse=True
    assert gate.step(1.,(-.1,0.,0.),*args)[0]==math.pi


def test_default_lateral_is_blocked_and_reverse_waits_for_rear_depth():
    gate = PanGate()
    def step(t, cmd, pan=0., depth=None):
        return gate.step(t, cmd, (0., 0., 0.), pan, 0., t, t, t if depth is None else depth)
    assert step(1., (0., .1, 0.))[2] == 'LATERAL_NOT_ENABLED'
    assert step(2., (-.1, 0., 0.))[2] == 'TURNING_PAN'
    assert step(3., (-.1, 0., 0.), math.pi)[2] == 'SETTLING'
    assert step(3.3, (-.1, 0., 0.), math.pi, 3.1)[2] == 'WAIT_FRESH_DEPTH'
    assert step(3.4, (-.1, 0., 0.), math.pi)[1] == (-.1, 0., 0.)


def test_bounded_diagonal_keeps_front_camera_and_rejects_rotation_and_wide_angles():
    gate = PanGate(translation_angle_deg=10.)
    def step(t, cmd, pan=0., depth=None):
        return gate.step(t, cmd, (0., 0., 0.), pan, 0., t, t, t if depth is None else depth)
    cmd = (.1, .1*math.tan(math.radians(10)), 0.)
    assert step(1., cmd)[2] == 'SETTLING'
    assert step(1.3, cmd)[1] == cmd
    assert step(1.4, (.1, -.01, 0.))[2] == 'PASS'
    assert step(1.5, (.1, .03, 0.))[2] == 'MIXED_AXIS'
    assert step(1.6, (.1, .01, .1))[2] == 'MIXED_AXIS'
    assert step(1.7, (0., .01, 0.))[2] == 'LATERAL_NOT_ENABLED'
    assert step(2., (-.1, -.01, 0.))[2] == 'TURNING_PAN'
    assert step(3., (-.1, -.01, 0.), math.pi)[2] == 'SETTLING'
    assert step(3.3, (-.1, -.01, 0.), math.pi, 3.1)[2] == 'WAIT_FRESH_DEPTH'
    assert step(3.4, (-.1, -.01, 0.), math.pi)[2] == 'PASS'


def test_holonomic_camera_tracks_bearing_without_stopping_for_small_corrections():
    gate = PanGate(allow_lateral=True, allow_mixed_motion=True)
    command = (.1, .1, .1)
    def step(t, cmd, pan, measured=(0., 0., 0.), depth=None, pan_speed=0.):
        return gate.step(t, cmd, measured, pan, pan_speed, t, t, t if depth is None else depth)
    assert step(1., command, 0.)[2] == 'TURNING_PAN'
    assert gate.target == math.pi/4
    assert step(2., command, math.pi/4, command, depth=1.9, pan_speed=.4)[2] == 'WAIT_FRESH_DEPTH'
    assert step(2.05, command, math.pi/4, command, pan_speed=.4)[1] == command
    corrected = (.1, .102, .1)
    assert step(2.1, corrected, math.pi/4, command, pan_speed=.4)[1] == corrected
    assert gate.target == math.atan2(corrected[1], corrected[0])
    assert step(2.2, corrected, math.pi/4, command, depth=1.9)[2] == 'WAIT_FRESH_DEPTH'
    assert step(2.6, (0., -.1, .1), math.pi/4, corrected)[1] == (0., 0., 0.)
    assert step(3., (0., -.1, .1), -math.pi/2, corrected, depth=2.9)[2] == 'WAIT_FRESH_DEPTH'
    assert step(3.05, (0., -.1, .1), -math.pi/2, corrected, pan_speed=.4)[2] == 'PASS'


def test_rear_bearing_uses_short_equivalent_position_within_actual_joint_limits():
    gate = PanGate(allow_mixed_motion=True, allow_lateral=True,
                   pan_min_position_rad=-3.2, pan_max_position_rad=3.2)
    command = (-.1, -.002, 0.)
    def step(t, pan, depth):
        return gate.step(t, command, (0., 0., 0.), pan, 0., t, t, depth)
    target, output, reason = step(1., math.pi, .9)
    assert math.pi < target < 3.2
    assert target == pytest.approx(math.atan2(command[1], command[0])+2*math.pi)
    assert reason == 'WAIT_FRESH_DEPTH' and output == (0., 0., 0.)
    assert step(1.1, math.pi, 1.1)[1] == command
    # An actual 0.1 rad pointing error still stops; choosing a representation
    # cannot authorize stale data or relax the alignment tolerance.
    assert step(1.2, target-.1, 1.2)[2] == 'TURNING_PAN'
    assert step(1.3, target, 1.2)[2] == 'WAIT_FRESH_DEPTH'
    assert step(1.4, target, 1.4)[2] == 'PASS'
    # The equivalent positive target is outside the finite mechanical range.
    # A real long rotation is then necessary; never wrap through the stop.
    wide = (-.1, -.02, 0.)
    target, output, reason = gate.step(2., wide, (0., 0., 0.), math.pi, 0., 2., 2., 2.)
    assert target == math.atan2(wide[1], wide[0])
    assert -3.2 <= target <= 3.2 and reason == 'TURNING_PAN'
    assert output == (0., 0., 0.)


def test_rear_target_keeps_negative_position_branch_and_unreachable_bearing_stops():
    gate = PanGate(allow_mixed_motion=True, allow_lateral=True,
                   pan_min_position_rad=-3.2, pan_max_position_rad=3.2)
    command = (-.1, .002, 0.)
    target, _, _ = gate.step(1., command, (0., 0., 0.), -math.pi, 0., 1., 1., 1.)
    assert -3.2 < target < -math.pi
    assert target == pytest.approx(math.atan2(command[1], command[0])-2*math.pi)
    gate = PanGate(allow_mixed_motion=True, pan_min_position_rad=-1., pan_max_position_rad=1.)
    target, output, reason = gate.step(1., (-.1, 0., 0.), (0., 0., 0.), 0., 0., 1., 1., 1.)
    assert target == 0. and output == (0., 0., 0.) and reason == 'PAN_TARGET_OUT_OF_RANGE'


@pytest.mark.parametrize('limits', [(1., 2.), (2., -2.), (0., 0.), (-math.inf, 3.2), (-3.2, math.nan)])
def test_invalid_pan_position_limits_are_rejected(limits):
    with pytest.raises(ValueError):
        PanGate(pan_min_position_rad=limits[0], pan_max_position_rad=limits[1])
