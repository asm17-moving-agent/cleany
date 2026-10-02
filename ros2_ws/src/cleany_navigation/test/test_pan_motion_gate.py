import math
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
