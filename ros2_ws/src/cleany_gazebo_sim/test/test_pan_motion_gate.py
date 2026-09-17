import math
from cleany_gazebo_sim.pan_motion_gate import PanGate


def test_side_motion_waits_for_stop_pan_settle_and_new_depth():
    gate = PanGate()
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
    gate=PanGate()
    args=((0.,0.,0.),0.,0.,1.,1.,1.)
    assert gate.step(1.,(-.1,0.,0.),*args)[2]=='REVERSE_NOT_ENABLED'
    assert gate.step(1.,(.1,.1,0.),*args)[2]=='MIXED_AXIS'
    assert gate.step(2.,(.1,0.,0.),*args)[2]=='STALE_FEEDBACK'
    gate.allow_reverse=True
    assert gate.step(1.,(-.1,0.,0.),*args)[0]==math.pi
