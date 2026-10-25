from cleany_manipulation_bt.drive_readiness import DriveGate


def samples(gate):
    for i in range(1, 62):
        gate.update(['arm'], [0.], [0.], i*10_000_000, i*.01)


def test_fresh_stowed_feedback_required():
    gate = DriveGate({'arm': 0.})
    assert not gate.ready(0, 0)
    samples(gate)
    assert gate.ready(.61, 610_000_000)
    assert not gate.ready(2., 610_000_000)
    assert not gate.ready(.61, 600_000_000)


def test_duplicate_stream_does_not_keep_gate_open():
    gate = DriveGate({'arm': 0.})
    samples(gate)
    gate.update(['arm'], [0.], [0.], 610_000_000, 2.)
    assert not gate.ready(2., 610_000_000)


def test_motion_missing_velocity_or_unstowed_position_revokes_readiness():
    for positions, velocities in (([.2], [0.]), ([0.], [.5]), ([0.], [])):
        gate = DriveGate({'arm': 0.})
        samples(gate)
        gate.update(['arm'], positions, velocities, 620_000_000, .62)
        assert not gate.ready(.62, 620_000_000)
