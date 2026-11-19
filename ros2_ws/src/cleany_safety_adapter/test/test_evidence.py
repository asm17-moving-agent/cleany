from cleany_safety_adapter.evidence import SafetyEvidence


def evidence(**kwargs):
    values = dict(timeout_s=.3, linear_mps=.01, angular_radps=.02, joint_radps=.02,
                  monitored_joints=['arm_joint'], estop_configured=True, protection_configured=True)
    values.update(kwargs)
    return SafetyEvidence(**values)


def feed(e, now, *, released=True, ready=True, base=0., joint=0.):
    e.estop_released(released, now)
    e.protection_ready(ready, now)
    e.odometry(base, 0., 0., now)
    e.joints(['arm_joint'], [joint], now)


def test_unconfigured_hardware_is_never_reported_safe():
    e = evidence(estop_configured=False, protection_configured=False)
    feed(e, 1.)
    report = e.evaluate(1.)
    assert not report.protection_ready and report.estop_engaged and not report.physical_release_confirmed
    assert report.stopped  # Motion evidence is real even without a protection layer.
    assert 'no e-stop circuit evidence' in report.detail


def test_fresh_evidence_is_safe_and_stale_evidence_is_not():
    e = evidence()
    feed(e, 1.)
    report = e.evaluate(1.1)
    assert report.protection_ready and report.physical_release_confirmed and report.stopped
    report = e.evaluate(1.5)
    assert not report.protection_ready and report.estop_engaged and not report.stopped


def test_engaged_estop_motion_and_missing_velocity():
    e = evidence()
    feed(e, 1., released=False, base=.2)
    report = e.evaluate(1.)
    assert report.estop_engaged and not report.stopped and 'base moving' in report.detail
    e.joints(['arm_joint'], [], 1.)
    feed(e, 2.)
    e.joints(['arm_joint'], [], 2.)
    assert not e.evaluate(2.).stopped


def test_adapter_stays_silent_until_configured_hardware_reports_once():
    e = evidence()
    assert e.awaiting_first_evidence()
    e.estop_released(False, 1.)
    assert e.awaiting_first_evidence()
    e.protection_ready(False, 1.)
    assert not e.awaiting_first_evidence()
    assert not evidence(estop_configured=False, protection_configured=False).awaiting_first_evidence()
