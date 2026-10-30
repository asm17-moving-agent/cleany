from types import SimpleNamespace as NS
import pytest
from cleany_skill_executor.core.hardware_execution import JointRangeValidationError, PhysicalJointLimit, bounded_path
import cleany_skill_executor.hardware_execution as transport


def test_range_failure_preserves_joint_sample_and_encoder_bounds():
    limit = PhysicalJointLimit(-2., 2., 1., 2., 2048., 1., 0., 1900., 2200.)
    with pytest.raises(JointRangeValidationError) as rejected:
        bounded_path(('yaw',), [(0.,), (1.,)], {'yaw': limit}, velocity=.8, acceleration=1.2)
    error = rejected.value
    assert error.joint_name == 'yaw' and error.sample_index > 0
    assert error.lower <= error.value <= error.upper and error.raw > error.raw_max
    assert not error.motion_started and 'joint=yaw' in str(error)


def test_interrupted_replay_never_rebuilds_or_resumes_a_different_path():
    from cleany_skill_executor.core.hardware_execution import CommunicationRecovered
    calls = []
    def interrupted(*args, **kwargs):
        calls.append('execute')
        error = CommunicationRecovered(1)
        error.cancellation_confirmed = True
        raise error
    fake = NS(_execute=interrupted, positive_integer=lambda _: 1,
              fail_stop=lambda: calls.append('stop'))
    with pytest.raises(RuntimeError, match='cannot resume its exact path'):
        transport.HardwareExecution._execute_with_recovery(
            fake, 'right', object(), 'arm', recorded_trajectory=object())
    assert calls == ['execute', 'stop']
