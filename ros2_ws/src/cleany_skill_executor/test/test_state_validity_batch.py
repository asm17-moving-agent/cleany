from types import SimpleNamespace as NS

import pytest
from cleany_interfaces.srv import CheckStateValidityBatch
from moveit_msgs.msg import RobotState, ContactInformation
from cleany_skill_executor.core.physical_grasp_selection import InfrastructureError
from cleany_skill_executor.state_validity_batch import StateValidityBatch
from cleany_skill_executor.hardware_execution import HardwareExecution


def test_hardware_batches_all_samples_and_preserves_first_failure_contacts():
    calls = []
    checks = []
    def call(client, request, **kwargs):
        calls.append(request)
        assert kwargs['monitor']
        if len(calls) == 1:
            return CheckStateValidityBatch.Response(success=True, valid=[True]*32, first_invalid_index=-1)
        return CheckStateValidityBatch.Response(success=True, valid=[True, False], first_invalid_index=1,
            contacts=[ContactInformation(contact_body_1='payload', contact_body_2='obstacle')])
    batch = StateValidityBatch(object(), client=NS(service_is_ready=lambda: True))
    hardware = NS(_batch_validity=batch, monitor=lambda: checks.append(True), node=NS(call=call))
    states = [RobotState(is_diff=True) for _ in range(65)]
    results = list(HardwareExecution._collision_responses(hardware, states))
    assert [len(request.robot_states) for request in calls] == [32, 32]
    assert [index for index, _ in results] == list(range(34))
    assert not results[-1][1].valid
    assert results[-1][1].contacts[0].contact_body_1 == 'payload'
    assert len(checks) == 4


def test_missing_batch_service_keeps_native_full_state_checks():
    native = []
    def call(client, request):
        native.append(request.robot_state)
        return NS(valid=True)
    hardware = NS(_batch_validity=StateValidityBatch(object(),
        client=NS(service_is_ready=lambda: False)), monitor=lambda: None,
        validity=object(), node=NS(call=call))
    states = [RobotState(is_diff=True) for _ in range(35)]
    assert len(list(HardwareExecution._collision_responses(hardware, states))) == len(states)
    assert native == states


@pytest.mark.parametrize('response', [
    CheckStateValidityBatch.Response(success=False, message='invalid state'),
    CheckStateValidityBatch.Response(success=True, valid=[True], first_invalid_index=-1),
    CheckStateValidityBatch.Response(success=True, valid=[True, True], first_invalid_index=1),
    CheckStateValidityBatch.Response(success=True, valid=[False, True], first_invalid_index=0),
    CheckStateValidityBatch.Response(success=True, valid=[True, True], first_invalid_index=-2),
])
def test_incomplete_or_malformed_responses_never_authorize_motion(response):
    batch = StateValidityBatch(object(), client=NS(service_is_ready=lambda: True))
    with pytest.raises(InfrastructureError):
        batch.check([RobotState(), RobotState()], lambda *_: response)
