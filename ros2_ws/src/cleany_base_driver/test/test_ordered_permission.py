from types import SimpleNamespace
from cleany_base_interfaces.srv import SetBaseEnabled
from cleany_base_driver.driver_node import BaseDriver


def test_late_enable_cannot_overtake_a_newer_stop():
    calls = []

    def enable(request, response):
        calls.append(request.data)
        response.success = True
        response.message = "sent, not acknowledged"
        return response

    driver = SimpleNamespace(
        permission_requests={}, command_sequence=21, boot_id=7, on_enable=enable
    )
    stop = SetBaseEnabled.Request(client_id="phone", request_sequence=5, enabled=False)
    result = BaseDriver.on_ordered_enable(driver, stop, SetBaseEnabled.Response())
    assert result.accepted and result.command_sequence == 21 and result.boot_id == 7
    late = SetBaseEnabled.Request(client_id="phone", request_sequence=4, enabled=True)
    assert not BaseDriver.on_ordered_enable(
        driver, late, SetBaseEnabled.Response()
    ).accepted
    assert calls == [False]


def test_retry_with_same_sequence_is_rejected_without_rearming():
    calls = []

    def enable(request, response):
        calls.append(request.data)
        response.success = True
        return response

    driver = SimpleNamespace(
        permission_requests={}, command_sequence=3, boot_id=9, on_enable=enable
    )
    request = SetBaseEnabled.Request(
        client_id="phone", request_sequence=1, enabled=True
    )
    assert BaseDriver.on_ordered_enable(
        driver, request, SetBaseEnabled.Response()
    ).accepted
    assert not BaseDriver.on_ordered_enable(
        driver, request, SetBaseEnabled.Response()
    ).accepted
    assert calls == [True]
