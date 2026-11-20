"""Base standstill diagnostics never substitute for STOP acknowledgement."""
from types import SimpleNamespace

import pytest

from cleany_base_interfaces.msg import WheelState
from cleany_base_driver.driver_node import BaseDriver


def driver():
    return SimpleNamespace(latest=WheelState(control_mode=WheelState.STOPPED),
                           enabled=False, pending_enable=None, disarmed_velocity_rad_s=.02)


def test_disabled_zero_output_and_measured_standstill_are_required():
    value = driver()
    assert BaseDriver._disarmed(value, .01)
    for age in (-.1, .251, float('nan'), float('inf')):
        assert not BaseDriver._disarmed(value, age)
    value.latest = None
    assert not BaseDriver._disarmed(value, .01)


@pytest.mark.parametrize('field,value', [
    ('enabled', True), ('fault_bits', 2), ('control_mode', WheelState.VELOCITY),
    ('target_rad_s', [0., 0., .1, 0.]), ('commanded_rad_s', [.1, 0., 0., 0.]),
    ('pwm_percent', [0, 0, 0, 1]), ('velocity_rad_s', [0., -.021, 0., 0.]),
    ('velocity_rad_s', [float('nan'), 0., 0., 0.]),
])
def test_mcu_motion_or_fault_is_not_disarmed(field, value):
    instance = driver()
    setattr(instance.latest, field, value)
    assert not BaseDriver._disarmed(instance, .01)


def test_host_permission_and_pending_enable_disqualify_standstill():
    instance = driver()
    instance.enabled = True
    assert not BaseDriver._disarmed(instance, .01)
    instance.enabled, instance.pending_enable = False, (4, 999)
    assert not BaseDriver._disarmed(instance, .01)


def test_watchdog_stop_can_be_disarmed_without_resetting_or_enabling_mcu():
    instance = driver()
    instance.latest.fault_bits = WheelState.FAULT_COMMAND_TIMEOUT
    instance.latest.control_mode = WheelState.TIMEOUT_STOP
    assert BaseDriver._disarmed(instance, .01)
    assert instance.latest.fault_bits == WheelState.FAULT_COMMAND_TIMEOUT
