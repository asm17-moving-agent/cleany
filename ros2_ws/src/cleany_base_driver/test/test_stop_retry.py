"""Production driver, synthetic packet loss and MCU watchdog state; no physical motion."""
from threading import Thread
from time import monotonic, sleep

import pytest
import rclpy
from cleany_base_interfaces.msg import WheelCommand, WheelState
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from cleany_base_driver.driver_node import BaseDriver
from cleany_base_driver.mock_node import MockMcu


def wait(predicate, timeout=4.):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        if predicate():
            return
        sleep(.01)
    assert predicate()


class LossyMcu(MockMcu):
    def __init__(self):
        self.drop_stops, self.stops, self.enables = 0, [], 0
        super().__init__()

    def command(self, command):
        if command.mode == WheelCommand.STOP:
            self.stops.append(command.sequence)
            if self.drop_stops:
                self.drop_stops -= 1
                return
        if command.mode == WheelCommand.ENABLE:
            self.enables += 1
        super().command(command)


@pytest.fixture
def graph():
    rclpy.init(domain_id=181)
    driver = BaseDriver(parameter_overrides=[Parameter(k, value=v) for k, v in {
        'geometry.wheel_radius_m': .05, 'geometry.wheelbase_m': .30, 'geometry.wheel_separation_m': .30,
        'limits.linear_x_mps': .4, 'limits.linear_y_mps': .4, 'limits.angular_z_rad_s': .5,
        'limits.wheel_rad_s': 10., 'limits.command_timeout_s': .4}.items()])
    mcu, observer = LossyMcu(), rclpy.create_node('stop_retry_observer')
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (driver, mcu, observer):
        executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait(lambda: driver.latest is not None)
        yield driver, mcu, observer
    finally:
        executor.shutdown(timeout_sec=4.)
        thread.join(4.)
        for node in (driver, mcu, observer):
            node.destroy_node()
        rclpy.shutdown()


def test_lost_stop_packets_are_retried_with_same_sequence_without_enable(graph):
    driver, mcu, observer = graph
    mcu.drop_stops = 3
    client = observer.create_client(SetBool, 'base/enable')
    wait(client.service_is_ready)
    future = client.call_async(SetBool.Request(data=False))
    wait(future.done)
    assert future.result().success
    wait(lambda: len(mcu.stops) >= 4 and driver.latest.last_command_sequence == driver._stop_sequence)
    assert len(set(mcu.stops)) == 1
    assert not driver.enabled and not mcu.enabled and mcu.enables == 0
    count = len(mcu.stops)
    sleep(.15)
    assert len(mcu.stops) <= count + 1  # Retry stops after observed acknowledgement.
    first_sequence = driver._stop_sequence
    future = client.call_async(SetBool.Request(data=False))
    wait(future.done)
    wait(lambda: driver.latest.last_command_sequence == driver._stop_sequence)
    assert driver._stop_sequence != first_sequence


def test_watchdog_stop_does_not_need_an_enable_to_report_available(graph):
    driver, mcu, _ = graph
    mcu.faults = WheelState.FAULT_COMMAND_TIMEOUT
    wait(lambda: driver.diagnostic_error == 'MCU command timeout')
    wait(lambda: driver.latest.last_command_sequence == driver._stop_sequence)
    assert driver.fault_reason == '' and driver._disarmed(.01)
    assert not driver.enabled and not mcu.enabled and mcu.enables == 0
    assert mcu.faults == WheelState.FAULT_COMMAND_TIMEOUT


def test_watchdog_fault_does_not_clear_before_stop_acknowledgement(graph):
    driver, mcu, _ = graph
    mcu.drop_stops = 1000
    mcu.faults = WheelState.FAULT_COMMAND_TIMEOUT
    wait(lambda: len(mcu.stops) >= 3)
    assert driver.fault_reason == 'MCU fault bits: 0x1'
    mcu.drop_stops = 0
    wait(lambda: driver.fault_reason == '' and driver.diagnostic_error == 'MCU command timeout')
    assert mcu.enables == 0


def test_unknown_mcu_fault_remains_unavailable(graph):
    driver, mcu, _ = graph
    mcu.faults = 2
    wait(lambda: driver.fault_reason == 'MCU fault bits: 0x2')
    assert not driver._disarmed(.01)
    assert not driver.enabled and mcu.enables == 0
