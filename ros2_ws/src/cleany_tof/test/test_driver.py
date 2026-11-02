"""Exercise the real serial bridge against a pseudo-terminal, without hardware."""
import json
import os
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from cleany_interfaces.msg import TofFrame
from cleany_tof.driver import TofDriver


@pytest.mark.parametrize('failure', ['replay', 'read_error', 'recovering'])
def test_serial_session_handshake_fragmentation_and_replay(failure):
    master, slave = os.openpty()
    os.set_blocking(master, False)
    rclpy.init(domain_id=194)
    driver = TofDriver(parameter_overrides=[Parameter('port', value=os.ttyname(slave))])
    observer = Node('pseudo_terminal_observer')
    messages = []
    observer.create_subscription(TofFrame, 'tof/frame', messages.append, 1)
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(observer)
    commands = bytearray()

    def until(predicate, timeout=3.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
            try:
                commands.extend(os.read(master, 65536))
            except BlockingIOError:
                pass
            if predicate():
                return
        assert predicate(), 'pseudo-terminal handshake timed out'

    try:
        until(lambda: b'stop\n' in commands)
        os.write(master, b'{"type":"event","event":"stopped"}\n')
        until(lambda: b'config 64 15 400000\nstart\n' in commands)
        os.write(master, b'{"type":"info","state":"running"}\n')
        until(lambda: driver.state == 'running')
        record = dict(type='frame', seq=1, time_us=1000000, read_us=36000, zones=64, hz=15,
                      distance_mm=[1000]*64, sigma_mm=[2]*64, target_status=[5]*64,
                      target_count=[1]*64, valid=[True]*64)
        raw = json.dumps(record).encode()+b'\n'
        os.write(master, raw[:15])
        until(lambda: len(driver.stream.buffer) == 15)
        assert not messages
        os.write(master, raw[15:])
        until(lambda: len(messages) == 1)
        assert messages[0].header.frame_id == 'tof_link' and messages[0].sequence == 1
        assert messages[0].distance_m[0] == 1.
        age = observer.get_clock().now().nanoseconds*1e-9-(messages[0].header.stamp.sec+messages[0].header.stamp.nanosec*1e-9)
        assert .09 < age < .25
        if failure == 'replay':
            fault = raw
        elif failure == 'read_error':
            fault = b'{"type":"event","event":"read_failed","status":255,"state":"running"}\n'
        else:
            fault = b'{"type":"event","event":"read_recovery","status":0,"state":"recovering"}\n'
        os.write(master, fault)
        until(lambda: driver.port is None)
        until(lambda: any(not m.sensor_ready for m in messages))
        assert len([m for m in messages if m.sensor_ready]) == 1
    finally:
        executor.shutdown()
        driver.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
