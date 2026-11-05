"""Operator CLI: list active faults or request a verified fault reset.

The reset is a request: the Supervisor still demands fresh stable health (or a confirmed
mission stop) and records the operator identity in the audit event stream.
"""
import argparse
from threading import Event
from time import monotonic, sleep

import rclpy
from cleany_interfaces.msg import SystemFaults, SystemHeartbeat
from cleany_interfaces.srv import RequestFaultReset
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def _wait(node, predicate, timeout):
    deadline = monotonic() + timeout
    while monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=.05)
    return predicate()


def main(args=None):
    parser = argparse.ArgumentParser(description='Cleany System Supervisor operator tool')
    parser.add_argument('fault_code', nargs='?', help='fault to reset; omit to list active faults')
    parser.add_argument('--operator', help='operator identity recorded with the reset request')
    parser.add_argument('--timeout', type=float, default=3.)
    options, ros_args = parser.parse_known_args(args)
    if options.fault_code and not options.operator:
        parser.error('--operator is required for a reset')
    rclpy.init(args=ros_args)
    node = Node('supervisor_operator_cli')
    heartbeat, faults, received = {}, {}, Event()
    node.create_subscription(SystemHeartbeat, 'system/heartbeat', lambda msg: heartbeat.update(epoch=msg.epoch), 1)
    node.create_subscription(SystemFaults, 'system/faults', lambda msg: (faults.update(msg=msg), received.set()),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        if not _wait(node, lambda: 'epoch' in heartbeat and received.is_set(), options.timeout):
            print('Supervisor heartbeat/fault snapshot not received; is system_supervisor running?')
            return 2
        snapshot = faults['msg']
        if snapshot.epoch != heartbeat['epoch']:
            sleep(.1)
            _wait(node, lambda: faults['msg'].epoch == heartbeat['epoch'], options.timeout)
            snapshot = faults['msg']
        if not options.fault_code:
            active = [f for f in snapshot.faults if f.active]
            print(f'epoch {heartbeat["epoch"]}: {len(active)} active fault(s)')
            for fault in active:
                print(f'  {fault.code} [{fault.severity}{", latched" if fault.latched else ""}] '
                      f'{fault.source}: {fault.detail}')
            return 0
        client = node.create_client(RequestFaultReset, 'system/request_fault_reset')
        if not client.wait_for_service(timeout_sec=options.timeout):
            print('reset service unavailable')
            return 2
        future = client.call_async(RequestFaultReset.Request(
            epoch=heartbeat['epoch'], fault_code=options.fault_code, operator_id=options.operator))
        if not _wait(node, future.done, options.timeout):
            print('reset request timed out')
            return 2
        response = future.result()
        print(('RESET: ' if response.reset else 'NOT RESET: ') + response.reason)
        return 0 if response.reset else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
