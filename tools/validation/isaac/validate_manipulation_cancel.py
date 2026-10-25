"""Cancel an owned Isaac test execution during measured LiftObject motion.

Start this monitor before offering the Mission. It observes the existing public
BT contract and sends IMMEDIATE cancellation once, without resetting faults or
releasing the payload. Run only on the dedicated test ROS domain.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cleany_interfaces.msg import ManipulationExecutionRecord, MissionStatus
from cleany_interfaces.srv import CancelManipulation
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES


def validate(output: Path, timeout: float) -> dict:
    rclpy.init()
    node = Node('isaac_manipulation_cancel_validation')
    state = {'record': None, 'mission': None, 'trajectory_status': None}
    joints = deque(maxlen=2000)
    stop_events = []
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def joint_sample(message):
        velocities = dict(zip(message.name, message.velocity))
        if not all(name in velocities for name in REQUIRED_JOINT_NAMES):
            return
        joints.append(dict(wall_s=time.monotonic(),
            stamp_s=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            velocities={name: velocities[name] for name in REQUIRED_JOINT_NAMES}))

    node.create_subscription(ManipulationExecutionRecord, '/sim/manipulation/execution_events',
        lambda msg: state.update(record=msg), qos)
    node.create_subscription(MissionStatus, '/mission/status',
        lambda msg: state.update(mission=msg), qos)
    node.create_subscription(JointState, '/joint_states', joint_sample, qos_profile_sensor_data)
    node.create_subscription(GoalStatusArray, '/execute_trajectory/_action/status',
        lambda msg: state.update(trajectory_status=msg), qos)
    node.create_subscription(String, '/trajectory_execution_event',
        lambda msg: stop_events.append(dict(wall_s=time.monotonic(), value=msg.data)), 100)
    cancel = node.create_client(CancelManipulation, '/sim/manipulation/cancel')
    ready = node.create_client(Trigger, '/sim/manipulation/ready_to_drive')
    report = dict(passed=False, scope='actual BT mid-motion IMMEDIATE cancellation; no cleanup success')

    def wait(predicate, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise TimeoutError('Cancellation validation checkpoint timed out')

    def lifting():
        record = state['record']
        trajectory = state['trajectory_status']
        return bool(record and record.execution_profile == 'isaac' and not record.has_result
            and record.substage == 'LiftObject' and joints
            and trajectory and any(item.status == GoalStatus.STATUS_EXECUTING
                for item in trajectory.status_list)
            and time.monotonic()-joints[-1]['wall_s'] < .3
            and max(abs(v) for name, v in joints[-1]['velocities'].items()
                if name.startswith(record.selected_arm+'_') and 'gripper' not in name) > .05
            and cancel.service_is_ready())

    try:
        wait(lifting, timeout)
        execution_id = state['record'].execution_id
        report['before'] = message_to_ordereddict(state['record'])
        report['active_trajectory'] = message_to_ordereddict(state['trajectory_status'])
        report['moving_sample'] = joints[-1]
        requested = time.monotonic()
        future = cancel.call_async(CancelManipulation.Request(
            execution_id=execution_id, mode='IMMEDIATE'))
        wait(future.done, 15.)
        report['cancel'] = message_to_ordereddict(future.result())
        assert future.result().accepted, report['cancel']
        wait(lambda: state['record'].execution_id == execution_id and state['record'].has_result, 30.)
        terminal = time.monotonic()
        report['terminal_latency_s'] = terminal-requested
        record = state['record']
        report['result'] = message_to_ordereddict(record)
        assert record.status == 'CANCELED' and record.cancel_mode == 'IMMEDIATE', report['result']
        assert record.stop_confirmed and not record.arm_recovered, report['result']
        assert any(event['wall_s'] >= requested and event['value'] == 'stop' for event in stop_events)

        def stopped():
            samples = [item for item in joints if item['wall_s'] >= terminal]
            if len(samples) < 5 or time.monotonic()-samples[-1]['wall_s'] > .3:
                return False
            recent = [item for item in samples if item['stamp_s'] >= samples[-1]['stamp_s']-.4]
            return (len(recent) >= 5 and recent[-1]['stamp_s']-recent[0]['stamp_s'] >= .25
                and all(0 < b['stamp_s']-a['stamp_s'] <= .1 for a, b in zip(recent, recent[1:]))
                and all(abs(v) <= .02 for item in recent for v in item['velocities'].values()))

        wait(stopped, 15.)
        wait(ready.service_is_ready, 5.)
        future = ready.call_async(Trigger.Request())
        wait(future.done, 5.)
        report['drive_ready'] = message_to_ordereddict(future.result())
        assert not future.result().success, report['drive_ready']
        wait(lambda: state['mission'] is not None
            and state['mission'].mission_id in ('', record.mission_id)
            and state['mission'].state == 'ERROR'
            and state['mission'].phase == 'TERMINAL'
            and not state['mission'].ready, 15.)
        report['passed'] = True
    except Exception as error:
        report['error'] = repr(error)
        raise
    finally:
        report['joint_samples'] = list(joints)
        report['stop_events'] = stop_events
        report['mission_status'] = message_to_ordereddict(state['mission']) if state['mission'] else None
        (output/'evaluations/manipulation-cancel.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/output'))
    parser.add_argument('--timeout', type=float, default=600.)
    args = parser.parse_args()
    result = validate(args.output, args.timeout)
    print(json.dumps({k: v for k, v in result.items() if k != 'joint_samples'}, indent=2))
