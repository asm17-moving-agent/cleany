"""UUID4 test client with feedback, optional stage cancellation and journal lookup."""

from __future__ import annotations

import argparse
import json
import time
import uuid

from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.srv import GetManipulationExecution
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .manipulation.models import Stage


def wait(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError('ROS response timed out; inspect the execution record before retrying')
    return future.result()


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/mock')
    parser.add_argument('--execution-id', default=None)
    parser.add_argument('--mission-id', default='mock-mission')
    parser.add_argument('--task-id', default='mock-task')
    parser.add_argument('--snapshot-id', default='mock-snapshot-001')
    parser.add_argument('--object-id', type=int, default=1)
    parser.add_argument('--destination-id', default='mock_trash_bin')
    parser.add_argument('--cancel-stage', choices=[stage.value for stage in Stage])
    parser.add_argument('--query', action='store_true', help='Only query --execution-id')
    parser.add_argument('--timeout', type=float, default=120.0)
    options, ros_args = parser.parse_known_args(args)
    if options.query and not options.execution_id:
        parser.error('--query requires --execution-id')
    execution_id = options.execution_id or str(uuid.uuid4())
    rclpy.init(args=ros_args)
    node = Node('manipulation_test_client')
    prefix = options.namespace.rstrip('/')
    client = ActionClient(node, ExecuteManipulationSkill, prefix + '/manipulation/execute_skill')
    service = node.create_client(GetManipulationExecution, prefix + '/manipulation/get_execution')
    handle = None
    cancel_future = None
    reached_cancel_stage = False

    def feedback(message):
        nonlocal cancel_future, reached_cancel_stage
        data = message.feedback
        print(f'{data.execution_id} {data.stage} {data.message}', flush=True)
        if data.stage == options.cancel_stage:
            reached_cancel_stage = True
        if reached_cancel_stage and handle is not None and cancel_future is None:
            cancel_future = handle.cancel_goal_async()

    try:
        print(f'execution_id={execution_id}', flush=True)
        if not options.query:
            if not client.wait_for_server(timeout_sec=10.0):
                raise TimeoutError('Action server unavailable')
            goal = ExecuteManipulationSkill.Goal(
                mission_id=options.mission_id, task_id=options.task_id,
                execution_id=execution_id, skill_name='collect_trash',
                snapshot_id=options.snapshot_id, object_id=options.object_id,
                destination_id=options.destination_id)
            handle = wait(node, client.send_goal_async(goal, feedback_callback=feedback), 10.0)
            if not handle.accepted:
                print('Goal rejected; inspecting execution record', flush=True)
            else:
                if reached_cancel_stage and cancel_future is None:
                    cancel_future = handle.cancel_goal_async()
                response = wait(node, handle.get_result_async(), options.timeout)
                result = response.result
                print(json.dumps({name: getattr(result, name) for name in result.get_fields_and_field_types()},
                                 sort_keys=True), flush=True)
        if not service.wait_for_service(timeout_sec=10.0):
            raise TimeoutError('Execution lookup unavailable')
        response = wait(node, service.call_async(GetManipulationExecution.Request(
            execution_id=execution_id)), 10.0)
        record = response.record
        print(json.dumps({'found': response.found, 'record': {
            name: getattr(record, name) for name in record.get_fields_and_field_types()}},
            sort_keys=True), flush=True)
        return 0 if response.found else 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
