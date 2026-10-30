"""UUID4 test client with feedback, optional stage cancellation and journal lookup."""

from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path

from cleany_interfaces.action import ExecuteManipulationSkill, InspectScene
from cleany_interfaces.srv import CancelManipulation, ConfirmManipulationRecovery, GetManipulationExecution
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .manipulation.models import CancelMode, Stage, Status
from .core.automatic_goal import select_automatic_target
from .core.command_progress import CommandProgress


def wait(node: Node, future, timeout: float):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise TimeoutError('ROS response timed out; inspect the execution record before retrying')
    return future.result()


def require_execution_available(node: Node, prefix: str = '') -> None:
    """Read historical inhibition before spending time on scene inspection."""
    status = node.create_client(ConfirmManipulationRecovery, prefix + '/manipulation/confirm_recovery')
    try:
        if not status.wait_for_service(timeout_sec=2.):
            raise RuntimeError('Manipulation recovery status unavailable before inspection')
        # Both confirmation flags remain false: this request never clears a block.
        response = wait(node, status.call_async(ConfirmManipulationRecovery.Request()), 2.)
        if not response.success or response.inhibited:
            raise RuntimeError('Manipulation is inhibited; inspect and confirm recovery before scene inspection: '
                + response.message)
    finally:
        node.destroy_client(status)


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/mock')
    parser.add_argument('--execution-id', default=None)
    parser.add_argument('--mission-id', default='mock-mission')
    parser.add_argument('--task-id', default='mock-task')
    parser.add_argument('--snapshot-id', default='mock-snapshot-001')
    parser.add_argument('--object-id', type=int, default=1)
    parser.add_argument('--skill-name', choices=['collect_trash', 'collect_lost_item'], default='collect_trash')
    parser.add_argument('--destination-id', default='mock_trash_bin')
    parser.add_argument('--cancel-stage', choices=[stage.value for stage in Stage])
    parser.add_argument('--cancel-mode', choices=[mode.value for mode in CancelMode], default='CHECKPOINT')
    parser.add_argument('--query', action='store_true', help='Only query --execution-id')
    parser.add_argument('--auto-select', action='store_true', help='Detect and select one actual classified object')
    parser.add_argument('--inspection-query', default='')
    parser.add_argument('--collection-config', help='Destination policy for --auto-select')
    parser.add_argument('--timeout', type=float, default=120.0)
    options, ros_args = parser.parse_known_args(args)
    client_began = time.monotonic()
    timings = {}
    if options.query and not options.execution_id:
        parser.error('--query requires --execution-id')
    if options.auto_select and (options.query or not options.collection_config or options.namespace.rstrip('/') != ''):
        parser.error('--auto-select requires --namespace /, --collection-config and execution mode')
    execution_id = options.execution_id or str(uuid.uuid4())
    progress = None if options.query else CommandProgress.from_environment(execution_id)

    def report(phase, message='', *, state='ACTIVE'):
        if progress is not None:
            progress.update(phase, message, state=state)
    rclpy.init(args=ros_args)
    node = Node('manipulation_test_client')
    node.declare_parameter('bt_minimum_confidence', .25)
    node.declare_parameter('bt_maximum_smaller_extent_m', .11)
    node.declare_parameter('bt_denylist', ['battery', 'knife', 'broken glass'])
    prefix = options.namespace.rstrip('/')
    client = ActionClient(node, ExecuteManipulationSkill, prefix + '/manipulation/execute_skill')
    service = node.create_client(GetManipulationExecution, prefix + '/manipulation/get_execution')
    cancel_service = node.create_client(CancelManipulation, prefix + '/manipulation/cancel')
    handle = None
    cancel_future = None
    reached_cancel_stage = False
    execution_ok = options.query
    inspection_client = inspection_handle = None
    inspection_finished = execution_finished = False

    def request_cancel():
        if options.cancel_mode == 'CHECKPOINT':
            return handle.cancel_goal_async()
        return cancel_service.call_async(CancelManipulation.Request(
            execution_id=execution_id, mode=options.cancel_mode))

    def feedback(message):
        nonlocal cancel_future, reached_cancel_stage
        data = message.feedback
        step = f'/{data.substage}' if data.substage else ''
        print(f'{data.execution_id} {data.stage}{step} {data.message}', flush=True)
        if data.stage == options.cancel_stage:
            reached_cancel_stage = True
        if reached_cancel_stage and handle is not None and cancel_future is None:
            cancel_future = request_cancel()

    try:
        print(f'execution_id={execution_id}', flush=True)
        if not options.query:
            report('CHECKING_EXECUTION', 'Checking Action availability and recovery inhibition')
            if not client.wait_for_server(timeout_sec=10.0):
                raise TimeoutError('Action server unavailable')
            if (options.cancel_stage and options.cancel_mode != 'CHECKPOINT'
                    and not cancel_service.wait_for_service(timeout_sec=10.0)):
                raise TimeoutError('Cancellation mode service unavailable')
            if options.auto_select:
                require_execution_available(node, prefix)
                report('INSPECTING_SCENE', 'Capturing and classifying the initial scene')
                inspection_began = time.monotonic()
                config = json.loads(Path(options.collection_config).expanduser().read_text())
                inspection_client = ActionClient(node, InspectScene, '/perception/inspect_scene')
                if not inspection_client.wait_for_server(timeout_sec=10.):
                    raise TimeoutError('Actual scene inspection server unavailable')
                inspection_handle = wait(node, inspection_client.send_goal_async(
                    InspectScene.Goal(query=options.inspection_query)), 10.)
                if not inspection_handle.accepted:
                    raise RuntimeError('Scene inspection rejected')
                inspected = wait(node, inspection_handle.get_result_async(), options.timeout).result
                inspection_finished = True
                timings['inspection_wall_sec'] = time.monotonic() - inspection_began
                if not inspected.success:
                    raise RuntimeError(inspected.message)
                report('SELECTING_TARGET', 'Selecting an eligible object from the observed scene')
                target = select_automatic_target(inspected.detections, config,
                    minimum_confidence=float(node.get_parameter('bt_minimum_confidence').value),
                    maximum_smaller_extent_m=float(node.get_parameter('bt_maximum_smaller_extent_m').value),
                    denylist=tuple(node.get_parameter('bt_denylist').value))
                options.snapshot_id, options.object_id = target.snapshot_id, target.object_id
                options.skill_name, options.destination_id = target.skill_name, target.destination_id
                print(json.dumps(vars(target), sort_keys=True), flush=True)
            goal = ExecuteManipulationSkill.Goal(
                mission_id=options.mission_id, task_id=options.task_id,
                execution_id=execution_id, skill_name=options.skill_name,
                snapshot_id=options.snapshot_id, object_id=options.object_id,
                destination_id=options.destination_id)
            execution_began = time.monotonic()
            report('SUBMITTING_GOAL', f'Submitting approved object {options.snapshot_id}/{options.object_id}')
            handle = wait(node, client.send_goal_async(goal, feedback_callback=feedback), 10.0)
            timings['goal_response_wall_sec'] = time.monotonic() - execution_began
            if not handle.accepted:
                print('Goal rejected; inspecting execution record', flush=True)
            else:
                report('EXECUTING', 'Action accepted; physical progress is reported by the manipulation server')
                if reached_cancel_stage and cancel_future is None:
                    cancel_future = request_cancel()
                response = wait(node, handle.get_result_async(), options.timeout)
                execution_finished = True
                timings['execution_wall_sec'] = time.monotonic() - execution_began
                result = response.result
                print(json.dumps({name: getattr(result, name) for name in result.get_fields_and_field_types()},
                                 sort_keys=True), flush=True)
                execution_ok = (result.status == Status.SUCCESS.value or
                    (cancel_future is not None and result.status == Status.CANCELED.value
                     and result.stop_confirmed))
        if not service.wait_for_service(timeout_sec=10.0):
            raise TimeoutError('Execution lookup unavailable')
        response = wait(node, service.call_async(GetManipulationExecution.Request(
            execution_id=execution_id)), 10.0)
        record = response.record
        print(json.dumps({'found': response.found, 'record': {
            name: getattr(record, name) for name in record.get_fields_and_field_types()}},
            sort_keys=True), flush=True)
        success = execution_ok and response.found
        report('COMMAND_FINISHED' if success else 'COMMAND_FAILED',
               'Execution command finished; inspect the server result', state='FINISHED' if success else 'FAILED')
        return 0 if success else 1
    except BaseException as error:
        report('COMMAND_FAILED', str(error) or type(error).__name__, state='FAILED')
        raise
    finally:
        # A timeout/interrupt must not silently abandon accepted physical work.
        if handle is not None and handle.accepted and not execution_finished:
            try:
                if cancel_service.wait_for_service(timeout_sec=2.):
                    wait(node, cancel_service.call_async(CancelManipulation.Request(
                        execution_id=execution_id, mode='IMMEDIATE')), 5.)
                else:
                    wait(node, handle.cancel_goal_async(), 5.)
            except Exception as error:
                print(f'Cancellation acknowledgement unavailable: {error}; execution_id={execution_id}', flush=True)
        if inspection_handle is not None and inspection_handle.accepted and not inspection_finished:
            try:
                wait(node, inspection_handle.cancel_goal_async(), 5.)
            except Exception:
                pass
        if inspection_client is not None:
            inspection_client.destroy()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps({'event': 'manipulation_client_timing', 'execution_id': execution_id,
            'client_wall_sec': time.monotonic() - client_began,
            'inspection_finished': inspection_finished, 'execution_finished': execution_finished,
            **timings}, sort_keys=True), flush=True)
