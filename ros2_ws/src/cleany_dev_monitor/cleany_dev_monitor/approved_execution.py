"""Execute one approved goal through the worker's ROS node."""

import json
import threading
import time

from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.srv import GetManipulationExecution
from rclpy.action import ActionClient


class ExecutionUnconfirmed(RuntimeError):
    """A submitted goal may have run without a durable result we can verify."""


def approved_goal(job: dict):
    """Bind the goal to the server-stored proposal and request UUID."""
    proposal = job['proposal']
    return ExecuteManipulationSkill.Goal(
        execution_id=job['id'], mission_id=proposal['tracking_session_id'],
        task_id=proposal['id'], snapshot_id=proposal['snapshot_id'],
        object_id=proposal['target']['object_id'], skill_name=proposal['skill_name'],
        destination_id=proposal['destination_id'])


def message_fields(message) -> dict:
    return {key: getattr(message, key) for key in message.get_fields_and_field_types()}


class ApprovedExecution:
    """Reuse the Action and execution lookup clients across approved jobs."""

    def __init__(self, port):
        self.port = port
        self.action = ActionClient(port.node, ExecuteManipulationSkill, '/manipulation/execute_skill')
        self.lookup = port.node.create_client(GetManipulationExecution, '/manipulation/get_execution')

    def close(self):
        self.action.destroy()

    def execute(self, job: dict, cancelled) -> dict:
        goal = approved_goal(job)
        timeout = float(job['proposal_policy']['proposal_execution_timeout_sec'])
        if not self.action.wait_for_server(timeout_sec=10.):
            raise RuntimeError('조작 Action 서버가 없습니다.')
        if cancelled():
            raise InterruptedError('실행 요청이 취소되었습니다.')

        print(f'execution_id={goal.execution_id}', flush=True)
        try:
            response = self.action.send_goal_async(goal, feedback_callback=self.feedback)
            handle = self.port.wait(response, 10., '조작 Action 접수')
        except Exception as error:
            return self.verify_after_interruption(goal.execution_id, error)
        if not handle.accepted:
            raise RuntimeError('조작 실행 요청이 거절되었습니다.')

        self.wait_result(handle, goal.execution_id, cancelled, timeout)
        return self.final_record(goal.execution_id)

    @staticmethod
    def feedback(message):
        value = message.feedback
        step = f'/{value.substage}' if value.substage else ''
        print(f'{value.execution_id} {value.stage}{step} {value.message}', flush=True)

    def wait_result(self, handle, execution_id: str, cancelled, timeout: float):
        stop_requested = False
        try:
            result = handle.get_result_async()
            finished = threading.Event()
            result.add_done_callback(lambda _: finished.set())
            deadline = time.monotonic() + timeout
            while not result.done():
                remaining = deadline - time.monotonic()
                if cancelled() or remaining <= 0:
                    stop_requested = True
                    self.request_stop(execution_id)
                    finished.wait(25.)
                    break
                finished.wait(min(.1, remaining))
            if result.done():
                print(json.dumps(message_fields(result.result().result), sort_keys=True), flush=True)
        except Exception as error:
            print(f'Action result unavailable; execution_id={execution_id}; {error}', flush=True)
            if not stop_requested:
                self.request_stop(execution_id)

    def request_stop(self, execution_id: str):
        try:
            response = self.port.cancel(execution_id, 'IMMEDIATE')
            print(f'Cancellation accepted={response["accepted"]}; {response["message"]}', flush=True)
        except Exception as error:
            print(f'Cancellation acknowledgement unavailable; execution_id={execution_id}; {error}', flush=True)

    def verify_after_interruption(self, execution_id: str, error: Exception) -> dict:
        print(f'Action response unavailable; execution_id={execution_id}; {error}', flush=True)
        self.request_stop(execution_id)
        return self.final_record(execution_id)

    def final_record(self, execution_id: str) -> dict:
        label = '조작 실행 결과 조회'
        try:
            response = self.port.call(self.lookup, GetManipulationExecution.Request(execution_id=execution_id),
                                      discovery_timeout=3., read_only_label=label)
            record = response.record
            if (response.found and record.execution_id == execution_id and record.has_result
                    and record.status in ('SUCCESS', 'FAILED', 'BLOCKED', 'CANCELED')):
                fields = message_fields(record)
                print(json.dumps({'found': True, 'record': fields}, sort_keys=True), flush=True)
                return fields
        except Exception as error:
            print(f'{label}: {error}', flush=True)
        raise ExecutionUnconfirmed('조작 실행 결과를 확인하지 못했습니다. 실행기와 실물 상태를 확인하세요.')
