"""ROS transport only; executor BT/stage order is not part of Mission control."""

from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rclpy.task import Future

from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.srv import GetManipulationExecution
from cleany_mission_manager.core.runtime_models import ManipulationOutcome

from .goals import cancel_request, goal_uuid


def result_response(response) -> dict:
    return dict(
        status=response.status,
        result={name: getattr(response.result, name)
                for name in ManipulationOutcome.__dataclass_fields__},
        details={name: getattr(response.result, name, "")
                 for name in ("failed_substage", "cancel_mode")},
    )


def translated(future, convert, cancel_source=None):
    output = Future()

    def done(source):
        if output.done():
            return
        try:
            output.set_result(convert(source.result()))
        except Exception as exc:
            output.set_exception(exc)
    future.add_done_callback(done)
    if cancel_source:
        def cleanup(result):
            if result.cancelled():
                cancel_source(future)
                future.cancel()
        output.add_done_callback(cleanup)
    return output


class GoalHandle:
    def __init__(self, handle, finished):
        self.handle = handle
        self.accepted = handle.accepted
        self.finished = finished
        if not self.accepted:
            finished()

    def get_result_async(self):
        def convert(response):
            self.finished()
            return result_response(response)
        return translated(self.handle.get_result_async(), convert)


class ROSManipulationTransport:
    def __init__(self, node, action_name: str, query_name: str) -> None:
        action_name = node.resolve_topic_name(action_name)
        self.client = ActionClient(node, ExecuteManipulationSkill, action_name)
        self.cancel_client = node.create_client(CancelGoal, action_name + "/_action/cancel_goal")
        self.query_client = node.create_client(GetManipulationExecution, query_name)
        self.active_execution_id: str | None = None
        self.feedback: dict = {}

    def _finished(self, execution_id: str) -> None:
        if self.active_execution_id == execution_id:
            self.active_execution_id = None

    def _feedback(self, execution_id: str, message) -> None:
        try:
            feedback = message.feedback
            if self.active_execution_id != execution_id or feedback.execution_id != execution_id:
                return
            self.feedback = {name: getattr(feedback, name) for name in (
                "execution_id", "stage", "substage", "selected_arm", "message",
            )}
        except Exception:
            # Feedback is observation only; a malformed message cannot stop control.
            return

    def ready(self) -> bool:
        return (self.client.server_is_ready() and self.query_client.service_is_ready()
                and self.cancel_client.service_is_ready())

    def send(self, goal: dict):
        request = ExecuteManipulationSkill.Goal(**goal)
        execution_id = goal["execution_id"]
        self.active_execution_id, self.feedback = execution_id, {}
        return translated(self.client.send_goal_async(
            request, goal_uuid=goal_uuid(execution_id),
            feedback_callback=lambda message: self._feedback(execution_id, message)),
            lambda handle: GoalHandle(handle, lambda: self._finished(execution_id)))

    def cancel(self, execution_id: str):
        future = self.cancel_client.call_async(cancel_request(execution_id))
        return translated(future, lambda response: response, self.cancel_client.remove_pending_request)

    def lookup(self, execution_id: str):
        future = self.query_client.call_async(GetManipulationExecution.Request(execution_id=execution_id))
        def convert(response):
            if not response.found:
                return None
            record = {name: getattr(response.record, name)
                      for name in response.record.get_fields_and_field_types()}
            if record["execution_id"] == execution_id and record["record_state"] in (
                    "FINISHED", "INTERRUPTED"):
                self._finished(execution_id)
            return record
        return translated(future, convert, self.query_client.remove_pending_request)
