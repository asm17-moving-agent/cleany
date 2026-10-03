"""ROS transport only; executor BT/stage order is not part of Mission control."""

from uuid import UUID

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from rclpy.task import Future
from unique_identifier_msgs.msg import UUID as GoalUUID

from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.srv import GetManipulationExecution
from cleany_mission_manager.core.runtime_models import ManipulationOutcome


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
    def __init__(self, handle):
        self.handle = handle
        self.accepted = handle.accepted

    def get_result_async(self):
        return translated(self.handle.get_result_async(), lambda response: dict(
            status=response.status,
            result={name: getattr(response.result, name)
                    for name in ManipulationOutcome.__dataclass_fields__},
        ))


class ROSManipulationTransport:
    def __init__(self, node, action_name: str, query_name: str) -> None:
        self.client = ActionClient(node, ExecuteManipulationSkill, action_name)
        self.cancel_client = node.create_client(CancelGoal, action_name + "/_action/cancel_goal")
        self.query_client = node.create_client(GetManipulationExecution, query_name)

    def ready(self) -> bool:
        return (self.client.server_is_ready() and self.query_client.service_is_ready()
                and self.cancel_client.service_is_ready())

    def send(self, goal: dict):
        request = ExecuteManipulationSkill.Goal(**goal)
        return translated(self.client.send_goal_async(
            request, goal_uuid=GoalUUID(uuid=list(UUID(goal["execution_id"]).bytes))), GoalHandle)

    def cancel(self, execution_id: str):
        goal = GoalInfo(goal_id=GoalUUID(uuid=list(UUID(execution_id).bytes)))
        future = self.cancel_client.call_async(CancelGoal.Request(goal_info=goal))
        return translated(future, lambda response: response, self.cancel_client.remove_pending_request)

    def lookup(self, execution_id: str):
        future = self.query_client.call_async(GetManipulationExecution.Request(execution_id=execution_id))
        return translated(future, lambda response: (
            {name: getattr(response.record, name)
             for name in response.record.get_fields_and_field_types()}
            if response.found else None), self.query_client.remove_pending_request)
