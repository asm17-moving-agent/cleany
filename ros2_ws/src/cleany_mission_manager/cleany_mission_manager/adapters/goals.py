"""Action goal identity shared by the Nav2 and manipulation adapters."""

from uuid import UUID

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID as GoalUUID


def goal_uuid(value: str) -> GoalUUID:
    return GoalUUID(uuid=list(UUID(value).bytes))


def cancel_request(value: str) -> CancelGoal.Request:
    return CancelGoal.Request(goal_info=GoalInfo(goal_id=goal_uuid(value)))
