"""Nav2와 조작 어댑터가 공유하는 Action 목표 식별자입니다."""

from uuid import UUID

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID as GoalUUID


def goal_uuid(value: str) -> GoalUUID:
    return GoalUUID(uuid=list(UUID(value).bytes))


def cancel_request(value: str) -> CancelGoal.Request:
    return CancelGoal.Request(goal_info=GoalInfo(goal_id=goal_uuid(value)))
