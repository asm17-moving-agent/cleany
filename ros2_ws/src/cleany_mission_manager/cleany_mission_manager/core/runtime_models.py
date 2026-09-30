"""ROS-independent contracts for the asynchronous mission runtime."""

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import isfinite


class RuntimeState(str, Enum):
    IDLE = "IDLE"
    NAVIGATE_TO_TARGET = "NAVIGATE_TO_TARGET"
    WORKING = "WORKING"
    POST_MISSION = "POST_MISSION"
    RETURN_HOME = "RETURN_HOME"
    CANCELLING = "CANCELLING"
    ERROR = "ERROR"


@dataclass(frozen=True)
class RuntimeRequest:
    mission_id: str
    target_id: str
    requested_by: str
    mission_type: str = "clean_desk"
    target_kind: str = "SEAT"


@dataclass(frozen=True)
class RuntimePolicy:
    post_mission: str = "return_home"
    navigation_timeout: float = 180.0
    cleaning_timeout: float = 180.0
    operation_timeout: float = 30.0
    cancel_timeout: float = 5.0
    max_actions: int = 30
    max_skill_retries: int = 2

    def __post_init__(self) -> None:
        if self.post_mission not in ("return_home", "wait_for_next"):
            raise ValueError("post_mission must be return_home or wait_for_next")
        if any(not isfinite(value) or value <= 0 for value in (
            self.navigation_timeout, self.cleaning_timeout,
            self.operation_timeout, self.cancel_timeout,
        )) or self.max_actions < 1 or self.max_skill_retries < 0:
            raise ValueError("timeouts and action budget must be positive; retries nonnegative")


@dataclass(frozen=True)
class SceneObject:
    object_id: str
    disposition: str = "collect_trash"


@dataclass(frozen=True)
class SceneSnapshot:
    snapshot_id: str
    captured_at: float
    objects: tuple[SceneObject, ...]
    reference: str
    source: str = "mock"


@dataclass(frozen=True)
class TaskProposal:
    action: str
    snapshot_id: str
    object_id: str = ""


@dataclass(frozen=True)
class ActionRecord:
    object_id: str
    action: str
    snapshot_id: str
    status: str
    message: str = ""


@dataclass
class RuntimeReport:
    request: RuntimeRequest
    outcome: str
    summary: str
    failure_code: str = ""
    completed_tasks: list[str] = field(default_factory=list)
    skipped_tasks: list[str] = field(default_factory=list)
    failed_task: str = ""
    needs_human_review: bool = False
    before_observation: str = ""
    after_observation: str = ""
    navigation_result: str = "NOT_STARTED"
    return_result: str = "NOT_STARTED"
    cleaning_mode: str = "mock"
    execution_profile: dict[str, str] = field(default_factory=dict)
    actions: list[ActionRecord] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict) -> "RuntimeReport":
        data = dict(value)
        data["request"] = RuntimeRequest(**data["request"])
        data["actions"] = [ActionRecord(**item) for item in data.get("actions", [])]
        return cls(**data)


@dataclass(frozen=True)
class Admission:
    accepted: bool
    reason: str = ""
    duplicate: bool = False
    report: RuntimeReport | None = None
