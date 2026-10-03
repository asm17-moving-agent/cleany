from dataclasses import dataclass
from enum import Enum
from typing import Generic, TypeVar


T = TypeVar("T")


class ResultStatus(str, Enum):
    OK = "OK"
    BLOCKED = "BLOCKED"
    FAILED = "FAILED"
    FATAL = "FATAL"
    CANCELLED = "CANCELLED"


class FailureCode(str, Enum):
    NAVIGATION_FAIL = "NAVIGATION_FAIL"
    PERCEPTION_FAIL = "PERCEPTION_FAIL"
    NO_OBJECTS = "NO_OBJECTS"
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    PLAN_EMPTY = "PLAN_EMPTY"
    PLAN_BLOCKED = "PLAN_BLOCKED"
    SKILL_FAIL = "SKILL_FAIL"
    GRASP_FAIL = "GRASP_FAIL"
    PLACE_FAIL = "PLACE_FAIL"
    TIMEOUT = "TIMEOUT"
    E_STOP = "E_STOP"
    HARDWARE_ERROR = "HARDWARE_ERROR"
    UNKNOWN_ERROR = "UNKNOWN_ERROR"


@dataclass(frozen=True)
class ModuleResult(Generic[T]):
    ok: bool
    status: ResultStatus
    failure_code: FailureCode | None = None
    retryable: bool = False
    message: str = ""
    data: T | None = None

    @classmethod
    def success(cls, data: T | None = None, message: str = "") -> "ModuleResult[T]":
        return cls(
            ok=True,
            status=ResultStatus.OK,
            failure_code=None,
            retryable=False,
            message=message,
            data=data,
        )

    @classmethod
    def failed(
        cls,
        failure_code: FailureCode,
        *,
        retryable: bool = False,
        message: str = "",
        data: T | None = None,
    ) -> "ModuleResult[T]":
        return cls(
            ok=False,
            status=ResultStatus.FAILED,
            failure_code=failure_code,
            retryable=retryable,
            message=message,
            data=data,
        )

    @classmethod
    def blocked(
        cls,
        failure_code: FailureCode,
        *,
        message: str = "",
        data: T | None = None,
    ) -> "ModuleResult[T]":
        return cls(
            ok=False,
            status=ResultStatus.BLOCKED,
            failure_code=failure_code,
            retryable=False,
            message=message,
            data=data,
        )

    @classmethod
    def fatal(
        cls,
        failure_code: FailureCode,
        *,
        message: str = "",
        data: T | None = None,
    ) -> "ModuleResult[T]":
        return cls(
            ok=False,
            status=ResultStatus.FATAL,
            failure_code=failure_code,
            retryable=False,
            message=message,
            data=data,
        )
