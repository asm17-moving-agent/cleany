"""Mock 포트와 모의 시계로 현재 MissionRuntime의 상태 변화를 관찰합니다."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws/src/cleany_mission_manager"))

from cleany_mission_manager.core.operations import DeferredPort  # noqa: E402
from cleany_mission_manager.core.result import FailureCode, ModuleResult  # noqa: E402
from cleany_mission_manager.core.runtime import MissionRuntime  # noqa: E402
from cleany_mission_manager.core.runtime_models import (  # noqa: E402
    RuntimeRequest, RuntimeState, TaskProposal,
)
from cleany_mission_manager.mocks.desk import MockDesk  # noqa: E402


class Clock:
    """실제 대기나 ROS 연결 없이 시간을 진행합니다."""

    def __init__(self) -> None:
        self.now = 10.0

    def __call__(self) -> float:
        return self.now


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("success", "retry", "blocked", "fatal"), default="success")
    args = parser.parse_args()
    clock = Clock()
    desk = MockDesk(clock)
    navigator = DeferredPort(lambda _: ModuleResult.success(), polls=3)
    if args.scenario == "retry":
        failed_once = False

        def execute(command: object) -> ModuleResult:
            nonlocal failed_once
            if not failed_once:
                failed_once = True
                return ModuleResult.failed(FailureCode.GRASP_FAIL, retryable=True)
            return desk.execute(command)

        desk.executor.handler = execute
    elif args.scenario == "blocked":
        desk.planner.handler = lambda _: ModuleResult.success(
            TaskProposal("collect_trash", "expired-snapshot", "trash-1")
        )
    elif args.scenario == "fatal":
        navigator.handler = lambda _: ModuleResult.fatal(FailureCode.HARDWARE_ERROR)

    runtime = MissionRuntime(
        navigator=navigator, perception=desk.perception, planner=desk.planner,
        executor=desk.executor, clock=clock, supported_targets={"seat-12"},
        on_accept=desk.begin,
    )
    print("scenario:", args.scenario)
    print("initial:", runtime.state.value)
    admission = runtime.offer(RuntimeRequest("guide_001", "seat-12", "student"))
    if not admission.accepted:
        raise RuntimeError(f"The example request was rejected: {admission.reason}")
    print("accepted:", admission.accepted)
    print("start:", runtime.state.value)
    for _ in range(1000):
        before = runtime.state
        runtime.tick()
        clock.now += 0.1
        if before != runtime.state:
            print(f"{before.value} -> {runtime.state.value}")
        if runtime.state in (RuntimeState.IDLE, RuntimeState.ERROR):
            break
    else:
        raise RuntimeError("The example did not finish within 1000 ticks.")
    report = runtime.last_report
    if report is None:
        raise RuntimeError("The example produced no report.")
    print("report:", report.outcome)
    print("completed:", ", ".join(report.completed_tasks) or "(none)")
    print("observations:", desk.observations)
    print("execution requests:", len(desk.executor.commands))
    print("return:", report.return_result)


if __name__ == "__main__":
    main()
