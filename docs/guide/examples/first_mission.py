"""Print one step at a time using the project's real core and mock ports.

Run from the repository root. No ROS, simulator, or robot connection is used.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT / "ros2_ws/src/cleany_mission_manager"))

from cleany_mission_manager.core.manager import MissionManager  # noqa: E402
from cleany_mission_manager.core.models import MissionRequest  # noqa: E402
from cleany_mission_manager.core.ports import Navigator, Planner  # noqa: E402
from cleany_mission_manager.core.result import FailureCode, ModuleResult  # noqa: E402
from cleany_mission_manager.core.states import MissionState  # noqa: E402
from cleany_mission_manager.mocks.components import (  # noqa: E402
    InMemoryReporter,
    MockNavigator,
    MockPerception,
    MockPlanner,
    MockSkillExecutor,
    ScriptedNavigator,
    ScriptedPlanner,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("success", "retry", "blocked", "fatal"), default="success")
    args = parser.parse_args()
    navigator: Navigator = MockNavigator()
    planner: Planner = MockPlanner()
    if args.scenario == "retry":
        navigator = ScriptedNavigator([
            ModuleResult.failed(FailureCode.NAVIGATION_FAIL, retryable=True),
            ModuleResult.success(),
        ])
    elif args.scenario == "blocked":
        planner = ScriptedPlanner([ModuleResult.blocked(FailureCode.PLAN_BLOCKED)])
    elif args.scenario == "fatal":
        navigator = ScriptedNavigator([ModuleResult.fatal(FailureCode.HARDWARE_ERROR)])

    reporter = InMemoryReporter()
    manager = MissionManager(
        navigator=navigator,
        perception=MockPerception(),
        planner=planner,
        skill_executor=MockSkillExecutor(),
        reporter=reporter,
    )
    print("scenario:", args.scenario)
    print("initial:", manager.state.value)
    manager.start(MissionRequest("guide_001", "clean_seat", "seat_A_12", "student"))
    print("start:", manager.state.value)
    # Bound the teaching loop so an unexpected code change cannot hang the example.
    for tick in range(1, 31):
        if manager.state in (MissionState.IDLE, MissionState.ERROR):
            break
        before = manager.state.value
        manager.step()
        print(f"{tick:02}: {before} -> {manager.state.value}")
    if manager.state not in (MissionState.IDLE, MissionState.ERROR):
        raise RuntimeError("The teaching example did not finish within 30 steps.")

    report = manager.context.report
    if report is None:
        raise RuntimeError("The mission did not produce a report.")
    print("report:", report.status)
    print("completed:", ", ".join(report.completed_tasks) or "(none)")
    print("published:", len(reporter.reports))


if __name__ == "__main__":
    main()
