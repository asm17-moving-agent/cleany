"""Summarize raw evidence without treating expected failures as mission success."""

from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from worlds import chair_name


def stopped(latest: dict) -> bool:
    """정지 명령과 실제 이동 속도가 모두 정지 기준을 만족하는지 확인한다."""
    cmd = latest.get("cmd_vel")
    gt = latest.get("ground_truth", {})
    twist = gt.get("twist")
    return bool(
        cmd is not None
        and twist is not None
        and all(abs(v) < 1e-6 for v in cmd)
        and math.hypot(*twist[:2]) <= 0.03
        and abs(twist[2]) <= 0.05
    )


def assess(result: dict, shutdown: dict) -> dict:
    """실행 완료, 미션 식별자, 시나리오별 결과와 종료 근거를 대조한다."""
    scenario = result.get("scenario", "normal")
    checks = {
        "no_harness_error": not result.get("error"),
        "has_missions": bool(result.get("missions")),
        "complete_mission_count": len(result.get("missions", []))
        == result.get("expected_mission_count", len(result.get("missions", []))),
        "final_stopped": stopped(result.get("final_latest", {})),
        "clean_shutdown": bool(shutdown) and all(v == 0 for v in shutdown.values()),
    }
    missions = []
    if result.get("chair_fixture_sequence"):
        sequence = result["chair_fixture_sequence"]
        evidence = result.get("chair_scene_evidence", [])
        chairs = set(result.get("chair_inventory", []))
        checks["chair_sequence_verified"] = (
            bool(chairs)
            and sequence == [m.get("seat") for m in result.get("missions", [])]
            and len(evidence) == len(sequence)
            and all(
                e.get("seat") == seat
                and e.get("removed") == chair_name(seat)
                and len(e.get("observed_chairs", [])) == len(chairs) - 1
                and set(e.get("observed_chairs", [])) == chairs - {chair_name(seat)}
                for e, seat in zip(evidence, sequence)
            )
        )
    for m in result.get("missions", []):
        terminal = m.get("final", {}).get("last_result") or {}
        # 이전 미션의 마지막 결과를 현재 요청의 결과로 오인하지 않는다.
        matched = (
            terminal.get("request", {}).get("mission_id") == m.get("mission_id")
            and isinstance(m.get("seat"), str)
            and terminal.get("request", {}).get("target_id") == m["seat"]
        )
        outcome = terminal.get("outcome")
        # 기대한 실패 시나리오의 통과와 정상 미션 성공은 구분한다.
        if scenario == "normal":
            expected = outcome == "SUCCESS"
        elif scenario.startswith("cancel") or scenario == "sensor_loss":
            expected = outcome == "CANCELLED"
        elif scenario in ("safety_stop", "blocked", "path_failure"):
            expected = outcome != "SUCCESS" and bool(outcome)
        else:
            expected = outcome in ("SUCCESS", "FAILED", "BLOCKED")
        passed = bool(m.get("accepted") and matched and expected)
        if scenario in (
            "cancel_outbound",
            "cancel_return",
            "safety_stop",
            "sensor_loss",
        ):
            passed = (
                passed
                and bool(m.get("events"))
                and bool(m.get("stopped_after_stimulus"))
            )
        if scenario.startswith("cancel"):
            passed = passed and all(e.get("accepted") for e in m.get("events", []))
        if scenario == "safety_stop":
            passed = passed and terminal.get("failure_code") == "E_STOP"
        if scenario == "path_failure":
            passed = passed and any(
                e.get("stopped_node", {}).get("name") == "planner_server"
                for e in m.get("events", [])
            )
        if scenario == "blocked":
            passed = passed and "target_ground_truth" not in m
        if scenario == "obstacle":
            passed = (
                passed
                and bool(m.get("events"))
                and m.get("min_fixture_clearance_m", 0) > 0
            )
        if scenario == "sensor_loss":
            passed = passed and "STALE_DEPTH" in m.get(
                "guard_decisions_after_stimulus", {}
            )
            stop = m.get("stopped_after_stimulus", {}).get("wall_seconds", float("inf"))
            cleanup = m.get("cleanup_cancel", {}).get("wall", 0)
            event = m.get("events", [{}])[0].get("wall", 0)
            # 후처리 취소가 아니라 센서 유실 자체로 정지했는지 확인한다.
            passed = passed and bool(cleanup) and event + stop < cleanup
        missions.append(
            {
                "seat": m.get("seat"),
                "outcome": outcome,
                "failure_code": terminal.get("failure_code"),
                "passed": passed,
                "rtf": m.get("measured_rtf"),
                "stop_after_stimulus": m.get("stopped_after_stimulus"),
            }
        )
    checks["expected_mission_results"] = bool(missions) and all(
        m["passed"] for m in missions
    )
    return {
        "passed": all(checks.values()),
        "scenario": scenario,
        "checks": checks,
        "missions": missions,
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("directory", type=Path)
    args = p.parse_args()
    entries = {}
    for result_path in sorted(args.directory.glob("*/worker-*/result.json")):
        directory = result_path.parent
        shutdown = directory / "shutdown.json"
        entries[str(directory.relative_to(args.directory))] = assess(
            json.loads(result_path.read_text()),
            json.loads(shutdown.read_text()) if shutdown.exists() else {},
        )
    (args.directory / "audit.json").write_text(json.dumps(entries, indent=2))
    for name, e in entries.items():
        print(
            name,
            e["passed"],
            [
                (m["seat"], m["outcome"], round(m["rtf"], 3) if m["rtf"] else None)
                for m in e["missions"]
            ],
        )


if __name__ == "__main__":
    main()
