"""Evaluate the requested 50/60 continuous campaign without hiding its failures."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
from audit import assess, stopped

# This sequence belongs to one experiment, not a project-wide acceptance rule.
SEATS = ["seat-44", "seat-16", "seat-17", "seat-20", "seat-09", "seat-35"]


def assess_campaign(result: dict, shutdown: dict) -> dict:
    """Require the fixed sixty-attempt sequence and at least fifty valid successes."""
    base = assess(result, shutdown)
    missions = result.get("missions", [])
    expected = SEATS * 10
    checks = {
        k: v for k, v in base["checks"].items() if k != "expected_mission_results"
    }
    checks["normal_scenario"] = result.get("scenario") == "normal"
    checks["same_sixty_seat_sequence"] = (
        result.get("expected_mission_count") == 60
        and result.get("chair_fixture_sequence") == expected
        and [m.get("seat") for m in missions] == expected
    )
    checks["all_terminals_match"] = bool(missions) and all(
        m.get("accepted")
        and m.get("matched_terminal")
        and (m.get("final", {}).get("last_result") or {})
        .get("request", {})
        .get("mission_id")
        == m.get("mission_id")
        and (m.get("final", {}).get("last_result") or {})
        .get("request", {})
        .get("target_id")
        == m.get("seat")
        for m in missions
    )
    successes, failures = [], []
    for index, (mission, verdict) in enumerate(zip(missions, base["missions"])):
        terminal = mission.get("final", {}).get("last_result") or {}
        valid = (
            verdict["passed"]
            and terminal.get("navigation_result") == "OK"
            and terminal.get("return_result") == "OK"
            and stopped(mission)
        )
        entry = {
            "index": index,
            "seat": mission.get("seat"),
            "mission_id": mission.get("mission_id"),
            "outcome": terminal.get("outcome"),
            "navigation": terminal.get("navigation_result"),
            "return": terminal.get("return_result"),
            "failure_code": terminal.get("failure_code"),
        }
        (successes if valid else failures).append(entry)
    checks["at_least_fifty_successes"] = len(successes) >= 50
    return {
        "achieved": all(checks.values()),
        "checks": checks,
        "successes": len(successes),
        "attempts": len(missions),
        "failures": failures,
        "scope": "One unchanged simulation, the original six seats repeated ten times; navigation sim, desk work mock.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("worker", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    verdict = assess_campaign(
        json.loads((args.worker / "result.json").read_text()),
        json.loads((args.worker / "shutdown.json").read_text()),
    )
    args.output.write_text(json.dumps(verdict, indent=2))
    print(json.dumps(verdict, indent=2))


if __name__ == "__main__":
    main()
