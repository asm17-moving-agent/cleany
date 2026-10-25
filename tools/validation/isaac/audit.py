"""Audit retained Mission reports against their independent SQLite journals."""
import json
import math
from pathlib import Path
import sqlite3

import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("runs", nargs="+", type=Path)
parser.add_argument("--output", required=True, type=Path)
args = parser.parse_args()
summary = {"scope": "actual Mission/Nav2/Isaac; mock perception/planning/manipulation", "runs": []}
for path in args.runs:
    path = path.resolve()
    run = json.loads((path / "provenance/run.json").read_text())
    report = json.loads((path / "evaluations/mission.json").read_text())
    arm = json.loads((path / "evaluations/arm.json").read_text())
    result = report["mission_result"]
    scene = json.loads((path / "config/room.json").read_text())
    outcome = {"success": "SUCCESS", "navigation_failure": "FAILED", "manipulation_failure": "FAILED", "cancel": "CANCELLED"}[report["scenario"]]
    state = "ERROR" if report["scenario"] == "manipulation_failure" else "IDLE"
    assert run["status"] == "passed" and report["passed"] and arm["passed"]
    assert result["outcome"] == outcome
    assert report["final_status"]["state"] == state
    assert report["final_status"]["phase"] == "TERMINAL"
    assert report["final_status"]["ready"] == (state == "IDLE")
    if state == "ERROR":
        assert result["needs_human_review"] and result["return_result"] == "NOT_STARTED"
        assert all(not action["manipulation"]["arm_recovered"] for action in result["actions"])
    with sqlite3.connect(f"file:{path/'missions.db'}?mode=ro", uri=True) as database:
        missions = database.execute("SELECT mission_id, report FROM missions").fetchall()
        assert len(missions) == 1 and json.loads(missions[0][1]) == result
    with sqlite3.connect(f"file:{path/'executions.db'}?mode=ro", uri=True) as database:
        executions = [json.loads(row[0]) for row in database.execute("SELECT payload FROM executions")]
        assert all(record["record_state"] == "FINISHED" for record in executions)
        assert {record["goal"]["execution_id"] for record in executions} == {
            action["execution_id"] for action in result["actions"]}
    base = [json.loads(line) for line in (path / "evaluations/base.jsonl").read_text().splitlines()]
    assert all(b["time"] > a["time"] for a, b in zip(base, base[1:]))
    point = report["final_odom"]["pose"]["pose"]["position"]
    states = []
    for event in report["states"]:
        if not states or event["state"] != states[-1]:
            states.append(event["state"])
    summary["runs"].append({"run": str(path), "world": scene["world"], "target_id": scene["target_id"], "scenario": report["scenario"], "outcome": outcome,
        "failure_code": result["failure_code"], "states": states,
        "ready": report["final_status"]["ready"], "return_result": result["return_result"],
        "needs_human_review": result["needs_human_review"], "active_execution_records": 0,
        "mission_journal_matches_result": True, "base_clock_monotonic": True,
        "arm_max_error_rad": max(arm["move"]["errors_rad"].values()),
        "final_distance_from_home_m": math.hypot(point["x"]-scene["home"][0], point["y"]-scene["home"][1]),
        "elapsed_s": run["elapsed_s"], "navigation_source": run.get("navigation_source"),
        "parking": report.get("parking"), "home_error_m": report.get("home_error_m")})
summary["passed"] = True
args.output.write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
