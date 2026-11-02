"""Submit one mission through the real Backend API and save its observed lifecycle.

Run against an isolated simulation backend; this script sends a mission request.
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from urllib.request import Request, urlopen
from uuid import uuid4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api-url", default="http://127.0.0.1:8080")
    parser.add_argument("--seat", default="seat-12")
    parser.add_argument("--mission-id", help="observe an already-submitted mission")
    parser.add_argument("--timeout", type=float, default=420)
    parser.add_argument("--cancel-after", type=float)
    parser.add_argument("--cancel-phase", choices=["NAVIGATING", "WORKING", "RETURNING"],
                        default="NAVIGATING", help="cancel only after this execution checkpoint")
    parser.add_argument("--expected-outcome", default="SUCCESS")
    parser.add_argument("--expected-robot-state", choices=["IDLE", "ERROR"], default="IDLE")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; select a new evidence file")

    def api(path, body=None):
        request = Request(args.api_url.rstrip("/") + path,
                          data=json.dumps(body).encode() if body is not None else None,
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=3) as response:
            return json.load(response)

    created = api("/api/missions", {"seat_id": args.seat, "priority": "NORMAL",
                                   "requested_by": "scrum-420-smoke",
                                   "idempotency_key": str(uuid4())}) if not args.mission_id else None
    mid = args.mission_id or created["mission_id"]
    evidence = {"api_url": args.api_url, "mission_id": mid, "observations": [],
                "scope": "Backend gateway, ROS runtime, configured navigation; cleaning remains mock"}
    started = time.monotonic()
    signature = ""
    cancelled = False
    try:
        while time.monotonic() - started < args.timeout:
            missions = api("/api/missions")["items"]
            mission = next(item for item in missions if item["mission_id"] == mid)
            current = json.dumps(mission, sort_keys=True)
            if current != signature:
                signature = current
                evidence["observations"].append({"elapsed_seconds": round(time.monotonic() - started, 3),
                                                 "mission": mission})
                print(f"{mission['phase']} {mission.get('message', '')}", flush=True)
            if mission["phase"] == "TERMINAL":
                evidence["robot"] = api("/api/robots")["items"][0]
                assert mission["outcome"] == args.expected_outcome, mission
                assert mission["execution_profile"]["execution"] == "mock", mission
                robot = evidence["robot"]
                fresh = robot.get("last_seen_at") and datetime.fromisoformat(
                    robot["last_seen_at"].replace("Z", "+00:00")
                ) >= datetime.fromisoformat(mission["finished_at"].replace("Z", "+00:00"))
                if (fresh and robot["state"] == args.expected_robot_state
                        and robot["active_mission_id"] is None):
                    return
            if (args.cancel_after is not None and not cancelled
                    and mission["phase"] == args.cancel_phase
                    and time.monotonic() - started >= args.cancel_after):
                api(f"/api/missions/{mid}/cancel", {})
                cancelled = True
            time.sleep(.2)
        raise TimeoutError(f"Mission {mid} did not terminate within {args.timeout}s; inspect and cancel explicitly")
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
