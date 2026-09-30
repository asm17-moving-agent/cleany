"""Gateway v1 event mapping and durable replay, independent of ROS and sockets."""

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from math import isfinite
from uuid import UUID, uuid4


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def observation_reference(reference: str, profile: dict) -> str | None:
    # Mock scene identifiers are diagnostic records, not photographs or observed evidence.
    if profile.get("perception") == "mock" or reference.startswith("mock://"):
        return None
    return reference or None


class BridgeSession:
    def __init__(self, path: str = ":memory:") -> None:
        if path != ":memory:":
            path = str(Path(path).expanduser())
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.executescript(
            "CREATE TABLE IF NOT EXISTS events (event_id TEXT PRIMARY KEY,"
            "mission_id TEXT, kind TEXT, payload TEXT, acknowledged INTEGER DEFAULT 0);"
            "CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT);"
            "CREATE TABLE IF NOT EXISTS commands (event_id TEXT PRIMARY KEY);"
        )
        self.snapshot = None

    def value(self, key: str, default: str = "") -> str:
        row = self.db.execute("SELECT value FROM metadata WHERE key=?", (key,)).fetchone()
        return row[0] if row else default

    def set(self, key: str, value: str) -> None:
        self.db.execute("INSERT OR REPLACE INTO metadata VALUES(?,?)", (key, value))

    def envelope(self, kind: str, payload: dict, mission_id: str | None = None,
                 sequence: int = 0) -> dict:
        return {"schema_version": 1, "event_id": str(uuid4()), "robot_id": "cleany-01",
                "mission_id": mission_id, "sequence": sequence, "occurred_at": utc_now(),
                "event_type": kind, "payload": payload}

    def enqueue(self, kind: str, mission_id: str, payload: dict) -> dict:
        with self.db:
            return self._enqueue(kind, mission_id, payload)

    def _enqueue(self, kind: str, mission_id: str, payload: dict) -> dict:
        UUID(mission_id)
        sequence = int(self.value("sequence:" + mission_id, "0")) + 1
        self.set("sequence:" + mission_id, str(sequence))
        event = self.envelope(kind, payload, mission_id, sequence)
        self.db.execute("INSERT INTO events(event_id,mission_id,kind,payload) VALUES(?,?,?,?)",
                        (event["event_id"], mission_id, kind, json.dumps(event)))
        return event

    def observe(self, snapshot: dict) -> None:
        self.snapshot = snapshot
        request = snapshot["active_request"]
        if request:
            mid = request["mission_id"]
            self.accepted(mid)
            phase = snapshot["phase"]
            signature = json.dumps([phase, snapshot["state"], snapshot["bt_stage"]])
            if phase != "ACCEPTED" and signature != self.value("phase:" + mid):
                with self.db:
                    self._enqueue("mission.phase", mid, {
                        "phase": phase, "message": f"{snapshot['state']} · {snapshot['bt_stage']}",
                        "internal_state": snapshot["state"],
                        "before_observation": observation_reference(
                            snapshot.get("before_observation", ""), snapshot["execution_profile"]),
                    })
                    self.set("phase:" + mid, signature)
        reports = snapshot.get("completed_reports", [])
        if snapshot["last_result"]:
            reports = [*reports, snapshot["last_result"]]
        for report in reports:
            self.result(report, snapshot["execution_profile"])

    def accepted(self, mission_id: str) -> None:
        if not self.value("accepted:" + mission_id):
            with self.db:
                self._enqueue("mission.accepted", mission_id, {"message": "Robot accepted the mission."})
                self.set("accepted:" + mission_id, "1")

    def result(self, report: dict, profile: dict) -> None:
        mid = report["request"]["mission_id"]
        if self.value("report:" + mid):
            return
        profile = report.get("execution_profile") or profile
        payload = {
            "outcome": report["outcome"],
            "execution_profile": profile,
            "message": (f"{report['summary']} Cleaning: mock. "
                        f"Navigation: {report['navigation_result']}; return: {report['return_result']}."),
            "failure_code": report["failure_code"] or None,
            "completed_tasks": report["completed_tasks"], "skipped_tasks": report["skipped_tasks"],
            "failed_task": report["failed_task"] or None,
            "needs_human_review": report["needs_human_review"],
            "before_observation": observation_reference(report["before_observation"], profile),
            "after_observation": observation_reference(report["after_observation"], profile),
        }
        with self.db:
            event = self._enqueue("mission.result", mid, payload)
            self.set("report:" + mid, event["event_id"])

    def status(self) -> dict:
        snapshot = self.snapshot
        if snapshot is None:
            raise RuntimeError("runtime snapshot is required")
        request = snapshot["active_request"]
        state = snapshot["robot_state"]
        if state == "IDLE" and not snapshot["ready"]:
            state = "ERROR"  # No offers until local readiness is confirmed.
        return {"boot_id": snapshot["boot_id"], "state": state,
                "active_mission_id": request["mission_id"] if request else None}

    def sync_event(self) -> dict:
        snapshot = self.snapshot
        request = snapshot["active_request"]
        reports = [json.loads(row[0]) for row in self.db.execute(
            "SELECT payload FROM events WHERE kind='mission.result' ORDER BY rowid DESC LIMIT 100"
        )]
        reports = [{key: report[key] for key in (
            "event_id", "mission_id", "sequence", "occurred_at", "payload",
        )} for report in reversed(reports)]
        return self.envelope("robot.snapshot", {
            **self.status(), "supported_seat_ids": snapshot["supported_seat_ids"],
            "can_cancel": True, "execution_profile": snapshot["execution_profile"],
            "active_phase": snapshot["phase"] if request else None,
            "active_sequence": int(self.value("sequence:" + request["mission_id"], "1"))
            if request else 0,
            "completed_reports": reports, "unaccepted_mission_ids": [],
        })

    def heartbeat(self) -> dict:
        return self.envelope("robot.heartbeat", self.status())

    def pending(self) -> list[dict]:
        return [json.loads(row[0]) for row in self.db.execute(
            "SELECT payload FROM events WHERE acknowledged=0 ORDER BY rowid"
        )]

    def acknowledge(self, event_id: str) -> None:
        with self.db:
            self.db.execute("UPDATE events SET acknowledged=1 WHERE event_id=?", (event_id,))

    def command_seen(self, event_id: str) -> bool:
        return self.db.execute("SELECT 1 FROM commands WHERE event_id=?", (event_id,)).fetchone() is not None

    def command_done(self, event: dict) -> dict:
        with self.db:
            self.db.execute("INSERT OR IGNORE INTO commands VALUES(?)", (event["event_id"],))
        return self.envelope("ack", {"ack_event_id": event["event_id"], "disposition": "applied"})

    @staticmethod
    def validate_command(event: dict) -> None:
        if (not isinstance(event, dict) or set(event) != {
                "schema_version", "event_id", "robot_id", "mission_id", "sequence",
                "occurred_at", "event_type", "payload",
        } or type(event.get("schema_version")) is not int
                or event.get("schema_version") != 1 or event.get("robot_id") != "cleany-01"
                or event.get("event_type") not in ("mission.offer", "mission.cancel", "sync.request", "ack")):
            raise ValueError("unsupported gateway command")
        if type(event["sequence"]) is not int or event["sequence"] != 0:
            raise ValueError("invalid sequence")
        stamp = datetime.fromisoformat(event["occurred_at"].replace("Z", "+00:00"))
        if stamp.tzinfo is None or stamp.utcoffset() is None:
            raise ValueError("occurred_at must have a timezone")
        UUID(event["event_id"])
        if event["event_type"].startswith("mission."):
            UUID(event["mission_id"])
        elif event.get("mission_id") is not None:
            raise ValueError("robot command must not have a mission_id")
        payload = event["payload"]
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        if event["event_type"] == "mission.offer":
            if (set(payload) != {"mission_type", "target_id", "requested_by"}
                    or payload["mission_type"] != "clean_seat"
                    or any(not isinstance(payload[key], str) or not payload[key].strip()
                           for key in ("target_id", "requested_by"))):
                raise ValueError("invalid mission offer")
        elif event["event_type"] == "mission.cancel" and payload:
            raise ValueError("cancel payload must be empty")
        elif event["event_type"] == "sync.request":
            interval = payload.get("heartbeat_interval_seconds")
            if (set(payload) != {"reason", "heartbeat_interval_seconds"}
                    or not isinstance(payload["reason"], str)
                    or type(interval) not in (int, float) or not isfinite(interval) or interval <= 0):
                raise ValueError("invalid sync request")
        elif event["event_type"] == "ack":
            if (set(payload) != {"ack_event_id", "disposition"}
                    or payload["disposition"] not in ("applied", "duplicate", "stale", "conflict")):
                raise ValueError("invalid acknowledgement")
            UUID(payload["ack_event_id"])

    def close(self) -> None:
        self.db.close()
