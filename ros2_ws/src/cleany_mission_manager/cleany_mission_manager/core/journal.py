"""영속 수락 기록과 불변 종료 결과이며 ROS 의존성이 없습니다."""

import json
import sqlite3
from dataclasses import asdict
from pathlib import Path

from .runtime_models import RuntimeReport, RuntimeRequest


class MissionJournal:
    """임무 수락과 임무별 종료 결과 하나를 SQLite에 저장합니다.

    생성한 스레드에서 사용해야 합니다. 변경은 동기적으로 commit하며
    파일 시스템과 데이터베이스 오류는 호출자에게 전달합니다.
    """
    def __init__(self, path: str = ":memory:") -> None:
        if path != ":memory:":
            Path(path).expanduser().parent.mkdir(parents=True, exist_ok=True)
            path = str(Path(path).expanduser())
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.executescript(
            "CREATE TABLE IF NOT EXISTS missions ("
            "mission_id TEXT PRIMARY KEY, request TEXT NOT NULL, report TEXT);"
            "CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT);"
        )

    def get(self, mission_id: str) -> tuple[RuntimeRequest, RuntimeReport | None] | None:
        row = self.db.execute(
            "SELECT request,report FROM missions WHERE mission_id=?", (mission_id,)
        ).fetchone()
        if row is None:
            return None
        return RuntimeRequest(**json.loads(row[0])), (
            RuntimeReport.from_dict(json.loads(row[1])) if row[1] else None
        )

    def accept(self, request: RuntimeRequest, profile: dict | None = None) -> None:
        """새 요청과 실행 프로파일을 원자적으로 저장합니다.

        Raises:
            sqlite3.IntegrityError: 같은 mission ID가 이미 수락된 경우.
        """
        with self.db:
            self.db.execute("INSERT INTO missions VALUES(?,?,NULL)", (
                request.mission_id, json.dumps(asdict(request)),
            ))
            self.db.execute("INSERT OR REPLACE INTO metadata VALUES(?,?)", (
                "profile:" + request.mission_id, json.dumps(profile or {}),
            ))

    def profile(self, mission_id: str) -> dict:
        return json.loads(self.value("profile:" + mission_id, "{}"))

    def finish(self, report: RuntimeReport) -> None:
        """수락된 임무의 첫 종료 보고서를 저장 확정합니다.

        Raises:
            RuntimeError: 알 수 없는 임무이거나 이미 종료 결과가 있는 경우.
        """
        with self.db:
            updated = self.db.execute(
                "UPDATE missions SET report=? WHERE mission_id=? AND report IS NULL",
                (json.dumps(report.to_dict()), report.request.mission_id),
            ).rowcount
            if updated != 1:
                raise RuntimeError("terminal outcome is immutable")

    def unfinished(self) -> list[RuntimeRequest]:
        return [RuntimeRequest(**json.loads(row[0])) for row in self.db.execute(
            "SELECT request FROM missions WHERE report IS NULL"
        )]

    def latest(self) -> RuntimeReport | None:
        row = self.db.execute(
            "SELECT report FROM missions WHERE report IS NOT NULL ORDER BY rowid DESC LIMIT 1"
        ).fetchone()
        return RuntimeReport.from_dict(json.loads(row[0])) if row else None

    def reports(self, limit: int = 100) -> list[RuntimeReport]:
        return [RuntimeReport.from_dict(json.loads(row[0])) for row in self.db.execute(
            "SELECT report FROM missions WHERE report IS NOT NULL ORDER BY rowid DESC LIMIT ?", (limit,)
        )][::-1]

    def set(self, key: str, value: str) -> None:
        with self.db:
            self.db.execute("INSERT OR REPLACE INTO metadata VALUES(?,?)", (key, value))

    def value(self, key: str, default: str = "") -> str:
        row = self.db.execute("SELECT value FROM metadata WHERE key=?", (key,)).fetchone()
        return row[0] if row else default

    def close(self) -> None:
        self.db.close()
