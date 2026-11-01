"""Serialized session resets; hardware operations live behind a narrow port."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
import threading
import uuid
from typing import Protocol
from zoneinfo import ZoneInfo


class Backend(Protocol):
    def devices(self) -> dict[str, bool]: ...
    def active(self) -> bool: ...
    def confirm_stop(self) -> bool: ...
    def stop(self) -> None: ...
    def start(self, output: Path) -> None: ...


class Busy(RuntimeError):
    pass


@dataclass
class SessionState:
    phase: str = "idle"
    message: str = "지도 초기화를 누르면 새 세션을 시작합니다."
    operation_id: str | None = None
    request_id: str | None = None
    generation: int = 0
    output: str | None = None
    previous_output: str | None = None
    stop_confirmed: bool | None = None


class SessionController:
    BUSY_PHASES = {"queued", "stopping", "waiting_usb", "starting"}

    def __init__(self, backend: Backend, output_root: Path) -> None:
        self.backend, self.output_root = backend, output_root
        self.lock = threading.RLock()
        self.state = SessionState()
        self.history: dict[str, str] = {}
        self.operation = "reset"
        self.pending = threading.Event()
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self._work, daemon=True)

    def snapshot(self) -> dict:
        with self.lock:
            return asdict(self.state)

    def update(self, **values) -> None:
        with self.lock:
            for name, value in values.items():
                setattr(self.state, name, value)

    def request_reset(self, request_id: str) -> dict:
        return self.request_operation("reset", request_id)

    def request_stop(self, request_id: str) -> dict:
        return self.request_operation("stop", request_id)

    def request_operation(self, operation: str, request_id: str) -> dict:
        if operation not in ("reset", "stop"):
            raise ValueError("지원하지 않는 매핑 작업입니다.")
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 80:
            raise ValueError("잘못된 초기화 요청 번호입니다.")
        key = operation + ":" + request_id
        with self.lock:
            if key in self.history:
                return {"operation_id": self.history[key], "duplicate": True}
            if self.state.phase in self.BUSY_PHASES:
                raise Busy("초기화를 진행 중입니다.")
            operation_id = str(uuid.uuid4())
            self.history[key] = operation_id
            # Keep deduplication for the lifetime of this service. No old retry can reset again.
            self.operation = operation
            self.state.phase = "queued"
            self.state.message = "초기화 요청을 받았습니다."
            self.state.operation_id, self.state.request_id = operation_id, request_id
            self.state.stop_confirmed = None
            self.pending.set()
            return {"operation_id": operation_id, "duplicate": False}

    def start(self) -> None:
        self.thread.start()

    def _work(self) -> None:
        while not self.closed.is_set():
            if not self.pending.wait(0.2):
                if self.snapshot()["phase"] == "running" and not self.backend.active():
                    self.update(
                        phase="failed",
                        message="SLAM 프로세스가 종료됐습니다. 기록을 확인하고 다시 초기화하세요.",
                    )
                continue
            self.pending.clear()
            if self.closed.is_set():
                return
            if self.operation == "stop":
                self.perform_stop()
            else:
                self.perform_reset()

    def perform_stop(self) -> None:
        try:
            self.update(
                phase="stopping", message="정지 응답을 확인하고 현재 기록을 저장합니다."
            )
            confirmed = self.backend.confirm_stop()
            self.update(stop_confirmed=confirmed)
            if not confirmed and self.backend.devices()["esp32"]:
                raise RuntimeError(
                    "STOP 응답을 확인하지 못했습니다. 현재 세션을 유지합니다."
                )
            self.backend.stop()
            previous = self.snapshot()["output"]
            self.update(
                phase="idle",
                message="지도와 기록 저장 완료",
                output=None,
                previous_output=previous or self.snapshot()["previous_output"],
            )
        except Exception as error:
            self.update(phase="failed", message=str(error))

    def perform_reset(self) -> None:
        """One operation at a time; exposed separately for deterministic lifecycle tests."""
        self.pending.clear()
        try:
            if (
                self.backend.active()
                or self.snapshot()["output"] is not None
                or getattr(self.backend, "has_control", lambda: False)()
            ):
                self.update(
                    phase="stopping",
                    message="정지 응답을 확인하고 현재 기록을 저장합니다.",
                )
                confirmed = self.backend.confirm_stop()
                self.update(stop_confirmed=confirmed)
                if not confirmed and self.backend.devices()["esp32"]:
                    raise RuntimeError(
                        "STOP 응답을 확인하지 못했습니다. 현재 세션을 유지합니다."
                    )
                self.backend.stop()
            previous = self.snapshot()["output"]
            self.update(
                output=None,
                previous_output=previous or self.snapshot()["previous_output"],
            )
            self.update(
                phase="waiting_usb",
                message="USB 재연결 대기 · LiDAR와 ESP32를 연결하면 새 원점에서 시작합니다.",
            )
            while not all(self.backend.devices().values()):
                if self.closed.wait(0.2):
                    return
            if self.closed.is_set():
                return
            stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%dT%H%M%S")
            output = self.output_root / f"hardware-slam-{stamp}-{uuid.uuid4().hex[:8]}"
            self.update(phase="starting", message="새 지도와 odom 원점을 준비합니다.")
            self.backend.start(output)
            current = self.snapshot()
            self.update(
                phase="running",
                message="초기화 완료 · 새 지도와 odom 원점에서 기록 중입니다.",
                output=str(output),
                generation=current["generation"] + 1,
            )
        except Exception as error:
            self.update(phase="failed", message=str(error))

    def close(self) -> None:
        self.closed.set()
        self.pending.set()
        if self.thread.is_alive():
            self.thread.join(timeout=60)
        if self.thread.is_alive():
            raise RuntimeError("초기화 작업 종료를 기다리는 중입니다.")
        if self.backend.active() or self.snapshot()["output"] is not None:
            self.backend.confirm_stop()
            self.backend.stop()
