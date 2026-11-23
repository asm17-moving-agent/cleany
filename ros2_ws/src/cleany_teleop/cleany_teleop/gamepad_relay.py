"""Keep socket replies off the input thread so stop can bypass pending updates."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import socket
import time
import uuid

from cleany_teleop.web_control import Control


@dataclass(frozen=True)
class Request:
    action: str
    sequence: int
    sent_at: float
    deadline: float
    speed: float
    vector: list | None


class GamepadRelay:
    """Manage controller state on the ROS thread and socket round trips on workers."""

    def __init__(self, control, path, warn):
        self.control = control
        self.path = path
        self.warn = warn
        self.client = uuid.uuid4().hex
        # Two in-flight updates allow a delayed reply without blocking the next update or stop.
        self.workers = ThreadPoolExecutor(max_workers=3, thread_name_prefix="pad-relay")
        self.pending = {}
        self.sequence = self.barrier = self.last_ack = 0
        self.confirmed_until = None
        self.retry_after = 0.0

    def exchange(self, request):
        command = dict(action=request.action, client=self.client,
                       sequence=request.sequence, speed=request.speed)
        if request.vector is not None:
            command["vector"] = request.vector
        payload = json.dumps(dict(command=command, expires_at=request.deadline)).encode() + b"\n"
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(Control.LEASE_SECONDS)
            sock.connect(self.path)
            sock.sendall(payload)
            with sock.makefile("rb") as stream:
                reply = json.loads(stream.readline(4096))
        if not isinstance(reply, dict) or type(reply.get("ok")) is not bool:
            raise ValueError("잘못된 조종 중계 응답")
        if not reply["ok"]:
            if reply.get("discarded") is True:
                raise TimeoutError(reply.get("error", "만료된 패드 입력"))
            raise ValueError(reply.get("error", "패드 요청 거부"))
        if type(reply.get("forward_only")) is not bool:
            raise ValueError("중계 응답에 주행 제한이 없습니다")
        return reply["forward_only"]

    def send(self, result, now):
        if result is None:
            return
        action, vector = result
        self.sequence += 1
        deadline = self.control.deadline
        if action == "stop":
            self.barrier = self.sequence
            self.confirmed_until = None
            deadline = now + self.control.mapping.max_age
        request = Request(action, self.sequence, now, deadline, self.control.command_speed, vector)
        self.pending[self.workers.submit(self.exchange, request)] = request

    def collect(self, now):
        for future, request in list(self.pending.items()):
            if not future.done():
                continue
            del self.pending[future]
            if request.sequence < self.barrier:
                continue
            try:
                forward_only = future.result()
            except (OSError, ValueError) as exc:
                self.warn(f"패드 {request.action} #{request.sequence}: {exc}")
                if isinstance(exc, TimeoutError) and request.action == "update":
                    continue  # Use the next physical report instead of replaying the old vector.
                if request.action == "press":
                    self.retry_after = now + Control.LEASE_SECONDS
                self.send(self.control.interrupt(now), now)
                continue
            if request.action != "stop" and request.sequence > self.last_ack:
                self.last_ack = request.sequence
                self.control.forward_only = forward_only
                self.confirmed_until = request.sent_at + Control.LEASE_SECONDS
        if self.confirmed_until is not None and now >= self.confirmed_until:
            self.warn("패드 조종권 갱신 응답이 끊겼습니다")
            self.send(self.control.interrupt(now), now)

    def sample(self, axes, buttons, stamp, age, now):
        self.collect(now)
        result = self.control.sample(
            axes, buttons, stamp, age, now,
            can_press=not self.pending and now >= self.retry_after,
            can_update=self.confirmed_until is not None and len(self.pending) < 2,
        )
        self.send(result, now)

    def tick(self, now):
        self.collect(now)
        self.send(self.control.timeout(now), now)

    def close(self):
        now = time.monotonic()
        self.send(self.control.interrupt(now), now)
        self.workers.shutdown(wait=True)
