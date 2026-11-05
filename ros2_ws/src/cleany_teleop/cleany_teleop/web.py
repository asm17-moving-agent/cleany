#!/usr/bin/env python3
"""LAN web remote for the existing USB micro-ROS firmware."""

from __future__ import annotations

import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import mimetypes
import os
from pathlib import Path
import secrets
import signal
import sys
import socket
import threading
import time
from types import SimpleNamespace
from urllib.parse import unquote, urlsplit

from cleany_teleop.core import newer
from cleany_teleop.web_control import Control


def ui_asset(ui_root: Path, request_path: str) -> tuple[bytes, str] | None:
    """Serve only existing compiled assets inside the UI asset directory."""
    path = unquote(urlsplit(request_path).path)
    if not path.startswith("/assets/"):
        return None
    # colcon --symlink-install links each asset through the trusted entrypoint's tree.
    asset_root = (ui_root / "index.html").resolve().parent / "assets"
    asset = (ui_root / path.lstrip("/")).resolve()
    if not asset.is_relative_to(asset_root) or not asset.is_file():
        return None
    content_type = mimetypes.guess_type(asset.name)[0] or "application/octet-stream"
    return asset.read_bytes(), content_type


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="UI preview only; never connects to ROS/ESP32",
    )
    parser.add_argument(
        "--paused",
        action="store_true",
        help="start with motion input blocked for SLAM maintenance",
    )
    parser.add_argument(
        "--max-speed", type=float, default=10.0, help="web wheel speed ceiling in rad/s"
    )
    parser.add_argument(
        "--base-config", type=Path, help="reviewed base driver geometry/limits profile"
    )
    parser.add_argument(
        "--token-file",
        type=Path,
        help="keep the same private access key across server restarts",
    )
    parser.add_argument(
        "--event-log",
        type=Path,
        default=Path("~/.local/state/cleany/teleop/events.jsonl"),
    )
    parser.add_argument("--event-log-max-bytes", type=int, default=2_000_000)
    parser.add_argument("--event-log-backups", type=int, default=3)
    parser.add_argument("--mapping-status-timeout", type=float, default=1.0)
    cli = sys.argv[1:]
    if "--ros-args" in cli:
        from rclpy.utilities import remove_ros_args

        cli = remove_ros_args(sys.argv)[1:]
    args = parser.parse_args(cli)
    if not math.isfinite(args.max_speed) or not 0.5 <= args.max_speed <= 10:
        parser.error("max-speed must be finite and in [0.5, 10] rad/s")
    if args.event_log_max_bytes <= 0 or args.event_log_backups < 1:
        parser.error("event log bounds must be positive")
    if (
        not math.isfinite(args.mapping_status_timeout)
        or args.mapping_status_timeout <= 0
    ):
        parser.error("mapping-status-timeout must be finite and positive")
    token = secrets.token_urlsafe(32)
    if args.token_file:
        args.token_file = args.token_file.expanduser().resolve()
        args.token_file.parent.mkdir(parents=True, exist_ok=True)
        if args.token_file.exists():
            token = args.token_file.read_text().strip()
            if len(token) < 32 or not token.isascii():
                parser.error("token file must contain at least 32 ASCII characters")
        else:
            with os.fdopen(
                os.open(args.token_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600),
                "w",
            ) as file:
                file.write(token)
    lock = threading.RLock()
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    from ament_index_python.packages import get_package_share_directory

    ui_root = Path(get_package_share_directory("cleany_teleop")) / "ui/dist"
    if not (ui_root / "index.html").is_file():
        parser.error(
            "조종 UI 빌드가 필요합니다: cd tools/esp32_motion/ui && npm ci && npm run build"
        )
    html = (ui_root / "index.html").read_bytes()
    from cleany_teleop.event_journal import EventJournal

    journal = EventJournal(
        args.event_log,
        max_bytes=args.event_log_max_bytes,
        backups=args.event_log_backups,
    )
    node = None
    synthetic_profile = False
    mapping = None
    tick_status = lambda: None
    if not args.mock:
        import rclpy
        import yaml
        from cleany_teleop.ros_transport import bind

        if args.base_config is None:
            parser.error("--base-config must match the active base driver profile")
        profile = yaml.safe_load(args.base_config.read_text())
        synthetic_profile = profile.get("mode") == "synthetic"
        from cleany_base_driver.core import Geometry, Limits

        g, lim = profile["geometry"], profile["limits"]
        Geometry(g["wheel_radius_m"], g["wheelbase_m"], g["wheel_separation_m"])
        Limits(
            *(
                lim[k]
                for k in (
                    "linear_x_mps",
                    "linear_y_mps",
                    "angular_z_rad_s",
                    "wheel_rad_s",
                    "command_timeout_s",
                )
            )
        )
        rclpy.init()
        signal.signal(signal.SIGINT, lambda *_: interrupted.set())
        signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
        node = rclpy.create_node("phone_remote")
        control, tick_status, mapping = bind(
            node,
            profile,
            lock,
            event_sink=journal.emit,
            mapping_status_timeout_s=args.mapping_status_timeout,
        )
    else:
        started = time.monotonic()
        mock = SimpleNamespace(
            protocol_version=2,
            boot_id=123,
            sequence=0,
            timestamp_us=1_000_000,
            last_command_sequence=0,
            watchdog_ms=250,
            max_velocity_rad_s=10.0,
            command_age_ms=0,
            enabled=False,
            fault_bits=0,
            target_rad_s=[0.0] * 4,
            commanded_rad_s=[0.0] * 4,
            velocity_rad_s=[0.0] * 4,
            pwm_percent=[0] * 4,
        )

        def publish(command):
            if command.mode == Control.STOP:
                mock.enabled = False
                mock.target_rad_s = mock.commanded_rad_s = mock.velocity_rad_s = [
                    0.0
                ] * 4
            elif command.mode == Control.ENABLE:
                mock.enabled = True
            elif mock.enabled:
                mock.target_rad_s = mock.commanded_rad_s = mock.velocity_rad_s = list(
                    command.velocity_rad_s
                )
            if newer(command.sequence, mock.last_command_sequence):
                mock.last_command_sequence = command.sequence

        control = Control(publish, event_sink=journal.emit)
        control.subscriber_count = 1

    control.MAX_WEB_VELOCITY = args.max_speed
    control.paused = args.paused

    class RemoteServer(ThreadingHTTPServer):
        # Multiple phones/tabs can reconnect together. The stdlib default is 5.
        request_queue_size = 128

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.settimeout(2.0)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def log_message(self, *_):
            pass

        def respond(self, code, body, content_type="application/json"):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            if self.close_connection:
                self.send_header("Connection", "close")
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'",
            )
            self.end_headers()
            self.wfile.write(body)

        def authorized(self):
            return hmac.compare_digest(self.headers.get("X-Control-Token", ""), token)

        def do_GET(self):
            if urlsplit(self.path).path == "/":
                self.respond(200, html, "text/html; charset=utf-8")
            elif urlsplit(self.path).path.startswith("/assets/"):
                asset = ui_asset(ui_root, self.path)
                if asset is None:
                    self.respond(404, {"error": "not found"})
                else:
                    self.respond(200, *asset)
            elif self.path == "/api/mapping/status":
                if not self.authorized():
                    self.respond(403, {"error": "접속 링크의 조종 키가 필요합니다"})
                else:
                    self.respond(
                        200,
                        mapping.status()
                        if mapping
                        else {"phase": "unavailable", "message": "모의 조종 모드"},
                    )
            elif self.path == "/api/status":
                if not self.authorized():
                    self.respond(403, {"error": "접속 링크의 조종 키가 필요합니다"})
                    return
                with lock:
                    status = control.status()
                status["mock"] = args.mock or synthetic_profile
                status["event_log"] = journal.status()
                self.respond(200, status)
            else:
                self.respond(404, {"error": "not found"})

        def do_POST(self):
            if self.path not in ("/api/control", "/api/reset"):
                self.respond(404, {"error": "not found"})
                return
            if not self.authorized() or self.headers.get(
                "Origin"
            ) != "http://" + self.headers.get("Host", ""):
                self.close_connection = True
                self.respond(
                    403, {"error": "조종 키 또는 접속 주소가 올바르지 않습니다"}
                )
                return
            data = None
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4096:
                    raise ValueError("잘못된 요청 크기")
                self.connection.settimeout(0.5)
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError("잘못된 요청")
                if self.path == "/api/reset":
                    if mapping is None:
                        raise ValueError("매핑 관리자가 연결되지 않았습니다.")
                    self.respond(202, mapping.request_reset(data.get("request_id")))
                    return
                if data.get("action") != "stop" and data.get("ui_version") != 2:
                    raise ValueError(
                        "조종 화면이 업데이트되었습니다. 페이지를 새로고침해주세요."
                    )
                with lock:
                    control.request(data)
                    status = control.status()
                self.respond(200, status)
            except (ValueError, RuntimeError, TimeoutError) as error:
                self.close_connection = True
                with lock:
                    client = data.get("client") if isinstance(data, dict) else None
                    latest = (
                        control.requests.get(client, -1)
                        if isinstance(client, str)
                        else -1
                    )
                self.respond(
                    409,
                    {
                        "error": str(error),
                        "latest_sequence": latest,
                        "stale_request": str(error) == "이전 요청은 처리하지 않습니다",
                    },
                )

    server = RemoteServer((args.host, args.port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(
        f"{'MOCK preview' if args.mock else 'ESP32 remote'}: http://127.0.0.1:{server.server_port}/#token={token}",
        flush=True,
    )
    print(
        "Phone: replace 127.0.0.1 with the laptop Wi-Fi IPv4; retain #token=...",
        flush=True,
    )
    next_tick = time.monotonic()
    try:
        while not interrupted.is_set():
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.005)
            else:
                time.sleep(0.005)
            if time.monotonic() >= next_tick:
                with lock:
                    if args.mock:
                        mock.sequence += 1
                        mock.timestamp_us = 1_000_000 + int(
                            (time.monotonic() - started) * 1e6
                        )
                        control.feedback(mock)
                    tick_status()
                    control.tick()
                next_tick = time.monotonic() + 0.02
    finally:
        with lock:
            control.stop("웹 서버 종료")
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.02)
            else:
                time.sleep(0.02)
            with lock:
                control.tick()
        server.shutdown()
        server.server_close()
        journal.close()
        if journal.status()["error"]:
            print(f"Teleop journal shutdown: {journal.status()}", file=sys.stderr)
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
