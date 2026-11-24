"""Exercise the Jetson Backend and ROS bridge on loopback with an isolated mock Runtime."""

import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import socket
import sqlite3
import subprocess
import sys
import tempfile
import time
from contextlib import ExitStack
from uuid import uuid4


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, required=True)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error("choose an unused test domain different from the physical domain 0")
    backend = args.release.resolve() / "apps/backend"
    python = backend / ".venv/bin/python"
    sys.path.insert(0, str(backend))
    import httpx
    from control_plane.server import ControlPlaneApplication
    from control_plane.settings import Settings

    with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
        root = Path(folder)
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        url = f"http://127.0.0.1:{port}"
        db = str(root / "mock.db")
        settings = Settings(database_path=db, robot_mode="gateway", cookie_secure=False)
        app = ControlPlaneApplication(settings=settings)
        customer = app.accounts.create_customer("Isolated ROS rehearsal")
        site = app.accounts.create_site(customer, "Test site", site_id="rehearsal")
        _, password = app.accounts.provision(customer, "rehearsal", "Test operator")
        temporary = app.accounts.login("rehearsal", password, "loopback")
        session = app.accounts.change_password(temporary, secrets.token_urlsafe(40))
        csrf = app.accounts.authenticate(session).csrf_token
        token = root / "device.token"
        token.touch(mode=0o600)
        token.write_text(app.accounts.register_robot(customer, site))
        app.store.repository.close()
        env = {k: v for k, v in os.environ.items() if not k.startswith("CLEANY_")}
        env.update(
            ROS_DOMAIN_ID=str(args.domain),
            ROS_LOCALHOST_ONLY="1",
            ROS_LOG_DIR=str(root / "ros-logs"),
            CLEANY_DATABASE_PATH=db,
            CLEANY_HOST="127.0.0.1",
            CLEANY_PORT=str(port),
            CLEANY_ROBOT_MODE="gateway",
            CLEANY_COOKIE_SECURE="false",
            CLEANY_ALLOWED_ORIGINS=json.dumps([url]),
        )

        def start(cmd, name):
            log = stack.enter_context((root / (name + ".log")).open("w+"))
            proc = subprocess.Popen(
                cmd, env=env, cwd=root, start_new_session=True, stdout=log, stderr=subprocess.STDOUT
            )

            def close():
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(10)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()

            stack.callback(close)
            return proc, log

        backend_command = [str(python), str(backend / "run.py")]
        processes = [start(backend_command, "backend")]
        client = stack.enter_context(
            httpx.Client(
                base_url=url,
                trust_env=False,
                timeout=3,
                cookies={"cleany_session": session},
                headers={"Origin": url, "X-CSRF-Token": csrf},
                params={"site_id": site},
            )
        )

        def wait(read):
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                for proc, log in processes:
                    if proc.poll() is not None:
                        log.seek(0)
                        raise RuntimeError(log.read()[-2000:])
                try:
                    value = read()
                    if value:
                        return value
                except httpx.ConnectError:
                    pass
                time.sleep(0.15)
            raise RuntimeError("Timed out waiting for " + read.__name__)

        wait(lambda: client.get("/api/health").is_success)
        targets = (
            Path(__file__).resolve().parents[2]
            / "ros2_ws/src/cleany_mission_manager/config/full_facility_targets.yaml"
        )
        runtime_command = [
            "ros2",
            "launch",
            "cleany_mission_manager",
            "mission_runtime.launch.py",
            "navigation_backend:=mock",
            "supervision_mode:=disabled",
            "use_sim_time:=false",
            f"targets_file:={targets}",
            f"device_token_file:={token}",
            f"gateway_url:=ws://127.0.0.1:{port}/api/robots/cleany-01/gateway/ws",
            f"journal_path:={root/'mission.db'}",
            f"bridge_journal_path:={root/'bridge.db'}",
        ]
        processes.append(start(runtime_command, "runtime"))

        def seats_ready():
            seats = client.get("/api/seats").raise_for_status().json()["items"]
            return {"seat-02", "seat-05"} <= {s["seat_id"] for s in seats if s["mission_supported"]}

        wait(seats_ready)
        reports = []
        for seat in ("seat-02", "seat-05"):
            payload = dict(seat_id=seat, priority="NORMAL", idempotency_key=str(uuid4()))
            posted = client.post("/api/missions", json=payload).raise_for_status().json()
            mid = posted["mission_id"]
            retry = client.post("/api/missions", json=payload).raise_for_status().json()
            assert retry["mission_id"] == mid

            def terminal():
                item = client.get("/api/missions/" + mid).raise_for_status().json()
                return item if item["phase"] == "TERMINAL" else None

            item = wait(terminal)
            assert item["outcome"] == "SUCCESS" and set(item["execution_profile"].values()) == {"mock"}
            assert not item["before_observation"] and not item["after_observation"]
            with sqlite3.connect(f"file:{root/'mission.db'}?mode=ro", uri=True) as journal:
                row = journal.execute(
                    "SELECT report FROM missions WHERE mission_id=?", (mid,)
                ).fetchone()
                report = json.loads(row[0])
            assert report["navigation_result"] == report["return_result"] == "OK"
            reports.append(
                dict(
                    seat=seat,
                    mission_id=mid,
                    outcome=item["outcome"],
                    navigation=report["navigation_result"],
                    return_home=report["return_result"],
                    profile=item["execution_profile"],
                )
            )
        result = dict(domain=args.domain, physical_motion=False, missions=reports)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))


if __name__ == "__main__":
    main()
