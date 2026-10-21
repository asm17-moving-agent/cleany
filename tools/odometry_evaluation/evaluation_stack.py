"""Runtime supervisor for the wheel odometry evaluation ROS graph.

Importing this module is deliberately independent of rclpy; callers supply a
probe which owns ROS initialization.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any


def _positive(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number")


def _serial(value: Any, name: str) -> None:
    if not isinstance(value, str) or not value.startswith("/dev/") or not value.strip():
        raise ValueError(f"{name} must be a nonempty absolute /dev path")


def validate_runtime_config(data: dict, root: Path) -> dict:
    """Validate and resolve runtime stack configuration without touching devices."""
    if not isinstance(data, dict):
        raise ValueError("runtime config must be an object")
    expected = {"schema_version", "startup_timeout_s", "discovery_s", "shutdown_timeout_s",
                "agent", "base", "lidar"}
    if data.keys() != expected:
        raise ValueError(f"runtime config keys missing={sorted(expected-data.keys())}, unknown={sorted(data.keys()-expected)}")
    if type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise ValueError("schema_version must be integer 1")
    for key in ("startup_timeout_s", "discovery_s", "shutdown_timeout_s"):
        _positive(data[key], key)
    agent, base, lidar = data["agent"], data["base"], data["lidar"]
    if not isinstance(agent, dict) or agent.keys() != {"serial_port", "baudrate"}:
        raise ValueError("agent requires exactly serial_port and baudrate")
    _serial(agent["serial_port"], "agent.serial_port")
    if type(agent["baudrate"]) is not int or agent["baudrate"] <= 0:
        raise ValueError("agent.baudrate must be a positive integer")
    if not isinstance(base, dict) or base.keys() != {"config"}:
        raise ValueError("base requires exactly config")
    if not isinstance(base["config"], str) or not base["config"].strip():
        raise ValueError("base.config must be a nonempty path")
    config_path = (root / base["config"]).resolve()
    if not config_path.is_file():
        raise ValueError(f"base.config does not exist: {config_path}")
    lkeys = {"serial_port", "baudrate", "frame_id", "inverted", "angle_compensate",
             "scan_mode", "node_name", "restart_stale", "restart_node_names"}
    if not isinstance(lidar, dict) or lidar.keys() != lkeys:
        raise ValueError(f"lidar requires exactly {sorted(lkeys)}")
    _serial(lidar["serial_port"], "lidar.serial_port")
    if type(lidar["baudrate"]) is not int or lidar["baudrate"] <= 0:
        raise ValueError("lidar.baudrate must be a positive integer")
    for key in ("frame_id", "scan_mode", "node_name"):
        if not isinstance(lidar[key], str) or not lidar[key].strip():
            raise ValueError(f"lidar.{key} must be a nonempty string")
    for key in ("inverted", "angle_compensate", "restart_stale"):
        if type(lidar[key]) is not bool:
            raise ValueError(f"lidar.{key} must be boolean")
    names = lidar["restart_node_names"]
    if not isinstance(names, list) or not names or any(not isinstance(n, str) or not n.strip() for n in names) or len(set(names)) != len(names):
        raise ValueError("lidar.restart_node_names must be a unique nonempty list of strings")
    result = dict(data)
    result["base"] = {**base, "config": str(config_path)}
    return result


def describe_commands(config: dict, root: Path) -> dict:
    """Return argument vectors (never shell strings) for each stack component."""
    base_config = (root / config["base"]["config"]).resolve()
    return {
        "agent": ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial",
                  "--dev", config["agent"]["serial_port"], "-b", str(config["agent"]["baudrate"])],
        "base": ["ros2", "launch", "cleany_base_driver", "base_driver.launch.py",
                 "mock:=false", f"config:={base_config}"],
        "lidar": ["ros2", "run", "rplidar_ros", "rplidar_node", "--ros-args",
                  "-r", f"__node:={config['lidar']['node_name']}",
                  "-p", f"serial_port:={config['lidar']['serial_port']}",
                  "-p", f"serial_baudrate:={config['lidar']['baudrate']}",
                  "-p", f"frame_id:={config['lidar']['frame_id']}",
                  "-p", f"inverted:={str(config['lidar']['inverted']).lower()}",
                  "-p", f"angle_compensate:={str(config['lidar']['angle_compensate']).lower()}",
                  "-p", f"scan_mode:={config['lidar']['scan_mode']}"],
    }


def _proc_entries() -> list[tuple[int, str, str]]:
    entries = []
    for item in Path("/proc").iterdir():
        if not item.name.isdigit():
            continue
        try:
            if item.stat().st_uid != os.getuid():
                continue
            raw = (item / "cmdline").read_bytes().decode(errors="replace").rstrip("\0")
            exe = Path((item / "exe").resolve()).name
            entries.append((int(item.name), exe, raw))
        except (OSError, PermissionError):
            continue
    return entries


def _pid_identity(pid: int) -> str | None:
    """Stable /proc start-time token used to guard signals against PID reuse."""
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19]  # field 22 in proc(5); suffix begins at field 3
    except (OSError, IndexError):
        return None


def _group_owned(pgid: int, identity: str | None) -> bool:
    if identity is None:
        return False
    current_identity = _pid_identity(pgid)
    if current_identity == identity:
        return True
    if current_identity is not None:
        return False  # PID was reused; its new process group is not ours.
    # The ros2 wrapper may exit while children remain in its session. The
    # original session ID still identifies those descendants as our group.
    for item in Path("/proc").iterdir():
        if not item.name.isdigit():
            continue
        try:
            if item.stat().st_uid != os.getuid():
                continue
            fields = (item / "stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[3]) == pgid and int(fields[4]) == pgid:
                return True
        except (OSError, IndexError, ValueError):
            continue
    return False


class EvaluationStack:
    def __init__(self, config: dict, root: Path, log_dir: Path, probe: Any,
                 cancelled=lambda: False):
        self.config, self.root, self.log_dir = config, Path(root), Path(log_dir)
        self.probe, self.cancelled = probe, cancelled
        self.processes: list[tuple[subprocess.Popen, int, str | None]] = []
        self.restarted: list[int] = []
        self._closed = False
        self.commands = describe_commands(config, self.root)

    def _check_cancelled(self) -> None:
        if self.cancelled():
            raise RuntimeError("evaluation stack startup cancelled")

    @staticmethod
    def _serial_match(cmd: str, path: str) -> bool:
        candidates = {path}
        try:
            candidates.add(str(Path(path).resolve(strict=True)))
        except OSError:
            pass
        args = cmd.split("\0")
        for i, arg in enumerate(args):
            value = arg.split(":=", 1)[1] if arg.startswith("serial_port:=") else None
            if arg == "--dev" and i + 1 < len(args):
                value = args[i + 1]
            if arg.startswith("--dev="):
                value = arg[6:]
            if value in candidates:
                return True
        return False

    def _matching(self, exe: str, path: str) -> list[int]:
        return [pid for pid, basename, cmd in _proc_entries()
                if basename == exe and self._serial_match(cmd, path)]

    def _device(self, path: str) -> None:
        if not Path(path).exists():
            raise RuntimeError(f"configured serial device missing: {path}")

    def _launch(self, component: str) -> None:
        self._check_cancelled()
        if component in ("agent", "lidar"):
            self._device(self.config[component]["serial_port"])
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log = (self.log_dir / f"{component}.log").open("ab")
        try:
            process = subprocess.Popen(self.commands[component], cwd=self.root,
                env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        finally:
            log.close()
        self.processes.append((process, process.pid, _pid_identity(process.pid)))
        print(f"Started {component}: pid={process.pid}, log={self.log_dir / (component + '.log')}",
              flush=True)

    def _stop_pid(self, pid: int, identity: str | None) -> None:
        if identity is None or _pid_identity(pid) != identity:
            raise RuntimeError("configured stale LiDAR process identity changed; refusing signal")
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + min(2.0, self.config["shutdown_timeout_s"])
        while time.monotonic() < deadline:
            if _pid_identity(pid) != identity:
                return
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.05)
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if _pid_identity(pid) != identity:
                return
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                return
            until = time.monotonic() + .5
            while time.monotonic() < until and _pid_identity(pid) == identity:
                time.sleep(.05)
            if _pid_identity(pid) != identity:
                return
        raise RuntimeError(f"stale LiDAR pid {pid} did not exit; refusing duplicate driver")

    def _wait(self, started: float) -> dict:
        deadline = started + self.config["startup_timeout_s"]
        while True:
            self._check_cancelled()
            for process, _, _ in self.processes:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(f"owned stack process pid={process.pid} exited early "
                                       f"with status {code}; see {self.log_dir}")
            self.probe.spin(min(.2, max(0, deadline-time.monotonic())))
            state = self.probe.snapshot()
            if state.get("publishers", {}).get("cmd_vel", 0) > 0:
                raise RuntimeError("refusing stack startup while /cmd_vel has a publisher")
            if state.get("ready") and state.get("odom_fresh") and state.get("scan_fresh") and state.get("wheel_fresh"):
                return state
            if state.get("errors"):
                blockers = "; ".join(state["errors"])
            else:
                blockers = "stack data not fresh"
            if time.monotonic() >= deadline:
                if not state.get("publishers", {}).get("wheel_state", 0):
                    blockers += ("; MCU WheelState publisher absent. Check the motor-controller "
                                 "firmware (the standalone IMU calibration image does not publish "
                                 f"WheelState); Agent log: {self.log_dir / 'agent.log'}")
                raise RuntimeError(f"runtime stack startup timed out: {blockers}")

    def start(self) -> dict:
        started = time.monotonic()
        try:
            self._check_cancelled()
            self.probe.spin(self.config["discovery_s"])
            state = self.probe.snapshot()
            pubs = state.get("publishers", {})
            if pubs.get("cmd_vel", 0):
                raise RuntimeError("refusing stack startup while /cmd_vel has a publisher")
            if any(pubs.get(key, 0) > 1 for key in ("odom", "scan", "wheel_state", "wheel_command")):
                raise RuntimeError("multiple publishers found for a runtime stack topic")
            entries = _proc_entries()
            agent_pids = self._matching("micro_ros_agent", self.config["agent"]["serial_port"])
            lidar_pids = self._matching("rplidar_node", self.config["lidar"]["serial_port"])
            stale_lidar_identity = _pid_identity(lidar_pids[0]) if lidar_pids else None
            if len(agent_pids) > 1 or len(lidar_pids) > 1:
                raise RuntimeError("multiple configured serial processes found")
            stale_lidar_pid = None
            if not state.get("scan_fresh"):
                if lidar_pids:
                    matched = [e for e in entries if e[0] == lidar_pids[0]]
                    tokens = matched[0][2].split("\0") if matched else []
                    identifiable = any(token == f"__node:={name}" for token in tokens
                                       for name in self.config["lidar"]["restart_node_names"])
                    if not (self.config["lidar"]["restart_stale"] and identifiable):
                        raise RuntimeError("stale LiDAR process is not an identifiable configured restart target")
                    stale_lidar_pid = lidar_pids[0]
                elif pubs.get("scan", 0):
                    raise RuntimeError("stale /scan publisher has no identifiable configured LiDAR process")
            driver_exists = pubs.get("wheel_command", 0) > 0 or state.get("enable_available")
            if not agent_pids and not pubs.get("wheel_state", 0):
                self._launch("agent")
            if not driver_exists and not pubs.get("odom", 0):
                self._launch("base")
            if not state.get("scan_fresh"):
                if stale_lidar_pid is not None:
                    if pubs.get("cmd_vel", 0):
                        raise RuntimeError("refusing LiDAR restart while /cmd_vel has a publisher")
                    self._stop_pid(stale_lidar_pid, stale_lidar_identity)
                    self.restarted.append(stale_lidar_pid)
                self._launch("lidar")
            final = self._wait(started)
            return {"commands": self.commands, "started": [p.pid for p, _, _ in self.processes],
                    "restarted_lidar_pids": list(self.restarted), "snapshot": final}
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for process, pgid, identity in reversed(self.processes):
            for sig, timeout in ((signal.SIGINT, self.config["shutdown_timeout_s"]),
                                 (signal.SIGTERM, .5), (signal.SIGKILL, .5)):
                process.poll()  # Reap the wrapper before inspecting descendants.
                if not _group_owned(pgid, identity):
                    break
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    process.poll()
                    if not _group_owned(pgid, identity):
                        break
                    time.sleep(.05)
