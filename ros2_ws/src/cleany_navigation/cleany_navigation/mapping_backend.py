"""Installed-workspace mapping lifecycle; permanent hardware is owned by bringup."""

from __future__ import annotations
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from .mapping_process import identity, descendants, live_recorded
from .mapping_storage import atomic_json, planar_origin, save_grid


class LoggerPort:
    """Use the mapping executor's ROS clients; no nested contexts or executors."""

    def __init__(self, backend):
        self.backend = backend

    def call(self, operation, *, settings=None, session_id="", revision=0):
        from cleany_interfaces.srv import RecordingControl
        from cleany_logger.wire import config_dict, config_message, status_dict
        import uuid

        request = RecordingControl.Request(
            request_id=uuid.uuid4().hex,
            session_id=session_id,
            expected_revision=revision,
        )
        if settings is not None:
            request.config = config_message(settings)
        response = self.backend.rpc(
            self.backend.node.logger_clients[operation], request, timeout=8.0
        )
        if not response.success:
            raise RuntimeError(response.reason)
        view = status_dict(response.status)
        view["settings"] = config_dict(response.config)
        view["settings"]["active"] = view["state"] == "RECORDING"
        return view

    def close(self):
        pass


class MappingBackend:
    def __init__(self, node, profile, profile_path: Path, output_root: Path):
        self.node, self.profile = node, profile
        self.profile_path, self.output_root = profile_path, output_root
        self.lock = threading.RLock()
        self.process = self.logger_process = None
        self.output = None
        self.logs = []
        self.teleop = self.scan = self.odom = self.grid = None
        self.scan_at = self.odom_at = self.teleop_at = self.grid_at = 0.0
        self.started = 0.0
        self.ready_once = False
        self.write_lock = threading.Lock()
        self.storage_lock = threading.Lock()
        self.write_future = None
        self.logger_session = None
        self.logger = None
        self.save_error = None
        self.saving_final = False

    def has_control(self):
        return True

    def devices(self):
        now = time.monotonic()
        with self.lock:
            return {
                "lidar": self.scan is not None and now - self.scan_at < 0.5,
                "esp32": self.teleop is not None
                and now - self.teleop_at < 0.5
                and self.teleop.connected,
            }

    def active(self):
        return (
            self.process is not None
            and self.process.poll() is None
            and self.logger_process is not None
            and self.logger_process.poll() is None
        )

    def inactive_reason(self):
        failures = []
        for name, process in (
            ("cartographer", self.process),
            ("logger", self.logger_process),
        ):
            if process is None or process.poll() is not None:
                code = None if process is None else process.returncode
                failures.append(
                    f"{name} exited (code={code}, log={self.output}/{name}.log)"
                )
        return "; ".join(failures) or "Mapping readiness lost"

    def rpc(self, client, request, timeout=4.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f"Service response timed out: {client.srv_name}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"Empty service response: {client.srv_name}")
        return result

    def pause(self, enabled):
        from std_srvs.srv import SetBool

        response = self.rpc(self.node.pause_client, SetBool.Request(data=enabled))
        if not response.success:
            raise RuntimeError(response.message)

    def confirm_stop(self):
        try:
            self.pause(True)
            end = time.monotonic() + 2
            while time.monotonic() < end:
                with self.lock:
                    state = self.teleop
                    if (
                        state
                        and time.monotonic() - self.teleop_at < 0.2
                        and state.connected
                        and state.paused
                        and state.stop_confirmed
                        and not state.enabled
                    ):
                        return True
                time.sleep(0.025)
        except (RuntimeError, TimeoutError):
            pass
        return False

    def spawn(self, name, command):
        stream = (self.output / (name + ".log")).open("ab")
        self.logs.append(stream)
        return subprocess.Popen(
            command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True
        )

    @staticmethod
    def terminate(process, timeout=12):
        if process is None:
            return
        owned = descendants(process.pid)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            # Leave ownership and artifacts intact; never start a replacement over it.
            raise RuntimeError(
                "Owned mapping child did not exit; restart blocked"
            ) from error
        if live_recorded(owned):
            raise RuntimeError("Owned mapping descendants remain; restart blocked")
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            return
        raise RuntimeError("Owned mapping descendants remain; restart blocked")

    def periodic_save(self):
        if (
            self.output is None
            or not self.active()
            or not self.write_lock.acquire(blocking=False)
        ):
            return
        try:
            from cartographer_ros_msgs.srv import WriteState

            if not self.node.write_client.service_is_ready():
                self.write_lock.release()
                return
            output = self.output
            target = output / (".periodic-" + str(time.time_ns()) + ".pbstream")
            future = self.node.write_client.call_async(
                WriteState.Request(
                    filename=str(target), include_unfinished_submaps=True
                )
            )
            self.write_future = future

            def complete(future):
                try:
                    response = future.result()
                    if (
                        response is None
                        or response.status.code != 0
                        or not target.is_file()
                    ):
                        raise RuntimeError("Periodic pbstream save failed")
                    os.replace(target, output / "latest.pbstream")
                except Exception as error:
                    self.save_error = str(error)
                finally:
                    self.write_lock.release()

            future.add_done_callback(complete)
        except Exception:
            self.write_lock.release()
            raise

    def save(self):
        self.saving_final = True
        pending = getattr(self.node, "snapshot_future", None)
        if pending is not None:
            pending.result(timeout=5)
        from cartographer_ros_msgs.srv import WriteState

        if not self.write_lock.acquire(timeout=8):
            raise RuntimeError("Previous pbstream write is incomplete")
        try:
            target = self.output / (".final-" + str(time.time_ns()) + ".pbstream")
            response = self.rpc(
                self.node.write_client,
                WriteState.Request(
                    filename=str(target), include_unfinished_submaps=True
                ),
                timeout=8.0,
            )
            if response.status.code != 0 or not target.is_file():
                raise RuntimeError("Final pbstream save failed; preserving session")
            os.replace(target, self.output / "latest.pbstream")
            with self.lock:
                if self.grid is None or self.grid_at < self.started:
                    raise RuntimeError("No map produced by this session")
                grid = self.grid
                with self.storage_lock:
                    save_grid(
                        self.output,
                        grid.info.width,
                        grid.info.height,
                        grid.info.resolution,
                        planar_origin(grid.info.origin),
                        grid.data,
                    )
        finally:
            self.write_lock.release()

    def stop(self):
        if self.output is None:
            return
        if self.process is not None:
            # Save errors retain SLAM and logger for an explicit retry.
            if (
                self.ready_once
                and self.grid_at >= self.started
                and self.process.poll() is None
            ):
                self.save()
            elif self.ready_once and self.grid_at >= self.started:
                raise RuntimeError(
                    "SLAM exited before final save; session requires recovery"
                )
            self.record_children()
            self.stop_logger()
            self.terminate(self.process)
            self.process = None
        else:
            self.stop_logger()
        atomic_json(
            self.output / "shutdown.json",
            dict(
                stopped=True,
                final_map=(self.output / "map.yaml").exists(),
                pbstream=(self.output / "latest.pbstream").exists(),
                logger_session=self.logger_session,
            ),
        )
        for stream in self.logs:
            stream.close()
        self.logs = []

    def stop_logger(self):
        if self.logger_process is not None and self.logger_process.poll() is not None:
            if self.logger_session is not None:
                raise RuntimeError(
                    "Logger exited before final flush; recording requires recovery"
                )
            self.logger = None
        if self.logger is not None:
            state = self.logger.call("get_config")
            if state["state"] == "RECORDING":
                if state["session_id"] != self.logger_session:
                    raise RuntimeError("Logger session ownership changed; stop blocked")
                self.logger.call(
                    "stop", session_id=self.logger_session, revision=state["revision"]
                )
            elif state["state"] != "IDLE":
                raise RuntimeError("Logger flush failed")
            self.logger.close()
            self.logger = None
        self.terminate(self.logger_process)
        self.logger_process = None

    def record_children(self):
        processes = {
            name: process
            for name, process in (
                ("slam", self.process),
                ("logger", self.logger_process),
            )
            if process is not None
        }
        tokens = {
            str(process.pid): identity(process.pid) for process in processes.values()
        }
        for process in processes.values():
            tokens.update(descendants(process.pid))
        atomic_json(
            self.output / "processes.json",
            {
                "children": {name: p.pid for name, p in processes.items()},
                "identities": tokens,
            },
        )

    def start(self, output: Path):
        if self.write_future is not None and not self.write_future.done():
            raise RuntimeError(
                "Previous pbstream request is unresolved; restart blocked"
            )
        if self.process is not None or self.logger_process is not None:
            raise RuntimeError("Previous session still owns children")
        from std_srvs.srv import Trigger

        output.mkdir(parents=True, exist_ok=False)
        self.output = output
        self.started = time.monotonic()
        self.started_ros = self.node.get_clock().now().nanoseconds
        self.ready_once = False
        self.saving_final = False
        self.logger_session = None
        self.grid = None
        self.grid_at = 0.0
        atomic_json(
            output / "session.json",
            dict(
                profile=self.profile,
                source_sha256={
                    name: hashlib.sha256(
                        Path(__file__).with_name(name).read_bytes()
                    ).hexdigest()
                    for name in (
                        "mapping_backend.py",
                        "mapping_node.py",
                        "scan_filter.py",
                        "mapping_storage.py",
                    )
                },
                profile_sha256=hashlib.sha256(
                    self.profile_path.read_bytes()
                ).hexdigest(),
                source_revision=os.environ.get("CLEANY_SOURCE_COMMIT", "unavailable"),
                ros_domain=os.environ.get("ROS_DOMAIN_ID", "0"),
            ),
        )
        try:
            if any(
                name == "cleany_logger"
                for name, _ in self.node.get_node_names_and_namespaces()
            ):
                raise RuntimeError(
                    "An existing Logger owns this ROS namespace; use one recording owner"
                )
            reset = self.rpc(self.node.odom_reset, Trigger.Request())
            if not reset.success:
                raise RuntimeError(reset.message)
            self.logger_process = self.spawn(
                "logger",
                [
                    "ros2",
                    "launch",
                    "cleany_logger",
                    "logger.launch.py",
                    "record_on_start:=false",
                    "config:=" + self.profile["logger_config"],
                ],
            )
            self.logger = LoggerPort(self)
            state = self.logger.call("get_config")
            if state["state"] != "IDLE":
                raise RuntimeError(
                    "Logger already active; refusing to replace its session"
                )
            settings = state["settings"]
            settings["enabled"].update(
                motion=True, slam=True, costmaps=True, lidar=True
            )
            settings["topics"]["lidar"] = "/scan"
            settings["limit_mib"] = min(
                1024, int(self.profile.get("recording_limit_mib", 1024))
            )
            settings["hz"]["lidar"] = 10.0
            for source in (
                "wheel_state_topic",
                "wheel_command_topic",
                "joint_state_topic",
                "canonical_odometry_topic",
                "odometry_topic",
                "raw_scan_topic",
                "map_topic",
                "slam_submaps_topic",
                "slam_trajectory_topic",
                "slam_constraints_topic",
                "slam_matched_points_topic",
            ):
                settings["source_enabled"][source] = True
                settings["source_hz"][source] = None
            self.logger.call("configure", settings=settings, revision=state["revision"])
            state = self.logger.call("start")
            self.logger_session = state["session_id"]
            atomic_json(
                output / "recording.json",
                dict(
                    session_id=self.logger_session,
                    state_directory=self.profile["recordings_directory"],
                ),
            )
            self.process = self.spawn(
                "cartographer",
                [
                    "ros2",
                    "launch",
                    "cleany_navigation",
                    "cartographer_mapping.launch.py",
                    "configuration_basename:=cartographer_hardware_2d.lua",
                    "shutdown_on_exit:=true",
                ],
            )
            self.record_children()
            deadline = time.monotonic() + float(
                self.profile.get("startup_timeout_s", 30)
            )
            while time.monotonic() < deadline:
                if not self.active() or self.logger_process.poll() is not None:
                    raise RuntimeError(self.inactive_reason())
                with self.lock:
                    from rclpy.time import Time

                    valid = (
                        self.grid is not None
                        and self.grid_at >= self.started
                        and time.monotonic() - self.odom_at < 0.5
                        and self.scan is not None
                        and self.scan_at >= self.started
                    )
                    if valid:
                        valid = self.node.tf_buffer.can_transform(
                            "map",
                            self.scan.header.frame_id,
                            Time.from_msg(self.scan.header.stamp),
                        )
                if valid:
                    self.record_children()
                    self.pause(False)
                    self.ready_once = True
                    return
                time.sleep(0.05)
            raise RuntimeError(
                "SLAM readiness timed out: fresh scan-time TF, odom and new map required"
            )
        except Exception as startup_error:
            # Retain both tracebacks if cleanup itself fails.
            try:
                self.stop()
            except Exception as cleanup_error:
                raise RuntimeError(
                    f"Mapping startup failed: {startup_error}; cleanup failed: {cleanup_error}"
                ) from cleanup_error
            raise
