"""Own installed device/session launches and finalize mapping before device shutdown."""

from __future__ import annotations
import argparse
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import yaml

from .hardware_profile import load_profile


def main():
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from rclpy.utilities import remove_ros_args
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from cleany_interfaces.msg import MappingStatus
    from cleany_interfaces.srv import MappingControl
    from cleany_navigation.mapping_process import identity, live_recorded
    from cleany_navigation.mapping_storage import atomic_json, atomic_bytes

    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--state-root", default="~/.local/state/cleany/hardware")
    parser.add_argument("--mock", choices=["true", "false"], default="false")
    parser.add_argument("--navigation-map", default="")
    parser.add_argument("--robot-params", default="")
    parser.add_argument("--depth-params", default="")
    parser.add_argument("--phone-motion-profile", choices=["longitudinal", "omni"], default="longitudinal")
    parser.add_argument("--slip-recovery", choices=["true", "false"], default="true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--phone-port", type=int, default=8766)
    args = parser.parse_args(remove_ros_args()[1:])
    args.navigation_map = Path(args.navigation_map) if args.navigation_map else None
    args.robot_params = Path(args.robot_params) if args.robot_params else None
    if not 1 <= args.phone_port <= 65535:
        parser.error("Invalid phone port")
    profile = load_profile(
        args.profile, mock=args.mock == "true", state_root=args.state_root
    )
    if args.navigation_map:
        if not args.robot_params:
            args.robot_params = Path(get_package_share_directory('cleany_navigation'))/'config/phone_lidar_robot.yaml'
        if not args.navigation_map.is_file() or not args.robot_params.is_file():
            parser.error('Navigation requires existing map and verified robot parameters')
        if args.depth_params and not Path(args.depth_params).is_file():
            parser.error('Depth overlay does not exist')
        profile['navigation_enabled'] = True
    state = Path(profile["state_root"])
    state.mkdir(parents=True, exist_ok=True)
    # A domain/state pair has one lifecycle owner, including across terminal restarts.
    import fcntl

    lease = (state / "runtime.lock").open("a")
    try:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("Hardware runtime already owns this state directory")
    domain_locks = Path.home() / ".local/state/cleany/hardware-locks"
    domain_locks.mkdir(parents=True, exist_ok=True)
    domain_lease = (
        domain_locks / ("domain-" + os.environ.get("ROS_DOMAIN_ID", "0") + ".lock")
    ).open("a")
    try:
        fcntl.flock(domain_lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error("A hardware runtime already owns this ROS domain")
    import json

    ledgers = list((state / "maps").glob("*/processes.json"))
    if (state / "runtime.json").exists():
        ledgers.append(state / "runtime.json")
    for ledger in ledgers:
        if live_recorded(json.loads(ledger.read_text()).get("identities", {})):
            parser.error(
                "A previous runtime still owns live children; inspect " + str(ledger)
            )
    profile["base_source"] = profile["base_config"]
    profile["base_config"] = str(state / "base.yaml")
    atomic_bytes(
        state / "base.yaml",
        yaml.safe_dump(profile["base_settings"], sort_keys=False).encode(),
    )
    profile.update(
        host=args.host, phone_port=args.phone_port, token_file=str(state / "web-token")
    )
    if args.navigation_map:
        atomic_json(state/'teleop-motion.json', {'forward_only': args.phone_motion_profile == 'longitudinal'})
    resolved = state / "resolved-profile.yaml"
    atomic_bytes(resolved, yaml.safe_dump(profile, sort_keys=False).encode())
    logger = {
        "cleany_logger": {
            "ros__parameters": {
                "record_on_start": False,
                "state_directory": profile["recordings_directory"],
                "session_limit_bytes": profile.get("recording_limit_mib", 2048) * 1024**2,
                "total_limit_bytes": profile.get("recording_total_limit_mib", 10240) * 1024**2,
            }
        }
    }
    atomic_bytes(Path(profile["logger_config"]), yaml.safe_dump(logger).encode())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("hardware_runtime")
    client = node.create_client(MappingControl, "mapping/control")
    latest = {}
    node.create_subscription(
        MappingStatus,
        "mapping/status",
        lambda msg: latest.update(
            phase=msg.phase,
            message=msg.message,
            request_id=msg.request_id,
            stop_confirmed=msg.stop_confirmed,
        ),
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    closed = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: closed.set())
    signal.signal(signal.SIGTERM, lambda *_: closed.set())
    children = []
    streams = []
    try:
        discovery = time.monotonic() + 1.5
        while time.monotonic() < discovery:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.count_publishers("base/wheel_command") or node.count_publishers(
            "cmd_vel"
        ):
            raise RuntimeError(
                "An existing controller owns this ROS domain; start one hardware runtime"
            )
        commands = (
            (
                "devices",
                [
                    "ros2",
                    "launch",
                    "cleany_bringup",
                    "hardware_devices.launch.py",
                    "resolved_profile:=" + str(resolved),
                ],
            ),
            (
                "mapping",
                [
                    str(
                        Path(get_package_prefix("cleany_navigation"))
                        / "lib/cleany_navigation/mapping_session"
                    ),
                    "--ros-args",
                    "-p",
                    "profile:=" + str(resolved),
                    "-p",
                    "output_root:=" + str(state / "maps"),
                ],
            ),
        )
        if args.navigation_map:
            commands = (commands[0], ('navigation', [
                'ros2', 'launch', 'cleany_navigation', 'phone_navigation.launch.py',
                'map:=' + str(args.navigation_map.resolve()),
                'robot_params_file:=' + str(args.robot_params.resolve()),
                'phone_motion_profile:=' + args.phone_motion_profile,
                'slip_recovery:=' + args.slip_recovery,
                'depth_params_file:=' + args.depth_params,
                'localization_evidence_directory:=' + str(state/'localization'),
            ]), ('supervisor', ['ros2','run','cleany_system_supervisor','system_supervisor','--ros-args','-p',
                'config_file:=' + str(Path(get_package_share_directory('cleany_navigation'))/'config/phone_lidar_supervisor.yaml')]), ('logger', ['ros2','launch','cleany_logger','logger.launch.py', 'record_on_start:=true','config:='+profile['logger_config']]))
        for name, command in commands:
            stream = (state / (name + ".log")).open("ab")
            streams.append(stream)
            children.append(
                (
                    name,
                    subprocess.Popen(
                        command,
                        stdout=stream,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    ),
                )
            )
        atomic_json(
            state / "runtime.json",
            dict(
                supervisor=os.getpid(),
                children={name: p.pid for name, p in children},
                identities={str(p.pid): identity(p.pid) for _, p in children},
                profile=profile,
            ),
        )
        print(
            f"Hardware runtime started; phone port {args.phone_port}; token file {profile['token_file']}; monitor is independent",
            flush=True,
        )
        while not closed.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(p.poll() is not None for _, p in children):
                raise RuntimeError(
                    "Owned hardware/session launch exited; see runtime logs"
                )
    finally:
        if args.navigation_map:
            from std_srvs.srv import SetBool
            pause_client = node.create_client(SetBool, 'teleop/pause')
            if pause_client.wait_for_service(timeout_sec=1.):
                stopped = pause_client.call_async(SetBool.Request(data=True))
                rclpy.spin_until_future_complete(node, stopped, timeout_sec=1.)
        shutdown = {"stop_confirmed": False, "mapping_saved": False}
        if any(name == "mapping" for name, _ in children) and client.wait_for_service(
            timeout_sec=2
        ):
            request_id = "shutdown-" + uuid.uuid4().hex
            future = client.call_async(
                MappingControl.Request(request_id=request_id, operation="stop")
            )
            end = time.monotonic() + 30
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                if future.done() and (
                    not future.result() or not future.result().accepted
                ):
                    break
                if latest.get("request_id") == request_id and latest.get("phase") in (
                    "idle",
                    "failed",
                ):
                    shutdown["mapping_saved"] = latest["phase"] == "idle"
                    shutdown["stop_confirmed"] = latest.get("stop_confirmed", False)
                    shutdown["message"] = latest["message"]
                    break
        # Mapping cleanup still has live device/teleop services here.
        for name, process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=35 if name == "mapping" else 12)
            except subprocess.TimeoutExpired:
                shutdown[name + "_error"] = (
                    "Owned process did not exit; no replacement was started"
                )
        atomic_json(state / "shutdown.json", shutdown)
        for stream in streams:
            stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lease.close()
        domain_lease.close()
