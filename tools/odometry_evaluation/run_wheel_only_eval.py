#!/usr/bin/env python3
"""Config-driven sequential batch wrapper for the one-way wheel odometry evaluator."""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
from typing import Any

from evaluation_stack import EvaluationStack, describe_commands, validate_runtime_config
from motion_control import MotionConfig
from motion_patterns import PATTERN_DIRECTIONS
from stack_readiness import RosReadinessProbe

ROOT = Path(__file__).resolve().parents[2]
EVALUATOR = Path(__file__).with_name("run_oneway_eval.py")
KEYS = {
    "schema_version", "repetitions", "ros_domain_id", "patterns",
    "linear_speeds_mps", "angular_speeds_rad_s", "translation_distance_m",
    "rotation_angle_deg", "odom_topic", "odom_timeout_s", "lidar_min_range_m",
    "extrinsic_config", "motion_config", "runtime_config", "plots", "output_dir",
}


def _number(value: Any, name: str, low: float, high: float | None = None,
            inclusive_low: bool = False) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if (value < low if inclusive_low else value <= low) or (high is not None and value > high):
        left = "[" if inclusive_low else "("
        raise ValueError(f"{name} must be in {left}{low}, {high}]")


def load_config(path: Path) -> tuple[dict[str, Any], Path]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("config must be a JSON object")
    missing, unknown = KEYS - raw.keys(), raw.keys() - KEYS
    if missing or unknown:
        raise ValueError(f"config keys missing={sorted(missing)}, unknown={sorted(unknown)}")
    c = raw
    if type(c["schema_version"]) is not int or c["schema_version"] != 1:
        raise ValueError("schema_version must be integer 1")
    for key in ("repetitions",):
        if type(c[key]) is not int or c[key] <= 0:
            raise ValueError(f"{key} must be a positive integer")
    if type(c["ros_domain_id"]) is not int or not 0 <= c["ros_domain_id"] <= 232:
        raise ValueError("ros_domain_id must be an integer in [0, 232]")
    pats = c["patterns"]
    allowed_patterns = set(PATTERN_DIRECTIONS) | {"rotation"}
    if not isinstance(pats, list) or not pats or any(not isinstance(x, str) or x not in allowed_patterns for x in pats) or len(set(pats)) != len(pats):
        raise ValueError(f"patterns must be a unique nonempty list from {sorted(allowed_patterns)}")
    for key, upper in (("linear_speeds_mps", .5), ("angular_speeds_rad_s", 1.)):
        values = c[key]
        if not isinstance(values, list) or not values:
            raise ValueError(f"{key} must be a nonempty list")
        for v in values:
            _number(v, key, 0, upper)
    _number(c["translation_distance_m"], "translation_distance_m", 0, 3)
    _number(c["rotation_angle_deg"], "rotation_angle_deg", 0)
    _number(c["odom_timeout_s"], "odom_timeout_s", 0)
    _number(c["lidar_min_range_m"], "lidar_min_range_m", 0, 10, True)
    if c["lidar_min_range_m"] >= 10:
        raise ValueError("lidar_min_range_m must be in [0, 10)")
    if not isinstance(c["odom_topic"], str) or not c["odom_topic"] or any(x.isspace() for x in c["odom_topic"]):
        raise ValueError("odom_topic must be a nonempty whitespace-free string")
    for key in ("extrinsic_config", "motion_config", "runtime_config", "output_dir"):
        if not isinstance(c[key], str) or not c[key].strip():
            raise ValueError(f"{key} must be a nonempty path string")
    if type(c["plots"]) is not bool:
        raise ValueError("plots must be a boolean")
    resolved = {key: (ROOT / c[key]).resolve() for key in ("extrinsic_config", "motion_config")}
    output = (ROOT / c["output_dir"]).resolve()
    # Pure config loaders; no ROS imports or initialization.
    sys.path.insert(0, str(EVALUATOR.parent))
    from planar_extrinsic import PlanarExtrinsic
    MotionConfig.from_json(resolved["motion_config"])
    PlanarExtrinsic.from_json(resolved["extrinsic_config"])
    validate_runtime_config(json.loads((ROOT / c["runtime_config"]).read_text()), ROOT)
    return c, output


def command(c: dict[str, Any], out: Path, motion: Path, extrinsic: Path) -> list[str]:
    args = [sys.executable, str(EVALUATOR), "--patterns", *c["patterns"],
            "--linear-speeds", *(str(x) for x in c["linear_speeds_mps"]),
            "--angular-speeds", *(str(x) for x in c["angular_speeds_rad_s"]),
            "--distance", str(c["translation_distance_m"]),
            "--rotation-angle-deg", str(c["rotation_angle_deg"]),
            "--odom-timeout", str(c["odom_timeout_s"]),
            "--motion-config", str(motion), "--extrinsic-config", str(extrinsic),
            "--odom-topic", c["odom_topic"], "--lidar-min-range", str(c["lidar_min_range_m"]),
            "--output-dir", str(out)]
    if not c["plots"]:
        args.append("--no-plots")
    return args


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=ROOT / "configs/mission/wheel_odometry_evaluation.json")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        config_path = args.config if args.config.is_absolute() else ROOT / args.config
        config_path = config_path.resolve()
        c, output = load_config(config_path)
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        parser.error(str(exc))
    motion = (ROOT / c["motion_config"]).resolve()
    extrinsic = (ROOT / c["extrinsic_config"]).resolve()
    runtime_path = (ROOT / c["runtime_config"]).resolve()
    runtime_bytes = runtime_path.read_bytes()
    runtime = validate_runtime_config(json.loads(runtime_bytes), ROOT)
    batch = output / f"batch_{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}"
    commands = [command(c, batch / f"run_{i}", motion, extrinsic) for i in range(1, c["repetitions"] + 1)]
    round_trips = c["repetitions"] * sum(
        len(c["angular_speeds_rad_s"] if p == "rotation" else c["linear_speeds_mps"])
        for p in c["patterns"])
    print(f"Batch: {batch}\nROS domain: {c['ros_domain_id']}\n"
          f"Runs: {len(commands)}; pattern/speed round trips: {round_trips}; "
          f"directional legs: {2 * round_trips}")
    settings = MotionConfig.from_json(motion)
    print("Evaluation-only distance/yaw P: "
          f"X={settings.linear_x_kp or settings.linear_kp:g}; "
          f"Y={settings.linear_y_kp or settings.linear_kp:g}; "
          f"yaw={settings.yaw_kp:g}; diagonal={settings.linear_kp:g}")
    print("Runtime: reuse healthy nodes; start missing components; require live telemetry.")
    for component, cmd in describe_commands(runtime, ROOT).items():
        print(f"{component}: {shlex.join(cmd)}")
    for i, cmd in enumerate(commands, 1):
        print(f"run_{i}: {shlex.join(cmd)}")
    if args.dry_run:
        return 0
    # Separate from the per-motion evaluator lock; held across all repetitions.
    with open("/tmp/cleany_wheel_eval_batch.lock", "a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Another wheel evaluation batch is active.", file=sys.stderr)
            return 1
        return run_batch(c, config_path, motion, extrinsic, runtime_path, runtime_bytes,
                         runtime, batch, commands)


def run_batch(c: dict, config_path: Path, motion: Path, extrinsic: Path,
              runtime_path: Path, runtime_bytes: bytes, runtime: dict,
              batch: Path, commands: list[list[str]]) -> int:
    batch.mkdir(parents=True, exist_ok=False)
    motion_bytes = motion.read_bytes()
    extrinsic_bytes = extrinsic.read_bytes()
    snapshot = {"config": c, "config_path": str(config_path),
                "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
                "motion_config_path": str(motion),
                "motion_config_sha256": hashlib.sha256(motion_bytes).hexdigest(),
                "extrinsic_config_path": str(extrinsic),
                "extrinsic_config_sha256": hashlib.sha256(extrinsic_bytes).hexdigest(),
                "runtime_config_path": str(runtime_path),
                "runtime_config_sha256": hashlib.sha256(runtime_bytes).hexdigest(),
                "runtime_config": json.loads(runtime_bytes),
                "ros_domain_id": c["ros_domain_id"]}
    motion_frozen = batch / "motion_config.json"
    extrinsic_frozen = batch / "extrinsic_config.json"
    motion_frozen.write_bytes(motion_bytes)
    extrinsic_frozen.write_bytes(extrinsic_bytes)
    (batch / "runtime_config.json").write_bytes(runtime_bytes)
    base_path = Path(runtime["base"]["config"])
    base_bytes = base_path.read_bytes()
    base_frozen = batch / "base_hardware.yaml"
    base_frozen.write_bytes(base_bytes)
    snapshot.update(base_config_path=str(base_path),
                    base_config_sha256=hashlib.sha256(base_bytes).hexdigest())
    runtime = {**runtime, "base": {**runtime["base"], "config": str(base_frozen)}}
    (batch / "config_snapshot.json").write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    results: list[dict[str, Any]] = []
    interrupted: int | None = None
    active: subprocess.Popen[Any] | None = None
    forwarded = False
    old_handlers: dict[int, Any] = {}
    stack = None
    probe = None
    runtime_result = None
    failed_stage = "runtime_startup"
    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = str(c["ros_domain_id"])

    def on_signal(sig: int, _frame: Any) -> None:
        nonlocal interrupted, forwarded
        if interrupted is not None:
            return
        interrupted = sig
        if active is not None:
            try:
                if active.poll() is None:
                    active.send_signal(sig)
                    forwarded = True
            except (OSError, ProcessLookupError):
                pass

    for sig in (signal.SIGINT, signal.SIGTERM):
        old_handlers[sig] = signal.signal(sig, on_signal)
    status = 1
    failure_reason: str | None = None
    try:
        print("Preparing Agent, base driver and LiDAR; waiting for live data...", flush=True)
        probe = RosReadinessProbe(c["odom_topic"], c["odom_timeout_s"],
                                  runtime["discovery_s"], c["lidar_min_range_m"])
        stack = EvaluationStack(runtime, ROOT, batch / "runtime_logs", probe,
                                cancelled=lambda: interrupted is not None)
        runtime_result = stack.start()
        if interrupted is not None:
            raise RuntimeError("Interrupted during runtime startup")
        print("Runtime ready: odometry, scans, disabled MCU and /base/enable verified.", flush=True)
        failed_stage = "evaluation"
        for i, cmd in enumerate(commands, 1):
            if interrupted is not None:
                status = 128 + interrupted
                break
            env = os.environ.copy()
            env.update(ROS_DOMAIN_ID=str(c["ros_domain_id"]), PYTHONNOUSERSITE="1")
            # All children use immutable copies of referenced configuration.
            frozen_cmd = list(cmd)
            frozen_cmd[frozen_cmd.index(str(motion))] = str(motion_frozen)
            frozen_cmd[frozen_cmd.index(str(extrinsic))] = str(extrinsic_frozen)
            try:
                active = subprocess.Popen(frozen_cmd, env=env, start_new_session=True)
                # A signal can arrive during Popen before `active` is assigned.
                if interrupted is not None and not forwarded:
                    try:
                        if active.poll() is None:
                            active.send_signal(interrupted)
                            forwarded = True
                    except (OSError, ProcessLookupError):
                        pass
                rc = active.wait()
            except KeyboardInterrupt:
                on_signal(signal.SIGINT, None)
                rc = active.wait()
            except OSError as exc:
                status = 1
                failure_reason = f"Could not start run_{i}: {exc}"
                results.append({"run": i, "return_code": None, "output_dir": str(batch / f"run_{i}"),
                                "failure_reason": failure_reason})
                break
            results.append({"run": i, "return_code": rc, "output_dir": str(batch / f"run_{i}")})
            active = None
            if interrupted is not None:
                status = 128 + interrupted
                break
            if rc:
                status = 128 + (-rc) if rc < 0 else rc
                failure_reason = f"run_{i} exited with status {rc}"
                break
        else:
            status = 0
    except (KeyboardInterrupt, Exception) as error:
        if isinstance(error, KeyboardInterrupt):
            on_signal(signal.SIGINT, None)
        status = 128 + interrupted if interrupted is not None else 1
        failure_reason = str(error) or "Interrupted"
        print(f"{failed_stage} failed: {failure_reason}", file=sys.stderr, flush=True)
    finally:
        # Preserve runtime until the evaluator has completed its STOP/disable path.
        if active is not None:
            try:
                if active.poll() is None:
                    if not forwarded:
                        active.send_signal(signal.SIGINT)
                    active.wait()
            except OSError as error:
                status = 1
                failure_reason = f"Evaluator cleanup failed: {error}"
        for resource in (stack, probe):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    status = 1
                    failure_reason = f"{failure_reason or ''}; runtime cleanup: {error}"
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain
        summary = {"status": "success" if status == 0 else "interrupted" if interrupted else "failed",
                   "return_code": status, "failure_reason": failure_reason,
                   "failed_stage": failed_stage if status else None,
                   "runtime": runtime_result,
                   "runs": results, **snapshot}
        try:
            (batch / "batch_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
            print(f"Batch summary: {batch / 'batch_summary.json'}", flush=True)
        finally:
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
