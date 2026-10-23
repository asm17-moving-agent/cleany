#!/usr/bin/env python3
"""Config-driven physical axial/diagonal/yaw P-response trials."""
from __future__ import annotations

import argparse
from dataclasses import replace
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import time

from evaluation_stack import EvaluationStack, validate_runtime_config
from icp_convergence import BoundedCycleDetector
from motion_control import MotionConfig
from motion_response import AXES, endpoint_errors, plot_responses, project_translation, response_metrics
from planar_extrinsic import PlanarExtrinsic
from run_wheel_only_eval import ROOT, load_config as load_evaluation_config


def load_config(path: Path) -> dict:
    config = json.loads(path.read_text())
    required = {"evaluation_config", "output_dir", "repetitions", "translation_distance_m",
                "rotation_angle_deg", "trials", "guards"}
    if (not isinstance(config, dict) or not required <= config.keys()
            or config.keys() - required - {
                "require_valid_icp", "icp_min_inlier_ratio", "icp_convergence",
            }):
        raise ValueError(f"response config requires exactly {sorted(required)}")
    if "require_valid_icp" in config and type(config["require_valid_icp"]) is not bool:
        raise ValueError("require_valid_icp must be boolean")
    for key in ("evaluation_config", "output_dir"):
        if not isinstance(config[key], str) or not config[key].strip():
            raise ValueError(f"{key} must be a path")
    if type(config["repetitions"]) is not int or not 1 <= config["repetitions"] <= 10:
        raise ValueError("repetitions must be in [1,10]")
    def number(value, maximum):
        return (not isinstance(value, bool) and isinstance(value, (int, float))
                and math.isfinite(value) and 0 < value <= maximum)
    if not number(config.get("icp_min_inlier_ratio", .40), 1):
        raise ValueError("icp_min_inlier_ratio must be in (0, 1]")
    convergence = config.get("icp_convergence", {})
    if not isinstance(convergence, dict):
        raise ValueError("icp_convergence must be an object")
    try:
        BoundedCycleDetector(**convergence)
    except (TypeError, ValueError) as error:
        raise ValueError(f"icp_convergence: {error}") from error
    if not number(config["translation_distance_m"], .5):
        raise ValueError("this supervised response session permits at most 0.5 m per leg")
    if not number(config["rotation_angle_deg"], 90):
        raise ValueError("this supervised response session permits at most 90 degrees per leg")
    if not isinstance(config["trials"], list) or not config["trials"]:
        raise ValueError("trials must be a nonempty list")
    for trial in config["trials"]:
        if not isinstance(trial, dict) or trial.keys() != {"axis", "kp", "speed"}:
            raise ValueError("each trial requires axis, kp and speed")
        if trial["axis"] not in AXES or not number(trial["kp"], 8):
            raise ValueError("invalid axis or candidate kp (0,8]")
        if not number(trial["speed"], 1 if trial["axis"] == "yaw" else .5):
            raise ValueError("candidate speed exceeds existing hardware limits")
    guards = config["guards"]
    guard_keys = {"session_radius_m", "cross_track_m", "translation_overshoot_m",
                  "translation_yaw_drift_deg", "rotation_overshoot_deg"}
    if not isinstance(guards, dict) or guards.keys() != guard_keys:
        raise ValueError(f"guards require exactly {sorted(guard_keys)}")
    if any(not number(v, 180) for v in guards.values()):
        raise ValueError("guards must be finite and positive")
    return config


def candidate_config(base: MotionConfig, axis: str, kp: float) -> MotionConfig:
    field = dict(x="linear_x_kp", y="linear_y_kp", yaw="yaw_kp",
                 diagonal_left="linear_kp", diagonal_right="linear_kp")[axis]
    return replace(base, **{field: kp})


def trial_validity(result: dict | None, require_valid_icp: bool) -> tuple[bool, bool, bool]:
    control_valid = bool(result) and all(
        result["control_stopping_performance"][leg]["is_valid"]
        for leg in ("outbound", "inbound"))
    icp_valid = bool(result and result["is_valid"])
    return control_valid, icp_valid, control_valid and (icp_valid or not require_valid_icp)


def close_resources(node, stack, probe, shutdown) -> list[str]:
    """Attempt every cleanup even if STOP raises or only latches an abort."""
    errors = []
    operations = [node.stop_robot, node.destroy_node] if node is not None else []
    if shutdown is not None:
        operations.append(shutdown)
    operations += [resource.close for resource in (stack, probe) if resource is not None]
    for operation in operations:
        try:
            operation()
        except Exception as error:
            errors.append(f"cleanup: {error}")
    if node is not None and node.aborted:
        errors.append(node.last_failure or "Base disable was not acknowledged")
    return errors


def bound_violation(pose, start, axis: str, target: float, guards: dict, session_start) -> str:
    dx, dy = pose.x_m-start.x_m, pose.y_m-start.y_m
    x = math.cos(start.yaw_rad)*dx + math.sin(start.yaw_rad)*dy
    y = -math.sin(start.yaw_rad)*dx + math.cos(start.yaw_rad)*dy
    yaw = abs((pose.yaw_rad-start.yaw_rad+math.pi) % (2*math.pi)-math.pi)
    if math.hypot(pose.x_m-session_start.x_m, pose.y_m-session_start.y_m) > guards["session_radius_m"]:
        return "session translation boundary exceeded"
    if axis == "yaw":
        if yaw > target + math.radians(guards["rotation_overshoot_deg"]):
            return "rotation excursion exceeded"
        if math.hypot(x, y) > guards["cross_track_m"]:
            return "rotation centre drift exceeded"
    else:
        along, cross = project_translation(x, y, axis)
        if abs(along) > target + guards["translation_overshoot_m"]:
            return "translation excursion exceeded"
        if abs(cross) > guards["cross_track_m"]:
            return "cross-track excursion exceeded"
        if yaw > math.radians(guards["translation_yaw_drift_deg"]):
            return "translation yaw drift exceeded"
    return ""


def run(config: dict, evaluation: dict, out: Path) -> int:
    # Import ROS only for a real run.
    import rclpy
    from evaluation_results import write_data_json
    from motion_runtime import await_fresh_pose, request_abort
    from run_oneway_eval import OneWayEvaluatorNode, run_single_oneway_test
    from stack_readiness import RosReadinessProbe

    class CheckedEvaluator(OneWayEvaluatorNode):
        watch = None
        session_start = None
        response_axis = None

        def _on_odom(self, msg):
            super()._on_odom(msg)
            if self.watch is not None and self._motion_active and not self.aborted:
                reason = bound_violation(self.current_pose, *self.watch,
                                         config["guards"], self.session_start)
                if reason:
                    request_abort(self, reason)

        def execute_leg(self, vx, vy, wz, target_displacement, is_rotation, **kwargs):
            start = await_fresh_pose(self)
            if self.session_start is None:
                self.session_start = start
            axis = self.response_axis
            reason = bound_violation(start, start, axis, target_displacement,
                                     config["guards"], self.session_start)
            if reason:
                request_abort(self, reason)
                return False, {"is_valid": False, "failure_reason": reason}
            self.watch = (start, axis, target_displacement)
            try:
                return super().execute_leg(vx, vy, wz, target_displacement, is_rotation, **kwargs)
            finally:
                self.watch = None

    os.environ["ROS_DOMAIN_ID"] = str(evaluation["ros_domain_id"])
    runtime = validate_runtime_config(
        json.loads((ROOT / evaluation["runtime_config"]).read_text()), ROOT)
    base = MotionConfig.from_json(ROOT / evaluation["motion_config"])
    extrinsic = PlanarExtrinsic.from_json(ROOT / evaluation["extrinsic_config"])
    out.mkdir(parents=True, exist_ok=False)
    write_data_json(out / "config.json", config)
    write_data_json(out / "evaluation_config.json", evaluation)
    write_data_json(out / "base_motion_config.json", base.to_dict())
    records = []
    stack, probe, node = None, None, None
    cancelled = cleaning_up = ros_initialized = False
    status, failure = 1, ""

    def interrupt(sig, frame):
        nonlocal cancelled
        if cancelled:
            return
        cancelled = True
        if cleaning_up:
            return
        if node is not None:
            request_abort(node, f"Interrupted by signal {sig}")
            raise KeyboardInterrupt

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        probe = RosReadinessProbe(evaluation["odom_topic"], evaluation["odom_timeout_s"],
                                  runtime["discovery_s"], evaluation["lidar_min_range_m"])
        stack = EvaluationStack(runtime, ROOT, out / "runtime_logs", probe, lambda: cancelled)
        startup = stack.start()
        write_data_json(out / "runtime_ready.json", startup)
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        ros_initialized = True
        node = CheckedEvaluator(
            min_range_m=evaluation["lidar_min_range_m"],
            odom_timeout_s=evaluation["odom_timeout_s"], motion_config=base,
            extrinsic=extrinsic, odom_topic=evaluation["odom_topic"],
            min_inlier_ratio=config.get("icp_min_inlier_ratio", .40),
            convergence_config=config.get("icp_convergence"))
        node.plot_enabled = False
        node.evaluation_settings = {"odom_topic": evaluation["odom_topic"],
                                    "lidar_min_range_m": evaluation["lidar_min_range_m"],
                                    "icp_min_inlier_ratio": node.matcher.min_inlier_ratio,
                                    "icp_convergence": node.matcher.convergence_settings.copy(),
                                    "extrinsic_calibration_applied": extrinsic.to_dict()}
        # Discover the service and confirm STOP while still passive, before a
        # command is armed. Initial DDS service discovery may exceed 150 ms.
        if not node.set_base_enabled(False):
            raise RuntimeError("Initial base disable handshake was not acknowledged")
        await_fresh_pose(node, timeout_s=5.0)
        for rep in range(1, config["repetitions"]+1):
            for index, trial in enumerate(config["trials"], 1):
                if cancelled:
                    raise RuntimeError("Response session interrupted")
                node.motion_config = candidate_config(base, trial["axis"], trial["kp"])
                node.response_axis = trial["axis"]
                prefix = out / f"rep{rep}_{index:02d}_{trial['axis']}_kp{trial['kp']:g}_v{trial['speed']:g}"
                print(f"PHYSICAL RESPONSE: rep={rep}, {trial}, +/- "
                      f"{config['rotation_angle_deg'] if trial['axis']=='yaw' else config['translation_distance_m']}",
                      flush=True)
                target = (math.radians(config["rotation_angle_deg"]) if trial["axis"] == "yaw"
                          else config["translation_distance_m"])
                result = run_single_oneway_test(
                    node, AXES[trial["axis"]], trial["speed"], target, prefix)
                record = {**trial, "repetition": rep, "result": result}
                control_valid, icp_valid, accepted = trial_validity(
                    result, config.get("require_valid_icp", True))
                record.update(control_valid=control_valid, icp_valid=icp_valid)
                if result is not None:
                    record["metrics"] = {
                        leg: response_metrics(item, trial["axis"])
                        for leg, item in result["control_stopping_performance"].items()}
                    record["endpoint_errors"] = endpoint_errors(result, trial["axis"])
                records.append(record)
                write_data_json(prefix.parent / f"{prefix.name}.json", record)
                print(json.dumps({"trial": trial, "metrics": record.get("metrics")}), flush=True)
                if not accepted or node.aborted:
                    raise RuntimeError(node.last_failure or "Motion/ICP rejected; no further trials")
        status = 0
    except (KeyboardInterrupt, Exception) as error:
        failure = str(error) or "Interrupted"
        print(f"RESPONSE STOPPED: {failure}", flush=True)
    finally:
        cleaning_up = True
        cleanup_errors = close_resources(
            node, stack, probe, rclpy.shutdown if ros_initialized else None)
        if cleanup_errors or cancelled:
            status = 1
            failure = "; ".join(filter(None, [failure, *cleanup_errors,
                                             "Interrupted" if cancelled else ""]))
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        write_data_json(out / "response_summary.json", {
            "status": "success" if status == 0 else "failed", "failure": failure,
            "measurement": "physical wheel-odometry feedback; external ICP endpoint comparison",
            "rejected_icp_pairs": sum(not r["icp_valid"] for r in records),
            "config": config, "records": records,
            "motion_runs": node.motion_results if node is not None else []})
        if any(record.get("result") for record in records):
            plot_responses(records, out / "response.png")
        print(f"Response data: {out}", flush=True)
    return status


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=ROOT / "configs/mission/all_axis_validation.json")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = load_config(ROOT / args.config)
        evaluation, _ = load_evaluation_config(ROOT / config["evaluation_config"])
    except (ValueError, TypeError, OSError) as error:
        parser.error(str(error))
    out = ROOT / config["output_dir"] / time.strftime("%Y%m%d_%H%M%S")
    if args.dry_run:
        print(json.dumps({"config": config, "output": str(out),
                          "directional_legs": 2*len(config["trials"])*config["repetitions"]}, indent=2))
        return 0
    with open("/tmp/cleany_wheel_eval_batch.lock", "a") as batch_lock, \
            open("/tmp/cleany_odom_eval.lock", "a") as motion_lock:
        try:
            for lock in (batch_lock, motion_lock):
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("Another motion evaluation is active.")
            return 1
        return run(config, evaluation, out)


if __name__ == "__main__":
    raise SystemExit(main())
