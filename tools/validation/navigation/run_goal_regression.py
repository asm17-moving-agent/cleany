#!/usr/bin/env python3
"""Headless saved-map goal regressions, with full navigation safety enabled."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from uuid import uuid4

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import yaml

from run_safety_evaluation import prepare, stop
from safety_probe import SafetyProbe

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / "ros2_ws/src/cleany_gazebo_sim/test/fixtures/navigation_goals"


def prepare_goal(
    output: Path, case: str, profile: str, rtf: float | None = None
) -> dict:
    """Generate current CAD/world and pair it with the versioned survey map."""
    specification = yaml.safe_load((FIXTURE / "goals.yaml").read_text())
    goal = specification["cases"][case]
    provenance = json.loads((FIXTURE / "provenance.json").read_text())
    share = Path(get_package_share_directory("cleany_gazebo_sim"))
    layout = share / "config/study_cafe/study_cafe_layout.yaml"
    for path, expected in [
        (layout, provenance["layout_sha256"]),
        (FIXTURE / "map.pgm", provenance["map_sha256"]),
    ]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(
                f"Survey fixture no longer matches {path}; review map/world together"
            )
    factor = specification["real_time_factor"] if rtf is None else rtf
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError("real-time factor must be finite and positive")
    scenario = prepare(
        output,
        FIXTURE / "map.yaml",
        robot_model="cad_frame",
        obstacle_memory=True,
        guard_3d=True,
        gazebo_profile=profile,
        real_time_factor=factor,
    )
    spawn = provenance["spawn_pose"]
    if scenario["spawn_pose"] != spawn:
        raise ValueError("Survey spawn differs from generated world")
    dx, dy = (goal["goal_world"][i] - spawn[i] for i in range(2))
    c, s = math.cos(spawn[5]), math.sin(spawn[5])
    target = [c * dx + s * dy, -s * dx + c * dy, goal["goal_map_yaw"]]
    scenario.update(
        map_source=provenance["method"],
        regression_case=case,
        goal_world=goal["goal_world"],
        requested_rtf=factor,
        regression_limits={**specification, **goal},
    )
    scenario["evaluation"]["avoid"].update(
        goal=target,
        timeout_wall_s=goal["timeout_wall_s"],
        max_recoveries=specification["max_recoveries"],
    )
    # Exercise the installed default holonomic configuration, without hidden test tuning.
    launch = output / "run.launch.py"
    launch.write_text(
        launch.read_text().replace(
            "'axis_params_file':", "'axis_motion':'false', 'axis_params_file':"
        )
    )
    (output / "scenario.json").write_text(json.dumps(scenario, indent=2))
    return scenario


def run_goal(
    output: Path,
    case: str,
    profile: str = "fortress",
    rtf: float | None = None,
    domain: int = 210,
) -> dict:
    """Run one isolated simulation; always retain evidence and stop owned processes."""
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
    partition = "cleany_goal_regression_" + uuid4().hex
    os.environ.update(IGN_PARTITION=partition, GZ_PARTITION=partition)
    result: dict = {"case": case, "completed": False}
    process = node = None
    samples: list[dict] = []
    try:
        scenario = prepare_goal(output, case, profile, rtf)
        with (output / "launch.log").open("w") as log:
            process = subprocess.Popen(
                ["ros2", "launch", str(output / "run.launch.py")],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        (output / "session.json").write_text(
            json.dumps(
                dict(
                    supervisor=os.getpid(),
                    launch=process.pid,
                    domain=domain,
                    partition=partition,
                )
            )
        )
        rclpy.init()
        node = SafetyProbe(output, scenario)
        maxima = {"lateral_command_mps": 0.0, "camera_pan_rad": 0.0}

        def command(message: Twist) -> None:
            if node.phase == "nav2_avoid":
                maxima["lateral_command_mps"] = max(
                    maxima["lateral_command_mps"], abs(message.linear.y)
                )

        def joint(message: JointState) -> None:
            if node.phase == "nav2_avoid" and "head_pan_joint" in message.name:
                angle = message.position[message.name.index("head_pan_joint")]
                maxima["camera_pan_rad"] = max(maxima["camera_pan_rad"], abs(angle))

        node.create_subscription(Twist, "/cmd_vel", command, qos_profile_sensor_data)
        node.create_subscription(
            JointState, "/joint_states", joint, qos_profile_sensor_data
        )
        original_tick = node.tick
        started = time.monotonic()

        def tick() -> None:
            before = len(node.rows)
            original_tick()
            if len(node.rows) != before:
                truth = node.truth()
                samples.append(
                    dict(
                        sim_s=node.now(),
                        wall_s=time.monotonic() - started,
                        phase=node.phase,
                        x=truth[0],
                        y=truth[1],
                        yaw=truth[2],
                    )
                )

        node.tick = tick
        node.initialize()
        node.wait(3.0)
        wall_start, sim_start = time.monotonic(), node.now()
        result.update(
            node.run_avoid(
                inject_obstacle=False, keep_running=lambda: process.poll() is None
            )
        )
        result.update(
            goal_wall_s=time.monotonic() - wall_start,
            goal_sim_s=node.now() - sim_start,
            requested_rtf=scenario["requested_rtf"],
            goal_map=scenario["evaluation"]["avoid"]["goal"],
            **maxima,
        )
        result["achieved_rtf"] = result["goal_sim_s"] / result["goal_wall_s"]
        limits = scenario["regression_limits"]
        truth = result["final_truth"]
        error = math.dist(truth[:2], result["goal_map"][:2])
        yaw_error = abs(
            math.atan2(
                math.sin(truth[2] - result["goal_map"][2]),
                math.cos(truth[2] - result["goal_map"][2]),
            )
        )
        checks = result["checks"]
        result.update(truth_goal_error_m=error, truth_yaw_error_rad=yaw_error)
        # Check simulator ground-truth arrival independently of the Nav2 result.
        checks.update(
            real_progress=error < limits["max_truth_error_m"],
            final_heading=yaw_error < limits["max_truth_yaw_error_rad"],
        )
        if case == "chair_gap":
            checks.update(
                lateral_displacement=abs(truth[1]) > limits["min_truth_lateral_m"],
                lateral_command=maxima["lateral_command_mps"]
                > limits["min_lateral_command_mps"],
                camera_rotated=maxima["camera_pan_rad"] > limits["min_camera_pan_rad"],
            )
        node.wait(1.0)
        velocity = node.latest.get("/cmd_vel")
        checks["stopped_after_goal"] = (
            velocity is not None
            and max(
                abs(velocity.linear.x), abs(velocity.linear.y), abs(velocity.angular.z)
            )
            < 0.001
        )
        result["completed"] = True
        result["passed"] = all(checks.values())
        return result
    except BaseException as error:
        result.update(passed=False, error=f"{type(error).__name__}: {error}")
        raise
    finally:
        try:
            if node is not None:
                try:
                    node.finish()
                    node.save()
                finally:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            stop(process)
            if samples:
                with (output / "trajectory.csv").open("w") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
                    writer.writeheader()
                    writer.writerows(samples)
            log_path = output / "launch.log"
            result["planner_shutdown_allocator_error"] = log_path.exists() and (
                "double free or corruption" in log_path.read_text(errors="replace")
            )
            (output / "result.json").write_text(json.dumps(result, indent=2))
            print(
                json.dumps({k: v for k, v in result.items() if k != "feedback"}),
                flush=True,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=("distant", "chair_gap"), required=True)
    parser.add_argument(
        "--output", type=Path, required=True, help="New evidence directory"
    )
    parser.add_argument(
        "--profile", choices=("fortress", "harmonic"), default="fortress"
    )
    parser.add_argument("--rtf", type=float, default=None)
    parser.add_argument("--domain-id", type=int, default=210)
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 232:
        parser.error("domain-id must be between 0 and 232")
    result = run_goal(args.output, args.case, args.profile, args.rtf, args.domain_id)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
