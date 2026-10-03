"""Explicit Gazebo-only check. Requires --simulation acknowledgement and isolated DDS domain.

Uses the configured Nav2/safety stack; never bypasses velocity guards.
"""

import argparse
import json
import time
from pathlib import Path
from uuid import uuid4

import rclpy
from cleany_mission_manager.core.runtime_models import RuntimeRequest
from cleany_mission_manager.node import MissionRuntimeNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

parser = argparse.ArgumentParser()
parser.add_argument("--simulation", required=True, choices=["gazebo"])
parser.add_argument("--targets", required=True)
parser.add_argument("--output", required=True)
parser.add_argument("--linger", type=float, default=10)
args = parser.parse_args()
output = Path(args.output)
output.mkdir(parents=True, exist_ok=False)
rclpy.init()
node = MissionRuntimeNode(
    parameter_overrides=[
        Parameter("targets_file", value=args.targets),
        Parameter("journal_path", value=str(output / "missions.db")),
        Parameter("use_sim_time", value=True),
        Parameter("odom_topic", value="wheel/odom"),
        Parameter("navigation_backend", value="nav2"),
        Parameter("post_mission", value="wait_for_next"),
        Parameter("mock_operation_polls", value=8),
    ]
)
executor = SingleThreadedExecutor()
executor.add_node(node)
start = time.monotonic()
offered = False
finished = None
try:
    while time.monotonic() - start < 90:
        executor.spin_once(timeout_sec=0.02)
        if not offered and node.runtime.readiness()[0]:
            assert node.runtime.offer(
                RuntimeRequest(str(uuid4()), "seat-12", "dev-monitor-gazebo")
            ).accepted
            offered = True
        if node.runtime.last_report and finished is None:
            finished = time.monotonic()
            print(json.dumps(node.runtime.last_report.to_dict()), flush=True)
        if finished and time.monotonic() - finished > args.linger:
            break
    (output / "snapshot.json").write_text(
        json.dumps(node.runtime.debug_snapshot(), indent=2)
    )
    assert offered, "Nav2 readiness: " + str(node.runtime.readiness())
    assert node.runtime.last_report and node.runtime.last_report.outcome == "SUCCESS"
finally:
    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
