"""Actual ROS runtime smoke. Explicit MOCK navigation; no HTTP robot commands.

Run in an isolated ROS_DOMAIN_ID with monitor on the same domain.
Optional peer package must already be in the overlay for --held.
"""

import argparse
import json
import time
from pathlib import Path
from urllib.request import urlopen
from uuid import uuid4

import rclpy
from cleany_mission_manager.core.runtime_models import RuntimeRequest
from cleany_mission_manager.node import MissionRuntimeNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

parser = argparse.ArgumentParser()
parser.add_argument("--held", action="store_true")
parser.add_argument("--linger", type=float, default=0)
parser.add_argument("--url", default="http://127.0.0.1:8768")
parser.add_argument("--output", required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
output = Path(args.output).expanduser()
output.mkdir(parents=True, exist_ok=False)
rclpy.init()
executor = SingleThreadedExecutor()
server = None
if args.held:
    from cleany_skill_executor.manipulation_node import ManipulationNode

    server = ManipulationNode(
        namespace="/mock",
        parameter_overrides=[
            Parameter("database_path", value=str(output / "executions.db")),
            Parameter(
                "mock_config",
                value=str(
                    root
                    / "ros2_ws/src/cleany_mission_manager/config/manipulation_server_fixture.yaml"
                ),
            ),
        ],
    )
    executor.add_node(server)
node = MissionRuntimeNode(
    parameter_overrides=[
        Parameter(
            "targets_file",
            value=str(
                root
                / "ros2_ws/src/cleany_mission_manager/config/study_cafe_targets.yaml"
            ),
        ),
        Parameter("journal_path", value=str(output / "missions.db")),
        Parameter("navigation_backend", value="mock"),
        Parameter("manipulation_backend", value="action_mock" if server else "mock"),
        Parameter("manipulation_initial_mock_safe", value=True),
        Parameter("mock_operation_polls", value=8),
        Parameter("operation_timeout", value=120.0),
        Parameter("cancel_timeout", value=15.0),
    ]
)
executor.add_node(node)
started = time.monotonic()
offered = canceled = False
mid = str(uuid4())
finished_at = None
try:
    while time.monotonic() - started < args.linger + 60:
        executor.spin_once(timeout_sec=0.02)
        if (
            not offered
            and time.monotonic() - started > 3
            and node.runtime.readiness()[0]
        ):
            assert node.runtime.offer(
                RuntimeRequest(mid, "seat-12", "dev-monitor-smoke")
            ).accepted
            offered = True
        if (
            server
            and server.core.record
            and server.core.record.stage.value == "TRANSPORTING"
            and not canceled
        ):
            assert node.runtime.cancel(mid).accepted
            canceled = True
        if node.runtime.last_report and finished_at is None:
            finished_at = time.monotonic()
            expected = "CANCELLED" if server else "SUCCESS"
            assert node.runtime.last_report.outcome == expected
            if server:
                assert (
                    node.runtime.state.value == "ERROR"
                    and node.navigator.commands == ["seat-12"]
                )
            print(
                json.dumps(
                    {
                        "outcome": expected,
                        "state": node.runtime.state.value,
                        "mission_id": mid,
                    }
                ),
                flush=True,
            )
        if finished_at and time.monotonic() - finished_at > max(3, args.linger):
            break
    assert finished_at is not None, "Mission timed out"
    with urlopen(args.url + "/api/snapshot", timeout=5) as response:
        observed = json.load(response)
    runtime = next(r["data"] for r in observed["rows"] if r["kind"] == "runtime")
    assert runtime["last_result"]["request"]["mission_id"] == mid
    assert runtime["last_result"]["outcome"] == ("CANCELLED" if server else "SUCCESS")
    if server:
        assert runtime["state"] == "ERROR" and not runtime["safe_to_drive"]
        assert runtime["manipulation"]["object_state"] == "HELD"
        assert runtime["manipulation"]["stop_confirmed"]
    assert any(
        r["kind"] == "event" and r["data"].get("kind") == "bt_tick"
        for r in observed["rows"]
    )
    (output / "observed.json").write_text(
        json.dumps(observed, ensure_ascii=False, indent=2)
    )
    print("HTTP observation matches actual ROS runtime", flush=True)
finally:
    node.destroy_node()
    if server:
        server.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
