"""Verify the existing trajectory controller using actual Isaac joint feedback."""
from __future__ import annotations

import json
from pathlib import Path
import time

import rclpy
from cadence import confirmation_window
from ros_scope import scoped_ros_args
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def main() -> None:
    rclpy.init(args=scoped_ros_args())
    node = rclpy.create_node("isaac_arm_validation")
    latest = {}
    samples = []
    names = ["right_"+part+"_joint" for part in
             ("shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll")]

    def receive(message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        latest.update(zip(message.name, message.position))
        samples.append({"stamp": stamp, "wall": time.monotonic(),
                        "positions": dict(zip(message.name, message.position)),
                        "velocities": dict(zip(message.name, message.velocity))})

    node.create_subscription(JointState, "/joint_states", receive, 10)
    client = ActionClient(node, FollowJointTrajectory, "right_arm_controller/follow_joint_trajectory")

    def wait(predicate, timeout=60.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
        raise TimeoutError("Arm feedback/action checkpoint timed out")

    def send(targets, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint(positions=targets)
        point.time_from_start.sec = seconds
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        wait(future.done)
        handle = future.result()
        assert handle.accepted, "Trajectory rejected"
        return handle

    record = {"passed": False, "scope": "actual ros2_control/PhysX; no MoveIt or grasp"}
    try:
        wait(lambda: all(name in latest for name in names) and client.server_is_ready(), 30.)
        initial = [latest[name] for name in names]
        targets = initial.copy()
        targets[-1] += .10
        handle = send(targets, 3)
        result = handle.get_result_async()
        wait(result.done)
        record["move"] = {"status": result.result().status, "code": result.result().result.error_code,
                          "errors_rad": {name: abs(latest[name]-target) for name, target in zip(names, targets)}}
        assert record["move"]["status"] == 4 and record["move"]["code"] == 0, record
        assert max(record["move"]["errors_rad"].values()) < .025, record
        handle = send(initial, 5)
        start = time.monotonic()
        wait(lambda: time.monotonic()-start > .5)
        cancellation = handle.cancel_goal_async()
        wait(cancellation.done)
        assert cancellation.result().goals_canceling, "Cancellation rejected"
        result = handle.get_result_async()
        wait(result.done)
        assert result.result().status == 5, result.result()
        record["cancel_status"] = result.result().status
        stopped_after = time.monotonic()

        def stopped():
            recent = confirmation_window([(s['wall'], s['stamp'], s) for s in samples
                                          if s['wall'] > stopped_after], time.monotonic())
            return bool(recent) and all(
                all(name in sample["velocities"] and abs(sample["velocities"][name]) < .02 for name in names)
                for _, _, sample in recent)

        wait(stopped)
        result = send(initial, 3).get_result_async()
        wait(result.done)
        assert result.result().status == 4 and result.result().result.error_code == 0
        stamps = [sample["stamp"] for sample in samples]
        assert all(b > a for a, b in zip(stamps, stamps[1:])), "Joint feedback stamps are not increasing"
        record.update(passed=True, stop_confirmed=True, samples=len(samples))
    except BaseException as error:
        record["error"] = repr(error)
        raise
    finally:
        Path("/output/evaluations/arm-samples.json").write_text(json.dumps(samples)+"\n")
        Path("/output/evaluations/arm.json").write_text(json.dumps(record, indent=2)+"\n")
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
