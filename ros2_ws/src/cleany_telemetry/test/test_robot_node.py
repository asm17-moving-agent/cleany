"""합성 ROS 입력 검증입니다. 로봇 제어 노드는 실행하지 않습니다."""
import json
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from cleany_interfaces.msg import SystemState
from cleany_telemetry.robot_node import RobotObserver
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import String


def test_real_ros_message_shapes_and_sources():
    rclpy.init()
    node = RobotObserver(parameter_overrides=[Parameter("provenance", value="sim")])
    fixture = rclpy.create_node("observation_fixture")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(fixture)
    received = []
    fixture.create_subscription(String, "/telemetry/robot_observations",
                                lambda m: received.append(json.loads(m.data)), 100)
    joints = fixture.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
    battery = fixture.create_publisher(BatteryState, "/battery_state", qos_profile_sensor_data)
    state = fixture.create_publisher(SystemState, "/system/state", QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            stamp = fixture.get_clock().now().to_msg()
            j = JointState(name=["left_gripper_joint"], position=[.3]); j.header.stamp = stamp
            b = BatteryState(percentage=.73, voltage=24.0, power_supply_status=1); b.header.stamp = stamp
            s = SystemState(epoch="fixture", operating_state="OPERATIONAL"); s.header.stamp = stamp
            joints.publish(j); battery.publish(b); state.publish(s)
            executor.spin_once(timeout_sec=.01)
            if {f["sample"]["kind"] for f in received} >= {"joints", "battery", "supervisor_state"}:
                break
        samples = {f["sample"]["kind"]: f for f in received}
        assert samples["battery"]["sample"]["percentage"] == pytest.approx(73)
        assert samples["joints"]["sample"]["positions"] == [.3]
        assert samples["supervisor_state"]["sample"]["operating_state"] == "OPERATIONAL"
        assert all(f["provenance"] == "sim" and f["source_stamp"] > 0 for f in received)
        assert not any(f["sample"]["kind"] == "joint_targets" for f in received)
        assert {p.topic_name for p in node.publishers} <= {
            "/telemetry/robot_observations", "/rosout", "/parameter_events"}
    finally:
        executor.shutdown()
        node.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()
