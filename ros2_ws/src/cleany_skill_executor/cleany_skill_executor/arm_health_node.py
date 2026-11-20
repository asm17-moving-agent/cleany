"""System Supervisor에 팔 제어기 가용성을 발행합니다."""
from time import monotonic

import rclpy
from cleany_interfaces.msg import ComponentHealth
from controller_manager_msgs.srv import ListControllers
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .core.arm_health import AVAILABLE, UNAVAILABLE, ArmHealth, ArmHealthConfig

STATUS = {AVAILABLE: ComponentHealth.AVAILABLE, UNAVAILABLE: ComponentHealth.UNAVAILABLE}


class ArmHealthMonitor(Node):
    """ros2_control과 관절 피드백에서 source=arm ComponentHealth를 보고합니다.

    Subscriptions:
        joint_states (sensor_msgs/msg/JointState): 팔 관절 위치.
            joint_states_topic 기본값이며 QoS는 sensor_data입니다.

    Publishers:
        system/component_health (cleany_interfaces/msg/ComponentHealth):
            20Hz이며 health_topic 기본값입니다.

    Services:
        controller_manager/list_controllers (controller_manager_msgs/srv/ListControllers):
            poll_s마다 조회하는 Client. controller_manager가 기본 prefix입니다.

    Notes:
        가용성만 보고합니다. 현재 팔 상태에서 베이스가 움직여도 되는지는 이 보고가
        아니라 조작 결과의 safe_to_drive가 나타냅니다.
    """

    def __init__(self, **kwargs):
        super().__init__('arm_health_monitor', **kwargs)
        self.poll_s = self.declare_parameter('poll_s', .5).value
        self.health = ArmHealth(ArmHealthConfig(
            controllers=tuple(self.declare_parameter(
                'controllers', ['left_arm_controller', 'right_arm_controller']).value),
            joints=tuple(self.declare_parameter('joints', [
                'left_shoulder_yaw_joint', 'left_shoulder_pitch_joint', 'left_elbow_pitch_joint',
                'left_wrist_pitch_joint', 'left_wrist_roll_joint', 'right_shoulder_yaw_joint',
                'right_shoulder_pitch_joint', 'right_elbow_pitch_joint', 'right_wrist_pitch_joint',
                'right_wrist_roll_joint']).value),
            joint_timeout_s=self.declare_parameter('joint_timeout_s', .3).value,
            controller_timeout_s=3 * self.poll_s))
        manager = self.declare_parameter('controller_manager', 'controller_manager').value
        self.client = self.create_client(ListControllers, manager + '/list_controllers')
        self.pending = None
        self.create_subscription(JointState, self.declare_parameter('joint_states_topic', 'joint_states').value,
                                 lambda msg: self.health.joint_state(msg.name, msg.position, monotonic()),
                                 qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            ComponentHealth, self.declare_parameter('health_topic', 'system/component_health').value, 10)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.poll_s, self._poll, clock=steady)
        self.create_timer(.05, self._publish, clock=steady)

    def _poll(self):
        if self.pending is not None and not self.pending.done():
            self.client.remove_pending_request(self.pending)
        if self.client.service_is_ready():
            self.pending = self.client.call_async(ListControllers.Request())
            self.pending.add_done_callback(self._listed)

    def _listed(self, future):
        try:
            controllers = future.result().controller
        except Exception:
            return  # 이전 표본은 controller_timeout_s에 따라 만료됩니다.
        self.health.controller_states({c.name: c.state for c in controllers}, monotonic())

    def _publish(self):
        status, detail = self.health.status(monotonic())
        msg = ComponentHealth(source='arm', status=STATUS.get(status, ComponentHealth.UNKNOWN), detail=detail)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
