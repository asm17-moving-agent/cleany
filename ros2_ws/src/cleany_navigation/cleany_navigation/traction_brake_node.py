"""BT 요청 제동은 충돌 검사 전에 적용하며 검사 뒤에 적용하지 않습니다."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from .traction_brake import BrakeRamp


class TractionBrake(Node):
    """자동 명령 경로에서 BT가 요청한 정지를 점진적으로 적용합니다.

    Subscriptions:
        motion/cmd_vel (Twist): 기존 자동 이동 의도, depth 1.
        navigation/traction_brake (Bool): BT 제동 heartbeat, depth 1.
    Publishers:
        motion/traction_cmd_vel (Twist): collision monitor 입력, 50Hz.
    Notes:
        heartbeat나 명령 입력이 없으면 0을 강제합니다. 수동 주행은 기존 경로를
        유지하며 collision STOP은 이 점진적 변경을 즉시 덮어쓸 수 있습니다.
    """
    def __init__(self):
        super().__init__('traction_brake')
        self.ramp = BrakeRamp(self.declare_parameter('linear_deceleration', .2).value,
                              self.declare_parameter('angular_deceleration', .4).value,
                              self.declare_parameter('wheel_acceleration', 3.).value,
                              self.declare_parameter('wheel_radius', .0635).value,
                              self.declare_parameter('wheel_arm', .47).value)
        self.timeout = self.declare_parameter('heartbeat_timeout_s', .5).value
        self.command_timeout = self.declare_parameter('command_timeout_s', .3).value
        self.command = (0., 0., 0.)
        self.command_at = self.brake_at = 0.
        self.brake = True
        self.last = time.monotonic()
        self.publisher = self.create_publisher(Twist, 'motion/traction_cmd_vel', 1)
        self.create_subscription(Twist, 'motion/cmd_vel', self.on_command, 1)
        self.create_subscription(Bool, 'navigation/traction_brake', self.on_brake, 1)
        self.create_timer(.02, self.tick)

    def on_command(self, message):
        self.command = (message.linear.x, message.linear.y, message.angular.z)
        self.command_at = time.monotonic()

    def on_brake(self, message):
        self.brake, self.brake_at = message.data, time.monotonic()

    def tick(self):
        now = time.monotonic()
        fresh = now-self.brake_at <= self.timeout and now-self.command_at <= self.command_timeout
        output = self.ramp.step(self.command, self.brake, now-self.last, fresh)
        self.last = now
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = output
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TractionBrake()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
