"""단일 기준 /odom과 odom → base_link TF를 발행합니다."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomRelay(Node):
    """휠 odometry를 전달하고 기준 베이스 좌표 변환의 발행을 담당합니다.

    Subscriptions:
        wheel/odom (nav_msgs/msg/Odometry): 휠 적분으로 얻은 위치와 속도.

    Publishers:
        odom (nav_msgs/msg/Odometry): 같은 표본에 기준 좌표계 이름을 적용한 결과.

    TF:
        입력 위치와 시각으로 odom → base_link를 발행합니다.

    Notes:
        좌표값은 변환하지 않고 이름만 바꾸므로 입력이 이미 odom → base_link를
        표현해야 합니다. 같은 TF를 발행하는 다른 노드를 함께 실행하지 않습니다.
    """
    def __init__(self):
        super().__init__('base_odom_relay')
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'wheel/odom', self.on_odom, 10)

    def on_odom(self, m):
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        self.pub.publish(m)
        t = TransformStamped()
        t.header, t.child_frame_id = m.header, 'base_link'
        t.transform.translation.x = m.pose.pose.position.x
        t.transform.translation.y = m.pose.pose.position.y
        t.transform.translation.z = m.pose.pose.position.z
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
