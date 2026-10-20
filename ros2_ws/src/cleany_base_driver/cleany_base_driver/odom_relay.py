"""The sole canonical /odom and odom->base_link TF publisher."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomRelay(Node):
    """Relay wheel odometry and own its canonical base transform.

    Subscriptions:
        wheel/odom (nav_msgs/msg/Odometry): Wheel-integrated pose and velocity.

    Publishers:
        odom (nav_msgs/msg/Odometry): The same sample with canonical frame IDs.

    TF:
        Broadcasts odom -> base_link using the input pose and timestamp.

    Notes:
        Relabels frames without transforming coordinates; the input pose must
        already represent odom -> base_link. Do not run a second TF owner.
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
