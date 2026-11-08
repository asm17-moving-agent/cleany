"""Provide a private RViz TF tree using the extension SLAM pose."""
import math
import copy

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import MarkerArray


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def correction(map_pose, odom_pose):
    """Return map->odom from two poses of the same base frame."""
    x, y, angle = map_pose
    ox, oy, oa = odom_pose
    a = angle-oa
    return x-math.cos(a)*ox+math.sin(a)*oy, y-math.sin(a)*ox-math.cos(a)*oy, a


class ViewTF(Node):
    """Forward odometry TF and add SLAM map->odom only on a private topic."""
    def __init__(self):
        super().__init__('map_extension_view_tf')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.out = self.create_publisher(TFMessage, '/map_extension/view_tf', 100)
        self.pending = None
        self.trajectory_out = self.create_publisher(MarkerArray, "/map_extension/view_trajectory", 10)
        self.create_subscription(MarkerArray, "/map_extension/trajectory_node_list", self.trajectory, 10)
        self.create_subscription(TFMessage, '/tf', self.forward, 100)
        self.create_subscription(PoseStamped, '/map_extension/tracked_pose', self.receive, 10)
        self.create_timer(.05, self.update)

    def trajectory(self, msg):
        # This display-only copy does not alter the recorded Cartographer data.
        display = copy.deepcopy(msg)
        for marker in display.markers:
            if marker.ns == 'Trajectory 0':
                marker.color.r, marker.color.g, marker.color.b = .25, .4, 1.
                marker.color.a = .65
                marker.scale.x = .04
            else:
                marker.color.r, marker.color.g, marker.color.b = 1., .85, .05
                marker.color.a = 1.
                marker.scale.x = .08
        self.trajectory_out.publish(display)

    def forward(self, msg):
        transforms = [t for t in msg.transforms if t.header.frame_id != 'map' and t.child_frame_id != 'odom']
        if transforms:
            self.out.publish(TFMessage(transforms=transforms))

    def receive(self, msg):
        self.pending = msg
        self.update()

    def update(self):
        if self.pending is None:
            return
        pose = self.pending
        try:
            odom = self.buffer.lookup_transform('odom', 'base_link', Time.from_msg(pose.header.stamp))
        except Exception:
            return
        p, t = pose.pose, odom.transform
        x,y,a = correction((p.position.x,p.position.y,yaw(p.orientation)),(t.translation.x,t.translation.y,yaw(t.rotation)))
        result = TransformStamped()
        result.header = pose.header
        result.child_frame_id = 'odom'
        result.transform.translation.x, result.transform.translation.y = x,y
        result.transform.rotation.z, result.transform.rotation.w = math.sin(a/2),math.cos(a/2)
        self.out.publish(TFMessage(transforms=[result]))
        self.pending = None


def main():
    rclpy.init()
    node = ViewTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
