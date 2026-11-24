"""Publish the CAD wheel envelope, start point and submaps for the laptop RViz display."""

import ast
import math
from pathlib import Path

import rclpy
import yaml
from cartographer_ros_msgs.msg import SubmapList
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

FOOTPRINT = (
    Path(__file__).resolve().parents[4]
    / "ros2_ws/src/cleany_navigation/config/lidar_robot.yaml"
)


def point(x, y, z=0.0):
    return Point(x=float(x), y=float(y), z=float(z))


def marker(frame, name, ident, kind, rgb, width):
    m = Marker()
    m.header.frame_id = frame
    m.ns = name
    m.id = ident
    m.type = kind
    m.frame_locked = frame == "base_link"
    m.pose.orientation.w = 1.0
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 1.0)
    return m


class Display(Node):
    def __init__(self):
        super().__init__("floor19_view_overlays")
        params = yaml.safe_load(FOOTPRINT.read_text())["local_costmap"][
            "local_costmap"
        ]["ros__parameters"]
        envelope = ast.literal_eval(params["footprint"])
        outline = marker(
            "base_link",
            "wheel envelope",
            0,
            Marker.LINE_STRIP,
            (0.0, 0.50, 0.56),
            0.085,
        )
        outline.points = [point(x, y, 0.12) for x, y in envelope + [envelope[0]]]
        arrow = marker("base_link", "forward", 0, Marker.ARROW, (1.0, 0.59, 0.04), 0.11)
        arrow.points = [point(-0.19, 0, 0.16), point(0.57, 0, 0.16)]
        arrow.scale.y = 0.23
        arrow.scale.z = 0.22
        self.body = MarkerArray(markers=[outline, arrow])
        self.start = None
        self.submaps = MarkerArray()
        self.robot_pub = self.create_publisher(MarkerArray, "/floor19/view/robot", 2)
        self.start_pub = self.create_publisher(MarkerArray, "/floor19/view/start", 2)
        self.sub_pub = self.create_publisher(MarkerArray, "/floor19/view/submaps", 2)
        self.create_subscription(
            MarkerArray,
            "/floor19/trajectory_node_list",
            self.path,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            SubmapList, "/floor19/submap_list", self.submap, qos_profile_sensor_data
        )
        self.create_timer(0.3, self.render)

    def path(self, msg):
        line = next(
            (
                m
                for m in msg.markers
                if m.type == Marker.LINE_STRIP
                and len(m.points)
                and m.action == Marker.ADD
            ),
            None,
        )
        if line is None:
            return
        pos = line.points[0]
        ring = marker(
            line.header.frame_id,
            "first trajectory point",
            0,
            Marker.LINE_STRIP,
            (0.63, 0.37, 0.85),
            0.055,
        )
        ring.pose = line.pose
        ring.points = [
            point(
                pos.x + 0.55 * math.cos(i * math.tau / 44),
                pos.y + 0.55 * math.sin(i * math.tau / 44),
                0.09,
            )
            for i in range(45)
        ]
        self.start = MarkerArray(markers=[ring])

    def submap(self, msg):
        dots = marker(
            msg.header.frame_id,
            "submap origins",
            0,
            Marker.SPHERE_LIST,
            (0.2, 0.65, 1.0),
            0.18,
        )
        dots.scale.y = dots.scale.z = 0.18
        dots.points = [
            point(s.pose.position.x, s.pose.position.y, 0.12) for s in msg.submap
        ]
        self.submaps = MarkerArray(markers=[dots])

    def render(self):
        self.robot_pub.publish(self.body)
        if self.start:
            self.start_pub.publish(self.start)
        if self.submaps.markers:
            self.sub_pub.publish(self.submaps)


rclpy.init()
viewer = Display()
try:
    rclpy.spin(viewer)
finally:
    viewer.destroy_node()
    rclpy.try_shutdown()
