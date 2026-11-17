"""Display the same ChArUco board through the operational calibrated cameras."""
import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from cleany_perception.rgbd_snapshot import _rgb_array


class BoardObserver(Node):
    """Publish /camera_calibration/boards for RViz; subscribe to images and TF only."""

    def __init__(self):
        super().__init__('calibrated_board_observer')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(MarkerArray, '/camera_calibration/boards', 5)
        self.images, self.infos = {}, {}
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.board = cv2.aruco.CharucoBoard_create(7, 5, .05, .035, self.dictionary)
        self.colors = [(1., .62, .1), (.1, .88, .94), (.9, .2, .85)]
        self.subs = []
        for name, prefix in [('head', '/camera/color'), ('left', '/left_wrist_camera'),
                             ('right', '/right_wrist_camera')]:
            self.subs.append(self.create_subscription(Image, prefix+'/image_raw',
                lambda msg, key=name: self.images.__setitem__(key, msg), qos_profile_sensor_data))
            self.subs.append(self.create_subscription(CameraInfo, prefix+'/camera_info',
                lambda msg, key=name: self.infos.__setitem__(key, msg), qos_profile_sensor_data))
        self.create_timer(.3, self.observe)

    def observe(self):
        markers = []
        for i, name in enumerate(('head', 'left', 'right')):
            if name not in self.images or name not in self.infos:
                continue
            image, info = self.images[name], self.infos[name]
            stamp = rclpy.time.Time.from_msg(image.header.stamp)
            if (self.get_clock().now() - stamp).nanoseconds > 500_000_000:
                continue
            gray = cv2.cvtColor(_rgb_array(image), cv2.COLOR_RGB2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if ids is None:
                continue
            _, points, ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
            if ids is None or len(ids) < 8:
                continue
            k, d = np.array(info.k).reshape(3, 3), np.array(info.d)
            ok, rv, tv = cv2.solvePnP(self.board.chessboardCorners[ids.ravel()], points, k, d)
            if not ok:
                continue
            try:
                transform = self.buffer.lookup_transform('base_link', image.header.frame_id, stamp).transform
            except TransformException:
                continue
            q, t = transform.rotation, transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            outline = np.array([[0, 0, 0], [.35, 0, 0], [.35, .25, 0],
                                [0, .25, 0], [0, 0, 0]])
            pixels = (outline @ cv2.Rodrigues(rv)[0].T + tv.ravel()) @ rotation.T
            pixels += [t.x, t.y, t.z]
            m = Marker()
            m.header.frame_id, m.header.stamp = 'base_link', image.header.stamp
            m.id, m.type, m.action = i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w, m.scale.x = 1., .003
            m.color.r, m.color.g, m.color.b = self.colors[i]
            m.color.a = 1.
            m.lifetime = Duration(seconds=.6).to_msg()
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in pixels]
            markers.append(m)
        self.publisher.publish(MarkerArray(markers=markers))


if __name__ == '__main__':
    cv2.setNumThreads(1)
    rclpy.init()
    node = BoardObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
