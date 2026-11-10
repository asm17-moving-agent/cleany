"""Exercise USB capture lifecycle through actual ROS SetBool and image topics."""
import time

import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from cleany_robot_interface import wrist_rgb_camera as module


class FakeUsb:
    def __init__(self, opened=True):
        self.opened, self.released, self.reads = opened, False, 0

    def isOpened(self):
        return self.opened

    def set(self, *args):
        return True

    def read(self):
        assert not self.released
        self.reads += 1
        return True, np.zeros((480, 640, 3), np.uint8)

    def release(self):
        self.released = True


@pytest.fixture
def camera(monkeypatch):
    profile = dict(arm='left', frame_id='left_wrist_rgb_optical_frame',
                   camera_device='fake-usb', width=640, height=480, publish_rate_hz=15.)
    intrinsic = dict(K=[500., 0., 320., 0., 500., 240., 0., 0., 1.], D=[0.] * 5)
    monkeypatch.setattr(module, 'load_camera_profile', lambda *_: (profile, intrinsic))
    captures = []
    opened = [True]

    def open_usb(*args):
        usb = FakeUsb(opened[0])
        captures.append(usb)
        return usb

    monkeypatch.setattr(module.cv2, 'VideoCapture', open_usb)
    context = Context()
    rclpy.init(context=context)
    executor = SingleThreadedExecutor(context=context)
    publisher = module.WristRgbCamera(context=context)
    client_node = Node('wrist_camera_test_client', context=context)
    executor.add_node(publisher)
    executor.add_node(client_node)
    control = client_node.create_client(SetBool, '/left_wrist_camera/set_enabled')
    assert control.wait_for_service(timeout_sec=3.)

    def toggle(enabled):
        future = control.call_async(SetBool.Request(data=enabled))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.done()
        return future.result()

    try:
        yield publisher, client_node, executor, captures, opened, toggle
    finally:
        publisher.destroy_node()
        client_node.destroy_node()
        executor.shutdown()
        context.shutdown()


def spin(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.02)


def test_idle_has_no_usb_capture_or_images_and_disable_releases_device(camera):
    publisher, client, executor, captures, _, toggle = camera
    images, infos = [], []
    client.create_subscription(Image, '/left_wrist_camera/image_raw', images.append, qos_profile_sensor_data)
    client.create_subscription(CameraInfo, '/left_wrist_camera/camera_info', infos.append, qos_profile_sensor_data)
    spin(executor, .25)
    assert not captures and not images and not infos
    assert toggle(True).success
    spin(executor, .3)
    assert len(captures) == 1 and images and infos
    assert images[-1].header == infos[-1].header
    assert (images[-1].width, images[-1].height, images[-1].encoding) == (640, 480, 'bgr8')
    assert toggle(True).success and len(captures) == 1
    assert toggle(False).success and captures[0].released
    spin(executor, .1)  # Drain frames already in flight before disable acknowledgement.
    count, reads = len(images), captures[0].reads
    spin(executor, .2)
    assert len(images) == count and captures[0].reads == reads
    assert toggle(True).success and len(captures) == 2
    assert toggle(False).success and captures[1].released


def test_unavailable_usb_fails_enable_without_leaking_capture_and_can_retry(camera):
    publisher, _, _, captures, opened, toggle = camera
    opened[0] = False
    response = toggle(True)
    assert not response.success and 'Cannot open' in response.message
    assert captures[0].released and publisher.camera is None
    assert toggle(False).success
    opened[0] = True
    assert toggle(True).success
    publisher.destroy_node()
    assert captures[-1].released


def test_preview_is_demand_driven_bounded_and_stops_with_capture(camera, monkeypatch):
    publisher, client, executor, captures, _, toggle = camera
    encodes = []
    original = module.cv2.imencode

    def encode(*args):
        encodes.append(time.monotonic())
        return original(*args)

    monkeypatch.setattr(module.cv2, 'imencode', encode)
    assert toggle(True).success
    spin(executor, .3)
    assert not encodes  # USB may be active; no viewer means no JPEG work.
    previews, images = [], []
    client.create_subscription(CompressedImage, '/left_wrist_camera/preview/compressed',
                               previews.append, qos_profile_sensor_data)
    client.create_subscription(Image, '/left_wrist_camera/image_raw', images.append, qos_profile_sensor_data)
    spin(executor, 1.2)
    assert 3 <= len(previews) <= 8 and len(images) > len(previews)
    assert previews[-1].format == 'bgr8; jpeg compressed bgr8'
    assert previews[-1].header in [image.header for image in images]
    decoded = module.cv2.imdecode(np.frombuffer(previews[-1].data, np.uint8), module.cv2.IMREAD_COLOR)
    assert decoded.shape == (480, 640, 3)
    assert toggle(False).success and captures[-1].released
    spin(executor, .1)
    counts = (len(previews), len(encodes))
    spin(executor, .3)
    assert counts == (len(previews), len(encodes))


def test_enabled_state_late_join_heartbeat_failure_and_disable(camera):
    publisher, client, executor, captures, opened, toggle = camera
    states = []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(Bool, '/left_wrist_camera/enabled', lambda m: states.append(m.data), qos)
    spin(executor, .3)
    assert states and states[-1] is False and not captures
    count = len(states); spin(executor, 1.1)
    assert len(states) > count and states[-1] is False
    opened[0] = False
    assert not toggle(True).success
    spin(executor, .1); assert states[-1] is False
    opened[0] = True
    assert toggle(True).success
    spin(executor, .1); assert states[-1] is True
    assert toggle(False).success
    spin(executor, .1); assert states[-1] is False
