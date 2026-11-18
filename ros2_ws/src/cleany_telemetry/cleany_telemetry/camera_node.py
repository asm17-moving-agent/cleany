"""Independent ROS camera preview provider; no Backend or motion dependencies."""
import asyncio
import threading

import rclpy
from aiohttp import web
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .camera import Cameras
from .camera_http import create_app


class CameraPreviewNode(Node):
    """Serve recent RGB previews to local consumers without opening camera devices.

    Subscriptions:
        Requested Image or CompressedImage topics discovered in the ROS graph.
        Sensor-data QoS, best effort and volatile. Leases expire without HTTP reads.

    Notes:
        Publishes no ROS topics, services or actions. HTTP binds to loopback only.
        Runs independently of the pose relay, Dashboard Backend and dev_monitor.
    """
    def __init__(self, **kwargs):
        super().__init__('camera_preview', **kwargs)
        defaults = {'host': '127.0.0.1', 'port': 8769, 'camera_max_streams': 2,
                    'camera_fps': 5.0, 'lease_seconds': 3.0, 'stale_seconds': 2.0,
                    'discovery_seconds': 0.5}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        config = {name: self.get_parameter(name).value for name in defaults}
        if config['host'] not in {'127.0.0.1', '::1'} or not 0 <= config['port'] <= 65535:
            raise ValueError('Camera preview requires a loopback address and valid port')
        if not 0 < config['discovery_seconds'] <= config['lease_seconds']:
            raise ValueError('Discovery interval must fit within the preview lease')
        self.cameras = Cameras(limit=config['camera_max_streams'], fps=config['camera_fps'],
                               lease_seconds=config['lease_seconds'], stale_seconds=config['stale_seconds'])
        self.subscriptions_by_topic = {}
        self.stop_event, self.started = threading.Event(), threading.Event()
        self.worker_error = None
        self.http_port = config['port']
        self.worker = threading.Thread(target=self._serve, args=(config,), daemon=True)
        self.worker.start()
        if not self.started.wait(5) or self.worker_error:
            self.stop_event.set()
            self.worker.join(timeout=5)
            self.destroy_node()
            raise RuntimeError('Camera preview HTTP server failed to start') from self.worker_error
        self.create_timer(config['discovery_seconds'], self.discover)

    def discover(self):
        if self.worker_error is not None:
            raise RuntimeError("Camera preview HTTP worker stopped") from self.worker_error
        self.cameras.discover(dict(self.get_topic_names_and_types()))
        topics = self.cameras.catalog()
        wanted = self.cameras.selected()
        for name, (subscription, kind) in list(self.subscriptions_by_topic.items()):
            if name not in wanted or topics.get(name) != kind:
                self.destroy_subscription(subscription)
                del self.subscriptions_by_topic[name]
        for name in wanted:
            if name not in self.subscriptions_by_topic and name in topics:
                kind = topics[name]
                message_type = Image if kind == 'sensor_msgs/msg/Image' else CompressedImage
                subscription = self.create_subscription(message_type, name,
                    lambda message, topic=name: self.cameras.receive(topic, message), qos_profile_sensor_data)
                self.subscriptions_by_topic[name] = (subscription, kind)

    def _serve(self, config):
        async def run():
            runner = web.AppRunner(create_app(self.cameras))
            await runner.setup()
            try:
                site = web.TCPSite(runner, config['host'], config['port'])
                await site.start()
                self.http_port = runner.addresses[0][1]
                self.started.set()
                while not self.stop_event.is_set():
                    await asyncio.sleep(.1)
            finally:
                await runner.cleanup()
        try:
            asyncio.run(run())
        except (OSError, RuntimeError, ValueError) as exc:
            self.worker_error = exc
            self.started.set()

    def destroy_node(self):
        self.stop_event.set()
        self.worker.join(timeout=5)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
