"""Synthetic ROS/HTTP test with no Backend, monitor, drivers or motion publishers."""
import io
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import pytest

rclpy = pytest.importorskip('rclpy')
from cleany_telemetry.camera_node import CameraPreviewNode
from PIL import Image as PILImage
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


@pytest.mark.parametrize('compressed', [False, True])
def test_standalone_camera_and_subscription_release(compressed):
    rclpy.init()
    node=CameraPreviewNode(parameter_overrides=[Parameter('port',value=0)])
    source=rclpy.create_node('camera_fixture')
    kind=CompressedImage if compressed else Image
    publisher=source.create_publisher(kind,'/preview_fixture',qos_profile_sensor_data)
    if compressed:
        output=io.BytesIO()
        PILImage.new('RGB',(2,2),'red').save(output,format='JPEG')
        message=CompressedImage(format='jpeg',data=output.getvalue())
    else:
        message=Image(height=2,width=2,encoding='rgb8',step=6,data=[255,0,0]*4)
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(source)
    url=f'http://127.0.0.1:{node.http_port}/api/camera?topic=/preview_fixture'
    try:
        deadline=time.monotonic()+10
        image=None
        while time.monotonic()<deadline:
            publisher.publish(message)
            executor.spin_once(timeout_sec=.05)
            try:
                with urlopen(url,timeout=1) as response:
                    if response.status==200:
                        image=response.read()
                        break
            except HTTPError as error:
                assert error.code==400
        with urlopen(url,timeout=1) as another_consumer:
            assert another_consumer.status==200
        assert len(node.subscriptions_by_topic)==1
        assert image and PILImage.open(io.BytesIO(image)).size==(2,2)
        assert {name for name,_ in node.get_publisher_names_and_types_by_node(node.get_name(),'/')} <= {'/rosout','/parameter_events'}
        node.cameras.clock=lambda:time.monotonic()+5
        node.discover()
        assert not node.subscriptions_by_topic
    finally:
        node.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
