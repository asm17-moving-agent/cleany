import struct

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

from cleany_perception.core.point_cloud import ColoredPointCloud
from cleany_perception.point_cloud_message import colored_point_cloud_message


@pytest.mark.parametrize('count', [0, 1, 19])
def test_colored_cloud_preserves_wire_layout_with_strided_input(count):
    points = np.arange(count * 6, dtype=np.float32).reshape(count, 6)[:, ::2]
    colors = np.arange(count * 6, dtype=np.uint8).reshape(count, 6)[:, ::2]
    if count:
        points[0] = (-0.0, -1.25, 2.5)
        colors[0] = (255, 128, 1)
    message = colored_point_cloud_message(
        ColoredPointCloud(points, colors), Time(sec=12, nanosec=34), 'camera',
    )
    expected = b''.join(
        struct.pack('<fffBBBB', *point, int(color[2]), int(color[1]), int(color[0]), 0)
        for point, color in zip(points, colors)
    )
    assert bytes(message.data) == expected
    assert (message.height, message.width, message.point_step, message.row_step) == (
        1, count, 16, 16 * count,
    )
    assert [(field.name, field.offset, field.datatype, field.count) for field in message.fields] == [
        ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('rgb', 12, 7, 1),
    ]
    assert not message.is_bigendian and message.is_dense
    decoded = deserialize_message(serialize_message(message), PointCloud2)
    assert decoded == message
    assert decoded.header.stamp == Time(sec=12, nanosec=34)
    assert decoded.header.frame_id == 'camera'


def test_colored_cloud_payload_owns_its_input_copy():
    cloud = ColoredPointCloud(np.ones((2, 3)), np.full((2, 3), 42, dtype=np.uint8))
    message = colored_point_cloud_message(cloud, Time(), 'camera')
    payload = bytes(message.data)
    cloud.points.fill(0)
    cloud.colors.fill(0)
    assert bytes(message.data) == payload
