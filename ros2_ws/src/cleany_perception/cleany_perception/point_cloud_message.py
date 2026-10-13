from __future__ import annotations

import struct

from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField

from cleany_perception.core.point_cloud import ColoredPointCloud


def colored_point_cloud_message(
    cloud: ColoredPointCloud,
    stamp: Time,
    frame_id: str,
) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = len(cloud.points)
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='rgb', offset=12, datatype=PointField.FLOAT32, count=1
        ),
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.point_step * message.width
    payload = bytearray(message.row_step)
    for index, (point, color) in enumerate(zip(cloud.points, cloud.colors)):
        packed_rgb = (
            (int(color[0]) << 16) | (int(color[1]) << 8) | int(color[2])
        )
        struct.pack_into('<fffI', payload, index * 16, *point, packed_rgb)
    message.data = bytes(payload)
    message.is_dense = True
    return message
