from __future__ import annotations

from array import array

import numpy as np

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
    # Keep the existing little-endian XYZ + 0x00RRGGBB wire layout.
    payload = array('B', [0]) * message.row_step
    packed = np.frombuffer(payload, dtype=[('xyz', '<f4', (3,)), ('rgb', 'u1', (4,))])
    packed['xyz'] = cloud.points
    packed['rgb'][:, :3] = cloud.colors[:, ::-1]
    packed['rgb'][:, 3] = 0
    message.data = payload
    message.is_dense = True
    return message
