"""XYZ PointCloud2 transport shared by voxel mapping and the simulation guard."""
from __future__ import annotations
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def xyz_cloud(points: np.ndarray, frame: str, stamp) -> PointCloud2:
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    msg = PointCloud2(header=Header(frame_id=frame, stamp=stamp), height=1, width=len(points),
                      is_bigendian=False, is_dense=True, point_step=12, row_step=12*len(points))
    msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.data = points.tobytes()
    return msg


def cloud_xyz(msg: PointCloud2) -> np.ndarray:
    fields = {field.name: field for field in msg.fields}
    if not all(k in fields and fields[k].datatype == PointField.FLOAT32 and fields[k].count == 1
               and 0 <= fields[k].offset <= msg.point_step-4 for k in ('x', 'y', 'z')):
        raise ValueError('Expected float32 XYZ fields')
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.row_step*msg.height:
        raise ValueError('Truncated XYZ cloud')
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                          offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=msg.point_step))
    array = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    return np.column_stack([array[k].ravel() for k in ('x', 'y', 'z')])
