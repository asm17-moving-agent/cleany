"""Full-view depth projection without detection or simulator geometry."""
from __future__ import annotations

import numpy as np
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from cleany_perception.core.geometry import deproject_masked_depth
from cleany_perception.core.models import CameraIntrinsics
from cleany_perception.rgbd_snapshot import _depth_array, _validate_camera_info


def depth_scene_cloud(
    depth: Image,
    info: CameraInfo,
    minimum_depth_m: float,
    maximum_depth_m: float,
    pixel_stride: int,
    depth_16u_scale_m: float = 0.001,
) -> PointCloud2:
    """Project every sampled valid pixel, including unrecognized objects.

    Input must be rectified depth with matching optical-frame intrinsics.
    Invalid / missing depth is omitted, never fabricated as free space.
    """
    if not (0 < minimum_depth_m < maximum_depth_m < float('inf')):
        raise ValueError('Depth limits must be finite, positive and ordered')
    if pixel_stride < 1 or not (0 < depth_16u_scale_m < float('inf')):
        raise ValueError('Stride and depth scale must be positive')
    _validate_camera_info(info, depth)
    if (not depth.header.frame_id
            or depth.header.frame_id != info.header.frame_id):
        raise ValueError('Depth and CameraInfo must share an optical frame')
    if not np.isfinite(info.d).all() or any(abs(v) > 1e-9 for v in info.d):
        raise ValueError('Rectified depth / zero-distortion info required')
    intrinsics = CameraIntrinsics(
        depth.width, depth.height, info.k[0], info.k[4], info.k[2], info.k[5]
    )
    array = _depth_array(depth, depth_16u_scale_m)
    points = deproject_masked_depth(
        array, intrinsics, np.ones(array.shape, dtype=bool),
        minimum_depth_m, maximum_depth_m, pixel_stride,
    ).astype('<f4')
    cloud = PointCloud2()
    cloud.header = depth.header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name=name, offset=index * 4,
                   datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(('x', 'y', 'z'))
    ]
    cloud.point_step = 12
    cloud.row_step = 12 * cloud.width
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.data = points.tobytes()
    return cloud
