from array import array
import struct

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

from cleany_grasping.grasp_node import point_cloud_from_message


def organized_cloud(big_endian=False, rgb_datatype=PointField.FLOAT32):
    offsets = {'rgb': 0, 'z': 4, 'x': 12, 'y': 16}
    points = [(1., 2., 3.), (-0., -2., .5), (np.nan, 1., 2.),
              (-3., 4., 5.), (1., np.inf, 2.), (1., 2., -np.inf)]
    colors = [0xAA0180FF, 0x00112233, 0x00000000, 0x00FE0304, 0, 0]
    row_step, point_step = 77, 24
    payload = bytearray(row_step * 2)
    endian = '>' if big_endian else '<'
    for index, (point, color) in enumerate(zip(points, colors)):
        start = (index // 3) * row_step + (index % 3) * point_step
        for name, value in zip(('x', 'y', 'z'), point):
            struct.pack_into(endian + 'f', payload, start + offsets[name], value)
        struct.pack_into(endian + 'I', payload, start, color)
    return PointCloud2(
        height=2, width=3, point_step=point_step, row_step=row_step,
        fields=[PointField(name=name, offset=offsets[name],
                           datatype=rgb_datatype if name == 'rgb' else PointField.FLOAT32,
                           count=1) for name in ('y', 'rgb', 'x', 'z')],
        is_bigendian=big_endian, data=array('B', payload),
    )


@pytest.mark.parametrize('big_endian', [False, True])
@pytest.mark.parametrize('rgb_datatype', [PointField.FLOAT32, PointField.UINT32])
def test_cloud_decode_preserves_padded_rows_offsets_rgb_bits_and_finite_filter(
    big_endian, rgb_datatype,
):
    message = organized_cloud(big_endian, rgb_datatype)
    message = deserialize_message(serialize_message(message), PointCloud2)
    cloud = point_cloud_from_message(message)
    np.testing.assert_array_equal(cloud.points, [(1., 2., 3.), (-0., -2., .5), (-3., 4., 5.)])
    np.testing.assert_array_equal(cloud.colors, np.array([(1, 128, 255), (17, 34, 51), (254, 3, 4)]) / 255.)
    assert np.signbit(cloud.points[1, 0])
    assert cloud.points.dtype == np.float64 and cloud.colors.dtype == np.float64
    message.data[:] = array('B', bytes(len(message.data)))
    assert cloud.points[0, 0] == 1. and cloud.colors[0, 0] == 1. / 255.


@pytest.mark.parametrize('empty_kind', ['width', 'height', 'nonfinite'])
def test_cloud_decode_empty_results_retain_nx3_float64_contract(empty_kind):
    message = organized_cloud()
    if empty_kind == 'nonfinite':
        for row in range(2):
            for column in range(3):
                struct.pack_into('<f', message.data, row * message.row_step + column * message.point_step + 12, np.nan)
    else:
        setattr(message, empty_kind, 0)
        message.data = array('B')
    cloud = point_cloud_from_message(message)
    assert cloud.points.shape == cloud.colors.shape == (0, 3)
    assert cloud.points.dtype == cloud.colors.dtype == np.float64


@pytest.mark.parametrize('invalid_kind', ['missing_rgb', 'point_step', 'row_step', 'truncated', 'offset'])
def test_cloud_decode_rejects_invalid_layout(invalid_kind):
    message = organized_cloud()
    if invalid_kind == 'missing_rgb':
        message.fields = [field for field in message.fields if field.name != 'rgb']
    elif invalid_kind == 'offset':
        message.fields[0].offset = message.point_step
    elif invalid_kind == 'point_step':
        message.point_step = 0
    elif invalid_kind == 'row_step':
        message.row_step = message.width * message.point_step - 1
    else:
        message.data = array('B', message.data[:10])
    with pytest.raises(ValueError):
        point_cloud_from_message(message)
