"""Wire bytes and independent ownership for strided and empty NumPy inputs."""
from copy import deepcopy

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

from cleany_perception.byte_buffer import numpy_byte_array
from cleany_perception.wrist_service import WristService


@pytest.mark.parametrize('layout', ['contiguous', 'reverse_rows', 'reverse_channels', 'roi', 'empty'])
@pytest.mark.parametrize('dtype', [np.uint8, np.dtype('>f4')])
def test_numpy_byte_array_matches_wire_bytes_and_owns_its_storage(layout, dtype):
    source = np.arange(90, dtype=dtype).reshape(5, 6, 3)
    values = {'contiguous': source, 'reverse_rows': source[::-1],
              'reverse_channels': source[..., ::-1], 'roi': source[:, ::2],
              'empty': source[:0]}[layout]
    expected = values.tobytes()
    data = numpy_byte_array(values)
    assert data.typecode == 'B' and data.tobytes() == expected
    source.fill(0)
    assert data.tobytes() == expected


def test_wrist_debug_preserves_cdr_metadata_and_does_not_alias_image_or_pixels():
    from array import array
    rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    source = Image(height=2, width=3, encoding='bgr8', step=11,
                   is_bigendian=True, data=bytes(22))
    source.header.frame_id = 'right_wrist_rgb_optical_frame'
    source.header.stamp.sec = 42
    expected = deepcopy(source)
    expected.encoding, expected.step = 'rgb8', 9
    expected.data = array('B', rgb.tobytes())
    output = WristService._debug_image(source, rgb)
    # CDR alignment padding is not message content and need not be deterministic.
    assert deserialize_message(serialize_message(output), Image) == expected
    rgb.fill(0)
    source.header.frame_id = 'changed'
    source.data[0] = 1
    assert deserialize_message(serialize_message(output), Image) == expected
