from copy import deepcopy

import pytest
from sensor_msgs.msg import PointCloud2

from cleany_perception.scene_cloud_receipt_node import receipt_for_cloud


def cloud():
    message = PointCloud2(width=2, height=1, point_step=12, row_step=24, data=bytes(24))
    message.header.frame_id = 'rgb_optical'
    message.header.stamp.sec = 7
    return message


def test_capture_stamp_is_preserved_not_replaced_by_current_time():
    source = cloud()
    before = deepcopy(source)
    receipt = receipt_for_cloud(source)
    assert receipt == before.header
    source.header.stamp.sec = 99
    assert receipt.stamp.sec == 7


@pytest.mark.parametrize('field,value', [
    ('width', 0), ('height', 0), ('point_step', 0), ('row_step', 1), ('data', bytes(23)),
])
def test_empty_or_truncated_cloud_never_creates_a_receipt(field, value):
    source = cloud()
    setattr(source, field, value)
    assert receipt_for_cloud(source) is None


def test_missing_frame_or_capture_stamp_is_rejected():
    source = cloud()
    source.header.frame_id = ''
    assert receipt_for_cloud(source) is None
    source = cloud()
    source.header.stamp.sec = 0
    assert receipt_for_cloud(source) is None
