import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

from cleany_perception.adapters.tf2_transform import (
    Tf2TransformAdapter,
    rotation_matrix_from_quaternion,
)


class _Buffer:
    def __init__(self, transform) -> None:
        self.transform = transform
        self.calls = []

    def lookup_transform(self, target, source, stamp, timeout):
        self.calls.append(
            (target, source, stamp.nanoseconds, timeout.nanoseconds)
        )
        return self.transform


def test_tf2_adapter_returns_target_from_source_transform():
    message = TransformStamped()
    message.transform.translation.x = 1.0
    message.transform.translation.y = 2.0
    message.transform.translation.z = 3.0
    message.transform.rotation.x = 1.0
    message.transform.rotation.w = 0.0
    buffer = _Buffer(message)
    adapter = Tf2TransformAdapter(
        object(),
        timeout_seconds=0.25,
        buffer=buffer,
    )

    transform = adapter.lookup('base_link', 'camera', 123)

    assert transform.translation == pytest.approx((1.0, 2.0, 3.0))
    assert transform.rotation == pytest.approx(np.diag((1.0, -1.0, -1.0)))
    assert buffer.calls == [('base_link', 'camera', 123, 250_000_000)]


def test_quaternion_conversion_normalizes_input():
    rotation = rotation_matrix_from_quaternion(0.0, 0.0, 0.0, 2.0)

    assert rotation == pytest.approx(np.eye(3))


def test_tf2_adapter_rejects_non_positive_cache_duration():
    with pytest.raises(ValueError, match='cache duration'):
        Tf2TransformAdapter(object(), cache_seconds=0.0, buffer=object())
