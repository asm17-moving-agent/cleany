"""Copy a contiguous NumPy payload directly into a ROS-owned byte array."""
from array import array

import numpy as np


def numpy_byte_array(values: np.ndarray) -> array:
    contiguous = np.ascontiguousarray(values)
    payload = array('B')
    if contiguous.size:
        payload.frombytes(memoryview(contiguous).cast('B'))
    return payload
