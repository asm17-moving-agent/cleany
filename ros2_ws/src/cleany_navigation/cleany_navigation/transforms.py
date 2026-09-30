"""Point transforms shared by ROS adapters, without a ROS import."""
from __future__ import annotations

import numpy as np


def transform(points: np.ndarray, pose) -> np.ndarray:
    q, t = pose.rotation, pose.translation
    values = np.array([q.x, q.y, q.z, q.w, t.x, t.y, t.z], dtype=float)
    if not np.isfinite(values).all() or not np.isclose(np.linalg.norm(values[:4]), 1., atol=1e-3):
        raise ValueError('Invalid transform')
    x, y, z, w = values[:4]
    matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return np.asarray(points) @ matrix.T + values[4:]
