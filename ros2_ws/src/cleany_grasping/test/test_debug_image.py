import numpy as np

from cleany_grasping.core.models import PointCloud, RawGrasp
from cleany_grasping.debug_image import debug_image_message, render_grasp_debug_image


def cloud(points):
    values = np.asarray(points, dtype=float).reshape((-1, 3))
    return PointCloud(values, np.zeros_like(values))


def test_renders_candidates_and_builds_rgb_message() -> None:
    target = cloud(((-0.02, -0.01, 0.04), (0.02, 0.01, 0.05)))
    context = cloud(((-0.1, -0.1, 0.0), (0.1, 0.1, 0.0)))
    candidate = RawGrasp(
        rotation=np.eye(3),
        translation=np.array((0.0, 0.0, 0.04)),
        width_m=0.04,
        depth_m=0.0,
        score=0.9,
    )

    rendered = render_grasp_debug_image(
        target, context, (candidate,), image_size=300
    )
    message = debug_image_message(rendered, 123, 'camera_frame')

    assert rendered.shape == (300, 300, 3)
    assert rendered.dtype == np.uint8
    assert np.any(rendered[:, :, 1] > 200)
    assert message.encoding == 'rgb8'
    assert message.header.frame_id == 'camera_frame'
    assert message.width == 300 and message.height == 300
