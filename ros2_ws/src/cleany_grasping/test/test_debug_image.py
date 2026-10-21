import numpy as np
import pytest

from cleany_grasping.core.models import PointCloud, RawGrasp
from cleany_grasping.debug_image import (
    _display_candidate_indices,
    debug_image_message,
    render_grasp_debug_image,
)


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


def test_debug_view_limits_candidates_but_keeps_selected() -> None:
    candidates = tuple(
        RawGrasp(
            rotation=np.eye(3),
            translation=np.array((float(index), 0.0, 0.04)),
            width_m=0.04,
            depth_m=0.0,
            score=float(12 - index),
        )
        for index in range(12)
    )

    visible = _display_candidate_indices(
        candidates,
        selected_index=11,
        maximum_displayed_candidates=5,
    )

    assert visible == (0, 1, 2, 3, 11)


@pytest.mark.parametrize('empty_context', [False, True])
def test_bulk_context_drawing_matches_point_by_point_pixels(monkeypatch, empty_context):
    from PIL import ImageDraw
    rng = np.random.default_rng(20261012)
    target = cloud(rng.uniform(-.1, .1, (30, 3)))
    context = cloud([] if empty_context else rng.uniform(-1., 1., (400, 3)))
    candidate = RawGrasp(np.eye(3), np.zeros(3), .04, .01, .9)
    current = render_grasp_debug_image(target, context, [candidate], image_size=300)
    original = ImageDraw.ImageDraw.point
    def point_by_point(self, points, fill=None):
        for point in points:
            original(self, point, fill)
    monkeypatch.setattr(ImageDraw.ImageDraw, 'point', point_by_point)
    expected = render_grasp_debug_image(target, context, [candidate], image_size=300)
    assert np.array_equal(current, expected)


def test_debug_message_owns_strided_rgb_payload_and_accepts_empty_rgb():
    rgb = np.arange(72, dtype=np.uint8).reshape(4, 6, 3)[:, ::-1]
    message = debug_image_message(rgb, 1, 'frame')
    expected = rgb.tobytes()
    rgb.fill(0)
    assert message.data.tobytes() == expected
    assert not debug_image_message(rgb[:0], 1, 'frame').data
