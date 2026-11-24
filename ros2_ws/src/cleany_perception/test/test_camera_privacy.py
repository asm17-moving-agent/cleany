import numpy as np
import pytest

from cleany_perception.privacy import FILL, cover_box, opaque_frame


def test_masks_are_clipped_opaque_and_leave_other_pixels_unchanged():
    raw = np.arange(900, dtype=np.uint8).reshape(15, 20, 3)
    mask = np.zeros(raw.shape[:2], np.uint8)
    cover_box(mask, (-4, -1, 6.5, 3.2), .1)
    out = opaque_frame(raw, mask)
    assert np.all(out[:4, :7] == FILL)
    np.testing.assert_array_equal(out[-3:], raw[-3:])
    assert not np.shares_memory(out, raw)
    with pytest.raises(ValueError):
        cover_box(mask, (1, 3, np.nan, 6), .1)
    with pytest.raises(ValueError):
        opaque_frame(raw, mask.T)


def test_display_hold_requires_continuity_and_expires():
    from cleany_perception.privacy_history import ScreenHistory
    rng = np.random.default_rng(43)
    bgr = rng.integers(0, 256, (120, 160, 3), dtype=np.uint8)
    screen = np.zeros(bgr.shape[:2], np.uint8)
    screen[30:70, 40:80] = 255
    missing = np.zeros_like(screen)
    h = ScreenHistory()
    h.apply(bgr, screen, 1.)
    assert h.apply(bgr, missing, 1.1)[50, 60]
    assert not h.apply(bgr, missing, 1.65).any()
    h.apply(bgr, screen, 3.)
    assert not h.apply(bgr, missing, 1.).any()
    h.apply(bgr, screen, 4.)
    assert not h.apply(np.zeros_like(bgr), missing, 4.1).any()


def test_video_withholds_first_frame_after_start_and_interruption(monkeypatch):
    from cleany_perception.privacy import CameraRedactor
    from cleany_perception.privacy_history import ScreenHistory
    redactor = CameraRedactor.__new__(CameraRedactor)
    redactor.history = ScreenHistory()
    monkeypatch.setattr(redactor, '_objects',
                        lambda bgr, mask, counts, stamp: redactor.history.apply(bgr, mask, stamp))
    monkeypatch.setattr(redactor, '_faces', lambda view, mask: 0)
    monkeypatch.setattr(redactor, '_letters', lambda view, mask: 0)
    frame = np.zeros((48, 64, 3), np.uint8)
    for stamp, withheld in ((1., True), (1.2, False), (2., True), (1.8, True), (2., False)):
        assert (redactor.redact(frame, stamp) is None) == withheld
    assert redactor.redact(frame).image.shape == frame.shape
    assert redactor.redact(frame, 2.1) is None
