import threading
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.core.continuous_tracking import ContinuousTracking, TrackingFrame


def test_latest_only_preserves_one_session_and_requires_post_request_frame():
    entered, proceed, finished = threading.Event(), threading.Event(), threading.Event()
    latest = [2]
    calls = []
    rgb = np.zeros((2, 2, 3), np.uint8)
    mask = np.ones((2, 2), bool)

    def track(frame):
        calls.append(int(frame[0, 0, 0]))
        if len(calls) == 1:
            entered.set()
            assert proceed.wait(2.)
        return mask

    def next_frame(after):
        if after >= 9:
            finished.wait(2.)
            raise ValueError('end of test')
        return TrackingFrame(latest[0], rgb + latest[0])

    session = SimpleNamespace(track=track, close=lambda: None)
    starts = []
    def start():
        starts.append(True)
        return session
    worker = ContinuousTracking(start, next_frame, 1)
    try:
        assert entered.wait(2.)
        latest[0] = 9  # Frames 3..8 never enter a growing inference queue.
        proceed.set()
        result = worker.wait_after(8, timeout_seconds=2., now_ns=lambda: 10,
                                   maximum_age_seconds=1.)
        assert result.frame.stamp_ns == 9
        assert result.processed_frames == 2 and calls == [2, 9] and len(starts) == 1
    finally:
        worker.close()
        finished.set()
        worker.close(timeout_seconds=2.)
    with pytest.raises(ValueError, match='cleared'):
        worker.wait_after(8, timeout_seconds=.02, now_ns=lambda: 10, maximum_age_seconds=1.)


@pytest.mark.parametrize('stamp,now,after', [(2, 10_000_000_000, 1), (2, 1, 1), (2, 3, 2)])
def test_old_future_and_pre_request_results_do_not_pass(stamp, now, after):
    done, release = threading.Event(), threading.Event()
    rgb = np.zeros((2, 2, 3), np.uint8)
    def next_frame(previous):
        if previous == stamp:
            release.wait(2.)
            raise ValueError('finished')
        return TrackingFrame(stamp, rgb)
    worker = ContinuousTracking(
        lambda: SimpleNamespace(track=lambda rgb: np.ones((2, 2), bool), close=lambda: None),
        next_frame, 0, lambda result: done.set())
    try:
        assert done.wait(2.)
        with pytest.raises(ValueError, match='No fresh post-request'):
            worker.wait_after(after, timeout_seconds=.02, now_ns=lambda: now, maximum_age_seconds=1.)
    finally:
        worker.close()
        release.set()
        worker.close(timeout_seconds=2.)


def test_inference_failure_is_latched_and_session_is_released():
    closed = threading.Event()
    errors = []
    def fail(rgb):
        raise ValueError('lost mask')
    worker = ContinuousTracking(
        lambda: SimpleNamespace(track=fail, close=closed.set),
        lambda after: TrackingFrame(2, np.zeros((2, 2, 3), np.uint8)), 1,
        on_error=lambda error: errors.append(str(error)))
    with pytest.raises(ValueError, match='lost mask'):
        worker.wait_after(1, timeout_seconds=2., now_ns=lambda: 3, maximum_age_seconds=1.)
    assert closed.wait(2.)
    assert errors == ['lost mask']
    worker.close(timeout_seconds=2.)
