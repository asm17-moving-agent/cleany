from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
import time

import pytest

from cleany_grasping.gpu_scheduling import GpuLease


def tokens(tmp_path: Path) -> str:
    path=tmp_path/'gpu.lock'
    for p in (path,Path(str(path)+'.demand')):
        p.write_text('shared read-only token\n')
        p.chmod(0o444)
    return str(path)


def test_readonly_shared_tokens_exclude_background_and_support_nested_calls(tmp_path):
    path=tokens(tmp_path);a=GpuLease(path);b=GpuLease(path)
    with a.acquire() as admitted:
        assert admitted and b.foreground_pending()
        with a.acquire(foreground=False) as nested:
            assert nested
        with b.acquire(foreground=False,blocking=False) as other:
            assert not other
    assert not b.foreground_pending()
    with b.acquire(foreground=False,blocking=False) as admitted:
        assert admitted


def test_pending_real_work_makes_background_yield_before_next_stage(tmp_path):
    path=tokens(tmp_path);background=GpuLease(path);foreground=GpuLease(path)
    finished=threading.Event()
    def real_request():
        with foreground.acquire() as admitted:
            assert admitted
            finished.set()
    with ThreadPoolExecutor(max_workers=1) as pool:
        with background.acquire(foreground=False) as admitted:
            assert admitted
            task=pool.submit(real_request)
            deadline=time.monotonic()+3
            while not background.foreground_pending() and time.monotonic()<deadline:
                time.sleep(.01)
            assert background.foreground_pending() and not finished.is_set()
            with GpuLease(path).acquire(foreground=False,blocking=False) as next_stage:
                assert not next_stage
        task.result(timeout=3)
    assert finished.is_set() and not background.foreground_pending()


def test_canceled_wait_releases_foreground_intent(tmp_path):
    path=tokens(tmp_path);a=GpuLease(path);b=GpuLease(path)
    with a.acquire(foreground=False):
        with pytest.raises(InterruptedError,match='canceled'):
            with b.acquire(continue_work=lambda:False):
                pytest.fail('Canceled work must not run')
        assert not a.foreground_pending()


def test_missing_shared_token_pair_fails_before_gpu_work(tmp_path):
    with pytest.raises(ValueError,match='shared lock'):
        GpuLease(str(tmp_path/'missing'))
