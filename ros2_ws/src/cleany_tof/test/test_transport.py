import json
import pytest
from cleany_tof.transport import Stream


def frame(seq=1, time_us=1000000):
    return dict(type='frame', seq=seq, time_us=time_us, read_us=36000, zones=64, hz=15,
                distance_mm=[1000]*64, sigma_mm=[2]*64, target_status=[5]*64,
                target_count=[1]*64, valid=[True]*64)


def test_partial_frames_corrupt_lines_and_bounded_backlog():
    stream = Stream()
    data = json.dumps(frame()).encode()+b'\n'
    assert stream.feed(data[:11]) == []
    assert stream.feed(data[11:]) == [frame()]
    with pytest.raises(ValueError):
        stream.feed(b'{oops}\n')
    with pytest.raises(ValueError):
        stream.feed(b'x'*131073)


def test_replay_restart_and_queued_frame_detection():
    s = Stream()
    assert s.frame(frame(), 2.) == pytest.approx(2.-.036-1/15)
    with pytest.raises(ValueError, match='replayed'):
        s.frame(frame(), 2.01)
    with pytest.raises(ValueError, match='delayed'):
        s.frame(frame(2, 1066667), 2.5)


@pytest.mark.parametrize('change', [dict(zones=16), dict(hz=10), dict(valid=[False]*64),
    dict(sigma_mm=[float('nan')]*64), dict(distance_mm=[]), dict(read_us=500000), dict(seq=-1),
    dict(valid=[1]*64), dict(target_status=[-1]*64)])
def test_malformed_quality_and_metadata(change):
    f = frame()
    f.update(change)
    with pytest.raises(ValueError):
        Stream().frame(f, 2.)
