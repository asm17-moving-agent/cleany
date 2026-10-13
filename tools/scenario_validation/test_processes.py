from pathlib import Path
from processes import signal_owned


def test_reused_pid_with_other_output_is_not_signalled(monkeypatch):
    monkeypatch.setattr(Path,'read_bytes',lambda _:b'python instance.py --output /another/run')
    monkeypatch.setattr('processes.os.kill',lambda *_:(_ for _ in ()).throw(AssertionError('unowned kill')))
    assert not signal_owned(100,Path('/owned/run'),b'instance.py')


def test_owned_observer_is_signalled(monkeypatch):
    calls=[]
    monkeypatch.setattr(Path,'read_bytes',lambda _:b'python instance.py --output /owned/run')
    monkeypatch.setattr('processes.os.kill',lambda *args:calls.append(args))
    assert signal_owned(100,Path('/owned/run'),b'instance.py')
    assert calls[0][0]==100


def test_group_signal_requires_recorded_process_to_be_group_leader(monkeypatch):
    monkeypatch.setattr(Path,'read_bytes',lambda _:b'ros2 launch --output_directory=/owned/run/navigation')
    monkeypatch.setattr('processes.os.getpgid',lambda _:999)
    monkeypatch.setattr('processes.os.killpg',lambda *_:(_ for _ in ()).throw(AssertionError('unowned group')))
    assert not signal_owned(100,Path('/owned/run'),b'ros2',group=True)
