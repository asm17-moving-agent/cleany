import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('dds_transport', (Path(__file__).resolve().parents[2] / 'tools').joinpath('dds_transport.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('current,existing,expected', [
    (212992, '', 8388608),
    (16777216, '', 16777216),
    (212992, 'net.core.rmem_max = 33554432 # existing\n', 33554432),
])
def test_persistence_does_not_lower_existing_limits(current, existing, expected):
    assert module.persistent_limit(current, existing) == expected


@pytest.mark.parametrize('limit,actual,ready', [
    (212992, 425984, False),
    (8388608, 425984, False),
    (8388608, 16777216, True),
    (None, 16777216, True),
    (None, 425984, False),
])
def test_preflight_checks_obtained_socket_buffer_not_only_sysctl(monkeypatch, limit, actual, ready):
    class Socket:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def setsockopt(self, level, option, requested):
            assert requested == module.MINIMUM_BYTES

        def getsockopt(self, *_):
            return actual

    def read_limit():
        if limit is None:
            raise FileNotFoundError('host-wide sysctl hidden in container')
        return str(limit)

    monkeypatch.setattr(module, 'KERNEL_LIMIT', SimpleNamespace(read_text=read_limit))
    monkeypatch.setattr(module.socket, 'socket', lambda *_: Socket())
    assert module.check_receive_buffer()[0] is ready
