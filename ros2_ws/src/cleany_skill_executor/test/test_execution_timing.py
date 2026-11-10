"""Timing remains observational across nesting, cancellation, and sink errors."""
import json
from types import SimpleNamespace

import pytest

from cleany_skill_executor.execution_timing import timed_operation


def test_uninstrumented_port_does_not_require_a_ros_node():
    @timed_operation('wait', node_attribute='node')
    def method(owner):
        return owner.value

    assert method(SimpleNamespace(value=42)) == 42


def test_nested_spans_preserve_results_and_parent_ids(monkeypatch):
    import cleany_skill_executor.execution_timing as module
    ticks = iter((0, 10, 30, 50))
    monkeypatch.setattr(module.time, 'monotonic_ns', lambda: next(ticks))
    records = []

    class Node:
        _execution_timing_enabled = True
        current_node = 'CarryObject'
        execution_context = SimpleNamespace(goal=SimpleNamespace(execution_id='pick-1'))

        def get_logger(self):
            return SimpleNamespace(info=lambda text: records.append(json.loads(text)))

        @timed_operation('outer')
        def outer(self):
            return self.inner(42)

        @timed_operation('inner')
        def inner(self, value):
            return value

    assert Node().outer() == 42
    child, parent = records
    assert child['parent_span_id'] == parent['span_id']
    assert parent['parent_span_id'] is None
    assert parent['wall_sec'] == 50e-9 and child['wall_sec'] == 20e-9
    assert parent['execution_id'] == 'pick-1' and parent['bt_node'] == 'CarryObject'
    assert parent['outcome'] == child['outcome'] == 'returned'


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('sink_fails', [False, True])
def test_cancellation_and_context_survive_diagnostic_failure(enabled, sink_fails):
    records = []

    def log(text):
        if sink_fails:
            raise OSError('diagnostic storage unavailable')
        records.append(json.loads(text))

    node = SimpleNamespace(_execution_timing_enabled=enabled,
                           get_logger=lambda: SimpleNamespace(info=log))

    class Hardware:
        @timed_operation('wait', node_attribute='node')
        def wait(self):
            raise KeyboardInterrupt('cancel')

    hardware = Hardware()
    hardware.node = node
    for _ in range(2):
        with pytest.raises(KeyboardInterrupt, match='cancel'):
            hardware.wait()
    assert len(records) == (2 if enabled and not sink_fails else 0)
    assert all(r['outcome'] == 'raised' and r['parent_span_id'] is None for r in records)
