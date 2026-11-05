"""Nested wall-clock spans for execution diagnostics, without control effects."""
from __future__ import annotations

from contextvars import ContextVar
from functools import wraps
import json
import time
from typing import Any, Callable
from uuid import uuid4


_parent_span: ContextVar[str | None] = ContextVar('execution_timing_parent', default=None)


def timed_operation(phase: str, *, node_attribute: str | None = None) -> Callable:
    """Log inclusive durations and parent IDs; preserve returns and failures.

    Enable with the owning pipeline's execution_timing_enabled parameter.
    Service spans include transport and waiting, not server-only CPU time.
    Nested durations must not be added to their inclusive parent duration.
    """
    def decorate(method: Callable) -> Callable:
        @wraps(method)
        def measured(owner: Any, *args: Any, **kwargs: Any) -> Any:
            node = getattr(owner, node_attribute, None) if node_attribute else owner
            if not getattr(node, '_execution_timing_enabled', False):
                return method(owner, *args, **kwargs)
            span_id = uuid4().hex
            parent = _parent_span.get()
            token = _parent_span.set(span_id)
            started_ns, wall_started_ns = time.monotonic_ns(), time.time_ns()
            context = getattr(node, 'execution_context', None)
            execution_id = getattr(getattr(context, 'goal', None), 'execution_id', '')
            bt_node = getattr(node, 'current_node', '')
            outcome = 'returned'
            try:
                return method(owner, *args, **kwargs)
            except BaseException:
                outcome = 'raised'
                raise
            finally:
                elapsed = (time.monotonic_ns() - started_ns) / 1e9
                _parent_span.reset(token)
                record = dict(stage='execution_timing', phase=phase, span_id=span_id,
                              parent_span_id=parent, execution_id=execution_id,
                              bt_node=bt_node, started_wall_ns=wall_started_ns,
                              wall_sec=elapsed, outcome=outcome)
                if phase == 'service_call' and args:
                    record['service'] = getattr(args[0], 'srv_name', '')
                try:
                    node.get_logger().info(json.dumps(record, allow_nan=False))
                except Exception:
                    # A diagnostic sink must never change motor control or cancel.
                    pass
        return measured
    return decorate
