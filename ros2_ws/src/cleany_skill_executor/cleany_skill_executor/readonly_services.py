"""Overlap independent read-only RPCs while preserving the owner's wait guards."""
from collections.abc import Sequence
from contextlib import suppress
import math
import time
from typing import Any

from .execution_timing import timed_operation


class ReadOnlyBatch:
    """Own a single-use set of independent reads, including unused futures."""

    def __init__(self, node: Any, calls: Sequence[tuple[Any, Any]], *,
                 timeout: float = 30., availability_timeout: float = 5.) -> None:
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Read-only service timeout must be positive')
        self.node, self.pending, self.closed = node, [], False
        self.check = getattr(node, 'check_execution', lambda: None)
        try:
            for client, _ in calls:
                self.check()
                if not client.wait_for_service(timeout_sec=availability_timeout):
                    raise RuntimeError(f'Service unavailable: {client.srv_name}')
            self.started = time.monotonic()
            self.deadline = self.started+timeout
            for client, request in calls:
                self.check()
                self.pending.append((client, client.call_async(request)))
        except BaseException:
            self.close()
            raise

    def result(self) -> list[Any]:
        if self.closed:
            raise RuntimeError('Read-only service batch was already consumed or closed')
        try:
            responses = []
            for client, future in self.pending:
                self.check()
                remaining = self.deadline-time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('Read-only service batch timed out')
                response = self.node.wait(future, remaining)
                if response is None:
                    raise RuntimeError(f'Empty service response: {client.srv_name}')
                if hasattr(response, 'success') and not response.success:
                    raise RuntimeError(getattr(response, 'message', '')
                                       or f'Service call failed: {client.srv_name}')
                responses.append(response)
            self.check()
            return responses
        finally:
            self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        for client, future in self.pending:
            if not future.done():
                with suppress(Exception):
                    future.cancel()
            with suppress(Exception):
                client.remove_pending_request(future)


@timed_operation('parallel_service_calls')
def call_readonly_services(node: Any, calls: Sequence[tuple[Any, Any]], *, timeout: float = 30.) -> list[Any]:
    """Wait with owner cancellation/deadline guards; discard failed batches."""
    return ReadOnlyBatch(node, calls, timeout=timeout).result()
