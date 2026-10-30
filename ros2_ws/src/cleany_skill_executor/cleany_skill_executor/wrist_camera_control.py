"""Keep USB capture open only for the selected arm's observation window."""
from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from rclpy.node import Node
from std_srvs.srv import SetBool


class WristCameraSession:
    def __init__(self, node: Node, timeout_seconds: float, wait: Callable[..., Any]) -> None:
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError('Wrist camera control timeout must be finite and positive')
        self.timeout_seconds = timeout_seconds
        self.wait = wait
        self.clients = {arm: node.create_client(SetBool, f'/{arm}_wrist_camera/set_enabled')
                        for arm in ('left', 'right')}
        self.active_arm: str | None = None
        # Record before sending enable: an aborted/timed-out reply can still open USB.
        self._requested_arms: set[str] = set()
        self._disable_pending: dict[str, Any] = {}

    def _set(self, arm: str, enabled: bool, *, cleanup: bool = False) -> None:
        client = self.clients[arm]
        if not client.wait_for_service(timeout_sec=self.timeout_seconds):
            raise RuntimeError(f'{arm} wrist camera control service unavailable')
        future = client.call_async(SetBool.Request(data=enabled))
        self._acknowledge(arm, future, cleanup=cleanup)

    def _acknowledge(self, arm: str, future: Any, *, cleanup: bool) -> None:
        if cleanup:
            # Closing capture must remain possible after the motion guard has aborted.
            deadline = time.monotonic() + self.timeout_seconds
            while not future.done() and time.monotonic() < deadline:
                time.sleep(.01)
            if not future.done():
                raise RuntimeError(f'Timed out disabling {arm} wrist camera')
            response = future.result()
        else:
            response = self.wait(future, self.timeout_seconds)
        if response is None or not response.success:
            message = response.message if response is not None else 'No response'
            raise RuntimeError(f'{arm} wrist camera control failed: {message}')

    def activate(self, arm: str) -> None:
        if arm not in self.clients:
            raise ValueError('Unknown wrist camera arm')
        if self.active_arm == arm and arm not in self._disable_pending:
            return
        other = 'right' if arm == 'left' else 'left'
        # Require the opposite capture to be closed before opening the selected one.
        self._requested_arms.add(other)
        self._set(other, False)
        self._requested_arms.discard(other)
        self._disable_pending.pop(other, None)
        if self.active_arm == other:
            self.active_arm = None
        self._requested_arms.add(arm)
        self._disable_pending.pop(arm, None)
        self._set(arm, True)
        self.active_arm = arm

    def close(self) -> None:
        errors = []
        for arm in sorted(self._requested_arms):
            try:
                pending = self._disable_pending.pop(arm, None)
                if pending is None:
                    self._set(arm, False, cleanup=True)
                else:
                    self._acknowledge(arm, pending, cleanup=True)
            except Exception as error:
                errors.append(str(error))
            else:
                self._requested_arms.discard(arm)
                if self.active_arm == arm:
                    self.active_arm = None
        if errors:
            raise RuntimeError('; '.join(errors))

    def request_close(self) -> None:
        """Submit capture shutdown without delaying trajectory cancellation."""
        for arm in sorted(self._requested_arms):
            if arm not in self._disable_pending:
                self._disable_pending[arm] = self.clients[arm].call_async(SetBool.Request(data=False))
