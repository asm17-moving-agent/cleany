"""Best-effort, ordered VLM-window notifications; never wait for GPU readiness."""
from __future__ import annotations

import threading

from std_srvs.srv import SetBool


class VlmWarmupClient:
    def __init__(self, node, service: str, callback_group=None) -> None:
        self._node = node
        self._client = node.create_client(SetBool, service, callback_group=callback_group)
        self._lock = threading.RLock()
        self._pending = False
        self._desired = False
        self._sent = False
        self._retry_timer = node.create_timer(.1, self._retry, callback_group=callback_group)

    def _retry(self) -> None:
        with self._lock:
            self._send()

    def set_active(self, active: bool) -> None:
        with self._lock:
            self._desired = active
            self._send()

    def _send(self) -> None:
        if self._pending or self._sent == self._desired:
            return
        if not self._client.service_is_ready():
            return  # Optional optimization; no new startup/readiness wait.
        active = self._desired
        try:
            future = self._client.call_async(SetBool.Request(data=active))
        except Exception as error:
            self._node.get_logger().warning(f'VLM IK warmup notification unavailable: {error}')
            return
        self._pending, self._sent = True, active
        future.add_done_callback(self._completed)

    def _completed(self, future) -> None:
        with self._lock:
            self._pending = False
            try:
                response = future.result()
                if not response.success:
                    self._node.get_logger().info('VLM IK warmup skipped: ' + response.message)
            except Exception as error:
                self._node.get_logger().warning(f'VLM IK warmup notification failed: {error}')
            # End cannot overtake start even if VLM finishes before the first
            # service response. A server lease also bounds abandoned sessions.
            self._send()
