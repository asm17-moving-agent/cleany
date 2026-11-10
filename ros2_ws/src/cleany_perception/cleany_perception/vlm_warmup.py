"""Best-effort, ordered VLM-window notifications; never wait for GPU readiness."""
from __future__ import annotations

import threading

from std_srvs.srv import SetBool


class VlmWarmupClient:
    def __init__(self, node, service: str, callback_group=None, *, prepare_service: str = '') -> None:
        self._node = node
        self._client = node.create_client(SetBool, service, callback_group=callback_group)
        self._prepare_client = (node.create_client(SetBool, prepare_service, callback_group=callback_group)
                                if prepare_service else None)
        self._prepared = False
        self._failed_start = False
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
            if active and not self._desired:
                self._failed_start = False
            self._desired = active
            self._send()

    def _send(self) -> None:
        if self._pending:
            return
        if self._desired:
            if self._failed_start:
                return
            preparing = self._prepare_client is not None and not self._prepared
            if not preparing and self._sent:
                return
            active = True
        else:
            if self._sent:
                preparing = False
            elif self._prepare_client is not None and self._prepared:
                preparing = True
            else:
                return
            active = False
        client = self._prepare_client if preparing else self._client
        if not client.service_is_ready():
            return  # Optional optimization; no new startup/readiness wait.
        try:
            future = client.call_async(SetBool.Request(data=active))
        except Exception as error:
            self._node.get_logger().warning(f'VLM IK warmup notification unavailable: {error}')
            return
        self._pending = True
        if preparing:
            self._prepared = active
        else:
            self._sent = active
        future.add_done_callback(lambda value: self._completed(value, active=active))

    def _completed(self, future, *, active: bool = False) -> None:
        with self._lock:
            self._pending = False
            try:
                response = future.result()
                if not response.success:
                    if active:
                        self._failed_start = True
                    self._node.get_logger().info('VLM IK warmup skipped: ' + response.message)
            except Exception as error:
                if active:
                    self._failed_start = True
                self._node.get_logger().warning(f'VLM IK warmup notification failed: {error}')
            # End cannot overtake start even if VLM finishes before the first
            # service response. A server lease also bounds abandoned sessions.
            self._send()
