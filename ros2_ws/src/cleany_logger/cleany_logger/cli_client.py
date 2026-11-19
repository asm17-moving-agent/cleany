"""Bounded synchronous ROS transport for the one-shot logger CLI."""
from __future__ import annotations

import math
import time
from uuid import uuid4

import rclpy
from cleany_interfaces.srv import RecordingControl
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.exceptions import InvalidTopicNameException
from rclpy.node import Node
from rclpy.validate_full_topic_name import validate_full_topic_name

from .cli import LoggerRejected, LoggerUnavailable
from .wire import config_dict, config_message, status_dict


class RosControlClient:
    def __init__(self, node_path: str, timeout: float):
        prefix = node_path.rstrip('/')
        try:
            validate_full_topic_name(prefix)
        except InvalidTopicNameException as exc:
            raise ValueError(str(exc)) from exc
        if not node_path.startswith('/') or not prefix:
            raise ValueError('--node must be an absolute logger node path')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('--timeout must be finite and positive')
        self.timeout = timeout
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node('cleany_logger_cli_' + uuid4().hex[:8], context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.clients = {op: self.node.create_client(RecordingControl, prefix + '/' + op)
                        for op in ('get_config', 'start', 'stop', 'configure')}

    def call(self, operation: str, *, settings: dict | None = None,
             session_id: str = '', revision: int = 0) -> dict:
        client = self.clients[operation]
        deadline = time.monotonic() + self.timeout
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise LoggerUnavailable(f'Logger service unavailable: {client.srv_name}; check node path and ROS domain')
        request = RecordingControl.Request(request_id=str(uuid4()), session_id=session_id,
                                           expected_revision=revision)
        if settings is not None:
            request.config = config_message(settings)
        pending = client.call_async(request)
        try:
            self.executor.spin_until_future_complete(pending,
                timeout_sec=max(0., deadline - time.monotonic()))
            if not pending.done():
                raise LoggerUnavailable('Logger response timed out; outcome is unknown. Recheck status before retrying')
            response = pending.result()
            if not response.success:
                raise LoggerRejected(response.reason)
            view = status_dict(response.status)
            view['settings'] = config_dict(response.config)
            view['settings']['active'] = view['state'] == 'RECORDING'
            return view
        finally:
            if not pending.done():
                client.remove_pending_request(pending)

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()
