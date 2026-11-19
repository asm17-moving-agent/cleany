"""ROS client for recording control. No local writer or offline write fallback."""
import asyncio
import copy
import threading
import time
from uuid import uuid4
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from cleany_interfaces.msg import RecordingStatus
from cleany_interfaces.srv import RecordingControl
from cleany_logger.controller import ControlConflict
from cleany_logger.recording import defaults
from cleany_logger.wire import config_dict, config_message, status_dict


class LoggerUnavailable(RuntimeError):
    pass


class LoggerClient:
    def __init__(self, node, config):
        self.node, self.config = node, config
        self.timeout = config['logger_timeout_s']
        prefix = config['logger_namespace'].rstrip('/')
        self.clients = {op: node.create_client(RecordingControl, prefix + '/' + op)
                        for op in ('start', 'stop', 'configure', 'get_config')}
        self.lock = threading.Lock()
        self.latest = None
        self.received = 0.
        self.settings = None
        self.subscription = node.create_subscription(RecordingStatus, prefix + '/status', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive(self, message):
        with self.lock:
            if self.latest and self.latest['boot_id'] != message.boot_id:
                self.settings = None
            self.latest = status_dict(message)
            self.received = time.monotonic()

    def status(self):
        with self.lock:
            if self.latest and time.monotonic() - self.received <= self.timeout:
                return copy.deepcopy(self.latest)
        return dict(boot_id='', session_id='', state='UNAVAILABLE', reason='logger 연결 끊김', revision=0,
                    status={}, stored_bytes=0, updated_at=0.)

    async def call(self, operation, settings=None, *, session_id='', revision=0):
        client = self.clients[operation]
        if not client.service_is_ready():
            raise LoggerUnavailable('logger 서비스를 사용할 수 없습니다.')
        request = RecordingControl.Request(request_id=str(uuid4()), session_id=session_id, expected_revision=revision)
        if settings is not None:
            request.config = config_message(settings, self.config)
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout
        while not future.done():
            if time.monotonic() >= deadline:
                # Timeout is not a stop acknowledgement; retain the last observed state.
                client.remove_pending_request(future)
                raise LoggerUnavailable('logger 응답 대기 시간이 초과됐습니다. 상태를 다시 확인하세요.')
            await asyncio.sleep(.02)
        response = future.result()
        if not response.success:
            raise ControlConflict(response.reason)
        with self.lock:
            self.settings = config_dict(response.config)
            self.latest = status_dict(response.status)
            self.received = time.monotonic()
        return self.view()

    def view(self):
        status = self.status()
        with self.lock:
            settings = copy.deepcopy(self.settings or defaults())
        settings['active'] = status['state'] == 'RECORDING'
        return dict(status, settings=settings)

    async def get_view(self):
        # Configuration may be changed by another ROS client.
        return await self.call('get_config')

    async def change(self, settings, *, new_session=False):
        current = await self.get_view()
        if not settings['active']:
            if current['state'] in ('RECORDING', 'STOPPING'):
                return await self.call('stop', session_id=current['session_id'])
            return await self.call('configure', settings, revision=current['revision'])
        if current['state'] == 'RECORDING':
            if new_session:
                return await self.call('start')
            raise ControlConflict('녹화를 정지한 후 설정을 변경하세요.')
        await self.call('configure', settings, revision=current['revision'])
        return await self.call('start')
