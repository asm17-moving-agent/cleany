"""독립적인 화면 없는 기록 노드이며 웹이나 Monitor를 import하지 않습니다."""
import rclpy
import signal
import threading
from rclpy.exceptions import InvalidHandle
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from cleany_interfaces.msg import RecordingStatus
from cleany_interfaces.srv import RecordingControl

from .controller import RecordingWorker
from .observer import ObservationNode
from .resources import ResourceWorker
from .slam_reader import SlamReader
from .wire import config_dict, config_message, status_message


class LoggerNode(ObservationNode):
    """관측 입력과 별도 worker를 연결해 기록 session의 생명주기를 소유합니다.

    Subscriptions:
        센서, TF와 런타임 관측은 ObservationNode의 설정된 소스 선택을 따릅니다.

    Publishers:
        ~/status (cleany_interfaces/msg/RecordingStatus): 현재 session과 설정 revision.
            reliable, transient_local, depth 1입니다.

    Services:
        ~/{start,stop,configure,get_config} (cleany_interfaces/srv/RecordingControl):
            기록 제어 Server. stop에는 현재 session_id가 필요하며 요청은 worker로
            전달하고 비동기 결과를 기다립니다.

    Notes:
        RecordingWorker가 저장소를, ResourceWorker가 호스트 지표 수집을 담당합니다.
        노드 종료는 worker를 닫고 기록을 flush한 후 ROS를 정리합니다. 웹 UI 없이
        동작하며 Monitor의 생존 기간에 기록을 묶지 않습니다.
    """

    def __init__(self, **kwargs):
        super().__init__(node_name='cleany_logger', record=True, slam_factory=SlamReader,
            extra_defaults={'record_on_start': True}, **kwargs)
        self.recording_worker = RecordingWorker(self.hub, self.config,
            record_on_start=self.config['record_on_start'])
        try:
            self.recording_worker.start()
            self.apply_sources()
            self.resource_worker = ResourceWorker(self.hub, self.config)
            self.resource_worker.thread.start()
            self.status_group = ReentrantCallbackGroup()
            self.status_publisher = self.create_publisher(RecordingStatus, '~/status', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.recording_services = [self.create_service(RecordingControl, '~/' + operation,
                self.handler(operation))
                for operation in ('start', 'stop', 'configure', 'get_config')]
            self.create_timer(.5, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME),
                              callback_group=self.status_group)
            self.publish_status()
        except BaseException:
            if hasattr(self, 'resource_worker'):
                self.resource_worker.stop.set()
                self.resource_worker.thread.join(timeout=5)
            if self.recording_worker.thread.is_alive():
                self.recording_worker.close()
            super().destroy_node()
            raise

    def publish_status(self):
        view = self.recording_worker.snapshot()
        if view and rclpy.ok():
            self.status_publisher.publish(status_message(view))

    def handler(self, operation):
        async def callback(request, response):
            try:
                if operation == 'stop' and not request.session_id:
                    raise ValueError('Stop requires the current session_id')
                arguments = dict(session_id=request.session_id, expected_revision=request.expected_revision)
                if operation == 'configure':
                    arguments['settings'] = config_dict(request.config)
                pending = self.recording_worker.submit(operation, **arguments)
                future = Future(executor=self.executor)
                def complete(done):
                    if done.exception(): future.set_exception(done.exception())
                    else: future.set_result(done.result())
                pending.add_done_callback(complete)
                view = await future
                if operation == 'configure':
                    self.apply_sources()
                response.success = True
                response.reason = ''
            except Exception as exc:
                response.success = False
                response.reason = str(exc)
                view = self.recording_worker.snapshot()
            response.config = config_message(view['settings'], self.config)
            response.status = status_message(view)
            self.publish_status()
            return response
        return callback

    def destroy_node(self):
        self.resource_worker.stop.set()
        self.resource_worker.thread.join(timeout=5)
        self.recording_worker.close()
        self.publish_status()
        return super().destroy_node()


def main(args=None):
    # launch 종료 시에도 writer가 flush할 때까지 ROS를 유지합니다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    closed = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: closed.set())
    signal.signal(signal.SIGTERM, lambda *_: closed.set())
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = LoggerNode()
        executor.add_node(node)
        while not closed.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except InvalidHandle:
                # Humble에서는 탐색이 구독을 교체하기 전에 콜백이 큐에 들어갈 수 있습니다.
                node.get_logger().debug(
                    "Discarded a callback for a retired subscription"
                )
    finally:
        executor.shutdown(timeout_sec=5.0)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
