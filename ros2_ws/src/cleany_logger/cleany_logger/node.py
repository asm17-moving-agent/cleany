"""Independent headless recording node; no web or Monitor imports."""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
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
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = LoggerNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok(): rclpy.shutdown()
