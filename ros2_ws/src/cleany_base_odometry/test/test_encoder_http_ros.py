"""ROS integration against a loopback fixture; never contacts real hardware."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread
from time import monotonic

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cleany_interfaces.msg')

from cleany_interfaces.msg import WheelEncoderTicks
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from cleany_base_odometry.encoder_http_node import EncoderHttpNode


def test_ros_receives_stamped_raw_ticks_and_stops_on_bad_responses():
    state = {'healthy': True}
    requests = []

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def do_GET(self):
            requests.append((self.command, self.path))
            body = (
                b'{"encoders":[10,-20,30,-40],"protocol_version":1,'
                b'"boot_id":"0123456789abcdef","sample_seq":1,"sample_time_us":123456}'
                if state['healthy'] else b'bad'
            )
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rclpy.init(args=[
        '--ros-args', '-p', 'host:=127.0.0.1',
        '-p', f'port:={server.server_port}',
        '-p', 'output_topic:=test_encoder_ticks',
        '-p', 'request_timeout_sec:=1.0',
    ])
    receiver = None
    observer = None
    executor = SingleThreadedExecutor()
    try:
        receiver = EncoderHttpNode()
        observer = Node('encoder_test_observer')
        received = []
        observer.create_subscription(
            WheelEncoderTicks, 'test_encoder_ticks', received.append, 10,
        )
        executor.add_node(receiver)
        executor.add_node(observer)

        def spin_for(duration):
            deadline = monotonic() + duration
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)

        spin_for(1.5)
        assert len(received) >= 5
        assert all(list(message.ticks) == [10, -20, 30, -40] for message in received)
        assert all(message.header.stamp.sec > 0 for message in received)
        assert all(message.has_mcu_time and message.boot_id == '0123456789abcdef' for message in received)
        assert all(message.sample_time_us == 123456 for message in received)
        assert all(0 <= message.round_trip_time_sec <= 1.0 for message in received)
        assert requests and set(requests) == {('GET', '/api/status')}

        state['healthy'] = False
        spin_for(0.2)  # Drain any already queued successful samples.
        count = len(received)
        spin_for(0.3)
        assert len(received) == count

        state['healthy'] = True
        spin_for(0.5)
        assert len(received) > count
    finally:
        executor.shutdown()
        if receiver is not None:
            receiver.destroy_node()
        if observer is not None:
            observer.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
