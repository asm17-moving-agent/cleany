from collections import deque
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread
from time import sleep

import pytest

from cleany_base_odometry.encoder_http import (
    EncoderHttpClient, EncoderReadError, parse_encoder_counts, parse_encoder_sample,
)
import json


def test_timestamped_firmware_status_and_legacy_compatibility():
    payload = {'encoders': [1, 2, 3, 4], 'protocol_version': 1,
               'boot_id': '0123456789abcdef', 'sample_seq': 5, 'sample_time_us': 123456789}
    result = parse_encoder_sample(json.dumps(payload).encode(), 0.03)
    assert result.boot_id == payload['boot_id']
    assert result.sample_seq == 5 and result.sample_time_us == 123456789
    assert result.round_trip_time_sec == 0.03
    assert parse_encoder_sample(b'{"encoders":[0,0,0,0]}').boot_id == ''
    for key, value in [('boot_id', 'bad'), ('protocol_version', True), ('sample_seq', -1), ('sample_time_us', 2**63)]:
        invalid = dict(payload, **{key: value})
        with pytest.raises(EncoderReadError):
            parse_encoder_sample(json.dumps(invalid).encode())
    del payload['sample_time_us']
    with pytest.raises(EncoderReadError):
        parse_encoder_sample(json.dumps(payload).encode())


@contextmanager
def status_server(responses):
    remaining = deque(responses)
    requests = []

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def do_GET(self):
            requests.append((self.command, self.path))
            status, body, delay = remaining.popleft()
            sleep(delay)
            self.send_response(status)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port, requests
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_restored_pid_status_preserves_raw_encoder_contract():
    # Firmware motor diagnostics use logical velocities; encoders remain raw
    # PCB FL/FR/RR/RL counts for the existing Jetson sign/mapping conversion.
    payload = {
        'encoders': [3172, -3172, -6344, 9516], 'protocol_version': 1,
        'boot_id': 'fedcba9876543210', 'sample_seq': 42, 'sample_time_us': 9876543,
        'target': [100] * 4, 'applied': [30, 31, 32, 33],
        'target_rad_s': [10.0] * 4, 'commanded_rad_s': [5.0] * 4,
        'omega_rad_s': [4.9, 5.1, 4.8, 5.2],
        'reverse_waiting': [False] * 4, 'calibration_move_active': [False] * 4,
    }
    with status_server([(200, json.dumps(payload).encode(), 0)]) as (port, requests):
        client = EncoderHttpClient('127.0.0.1', port, timeout_sec=1.0)
        try:
            sample = client.read()
            assert sample.ticks == (3172, -3172, -6344, 9516)
            assert sample.boot_id == payload['boot_id']
            assert sample.sample_seq == 42 and sample.sample_time_us == 9876543
            assert requests == [('GET', '/api/status')]
        finally:
            client.close()


@pytest.mark.parametrize('payload', [
    b'{}', b'[]', b'{', b'\xff', b'{"encoders":[1,2,3]}',
    b'{"encoders":[1,2,3,4,5]}', b'{"encoders":[true,2,3,4]}',
    b'{"encoders":[1.0,2,3,4]}', b'{"encoders":["1",2,3,4]}',
    b'{"encoders":[2147483648,2,3,4]}',
    b'{"encoders":[-2147483649,2,3,4]}', b' ' * 2049,
])
def test_rejects_invalid_counts(payload):
    with pytest.raises(EncoderReadError):
        parse_encoder_counts(payload)


def test_preserves_signed_counts_motor_order_and_rollover_values():
    assert parse_encoder_counts(
        b'{"encoders":[-2147483648,2147483647,-123,456]}'
    ) == (-2147483648, 2147483647, -123, 456)


def test_reads_repeated_samples_and_recovers_after_error():
    responses = [
        (200, b'{"encoders":[1,-2,3,-4]}', 0),
        (200, b'{"encoders":[11,-22,33,-44]}', 0),
        (503, b'unavailable', 0),
        (200, b'{"encoders":[0,0,0,0]}', 0),
    ]
    with status_server(responses) as (port, requests):
        client = EncoderHttpClient('127.0.0.1', port, timeout_sec=1.0)
        try:
            first = client.read()
            assert first.ticks == (1, -2, 3, -4)
            assert 0 <= first.round_trip_time_sec < 1.0
            assert client.read().ticks == (11, -22, 33, -44)
            with pytest.raises(EncoderReadError, match='503'):
                client.read()
            # Raw telemetry preserves a possible MCU reset; it does not invent
            # continuous wheel positions or attempt to integrate this jump.
            assert client.read().ticks == (0, 0, 0, 0)
            assert requests == [('GET', '/api/status')] * 4
        finally:
            client.close()


@pytest.mark.parametrize('status, body', [
    (302, b'redirect'), (200, b'broken JSON'), (200, b'x' * 2049),
])
def test_bad_http_response_does_not_poison_next_read(status, body):
    with status_server([
        (status, body, 0), (200, b'{"encoders":[5,6,7,8]}', 0),
    ]) as (port, requests):
        client = EncoderHttpClient('127.0.0.1', port, timeout_sec=1.0)
        try:
            with pytest.raises(EncoderReadError):
                client.read()
            assert client.read().ticks == (5, 6, 7, 8)
            assert len(requests) == 2
        finally:
            client.close()


def test_timeout_does_not_emit_cached_or_zero_counts():
    with status_server([
        (200, b'{"encoders":[1,2,3,4]}', 0.2),
        (200, b'{"encoders":[8,7,6,5]}', 0),
    ]) as (port, _):
        client = EncoderHttpClient('127.0.0.1', port, timeout_sec=0.05)
        try:
            with pytest.raises(EncoderReadError):
                client.read()
            assert client.read().ticks == (8, 7, 6, 5)
        finally:
            client.close()
