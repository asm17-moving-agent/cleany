import json
import base64
import hashlib
import socket
import struct
import threading
import time

from cleany_telemetry.relay import PoseCache, PoseRelay


def test_cache_rejects_nonfinite_and_clock_reset():
    cache = PoseCache(1.0)
    assert not cache.update(float("nan"), 2, 0, 1)
    assert cache.update(1, 2, 0, 10)
    assert not cache.update(3, 4, 0.1, 2)
    assert cache.fresh(0.1) is None
    assert cache.update(5, 6, 0.2, 3)
    assert not cache.update(7, 8, 0.3, 3)


def test_expiry_and_payload_shape():
    cache = PoseCache(1.0)
    cache.update(1.25, -2.5, 4.0, 1)
    pose = cache.fresh(4.5)
    assert pose is not None
    assert json.loads(json.dumps({"x": pose.x, "y": pose.y})) == {"x": 1.25, "y": -2.5}
    assert cache.fresh(5.01) is None


def test_worker_sends_latest_and_stops_after_input_expires():
    class FakeTransport:
        def __init__(self):
            self.messages = []
            self.ready = threading.Event()

        def send(self, message):
            self.messages.append(message)
            self.ready.set()

        def close(self):
            pass

    transport = FakeTransport()
    relay = PoseRelay(
        "ws://test",
        rate=50,
        input_timeout=0.05,
        transport_factory=lambda _: transport,
    )
    relay.update_pose(1.0, 2.0, 10.0)
    relay.start()
    assert transport.ready.wait(1.0)
    time.sleep(0.08)
    count = len(transport.messages)
    time.sleep(0.06)
    relay.stop()
    assert count > 0
    assert len(transport.messages) == count
    assert json.loads(transport.messages[0]) == {"x": 1.0, "y": 2.0}


def test_relay_validates_url_and_finite_timing():
    for kwargs in (
        {"url": "http://localhost"},
        {"url": "ws://localhost", "rate": float("nan")},
        {"url": "ws://localhost", "input_timeout": float("inf")},
        {"url": "ws://localhost", "reconnect_initial": float("nan")},
    ):
        try:
            PoseRelay(**kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid relay configuration was accepted")


def test_local_websocket_ping_and_latest_rate():
    received = []
    first_message = threading.Event()
    pongs = []
    connections = []
    ready = threading.Event()
    stop = threading.Event()
    state = {}

    def server_thread():
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(2)
        state["url"] = "ws://127.0.0.1:%d" % listener.getsockname()[1]
        ready.set()
        client, _ = listener.accept()
        connections.append(client)
        client.settimeout(0.02)
        request = b""
        while b"\r\n\r\n" not in request:
            request += client.recv(4096)
        key = next(
            line.split(b": ", 1)[1]
            for line in request.split(b"\r\n")
            if line.lower().startswith(b"sec-websocket-key:")
        )
        accept = base64.b64encode(
            hashlib.sha1(key.strip() + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest()
        )
        client.sendall(
            b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
            b"Connection: Upgrade\r\nSec-WebSocket-Accept: " + accept + b"\r\n\r\n"
        )
        last_ping = time.monotonic()
        buffer = b""
        while not stop.is_set():
            if time.monotonic() - last_ping >= 0.05:
                client.sendall(b"\x89\x04ping")
                last_ping = time.monotonic()
            try:
                buffer += client.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            while len(buffer) >= 2:
                first, second = buffer[0], buffer[1]
                length, offset = second & 127, 2
                if length == 126:
                    if len(buffer) < 4:
                        break
                    length, offset = struct.unpack("!H", buffer[2:4])[0], 4
                if len(buffer) < offset + 4 + length:
                    break
                mask = buffer[offset : offset + 4]
                payload = bytes(
                    value ^ mask[index % 4]
                    for index, value in enumerate(buffer[offset + 4 : offset + 4 + length])
                )
                buffer = buffer[offset + 4 + length :]
                if first & 15 == 10:
                    pongs.append(payload)
                elif first & 15 == 1:
                    received.append((time.monotonic(), json.loads(payload)))
                    first_message.set()
        client.close()
        listener.close()

    thread = threading.Thread(target=server_thread, daemon=True)
    thread.start()
    assert ready.wait(2)
    relay = PoseRelay(
        state["url"], input_timeout=0.8, reconnect_initial=0.05, reconnect_max=0.1
    )
    relay.update_pose(1, 2, 1)
    relay.start()
    assert first_message.wait(2)
    relay.update_pose(3, 4, 2)
    time.sleep(0.65)  # exercises several server ping/pong intervals
    relay.stop()
    stop.set()
    thread.join(timeout=2)
    assert len(received) >= 2
    assert len(pongs) >= 2
    assert len(connections) == 1
    assert all(item[1].keys() == {"x", "y"} for item in received)
    assert received[-1][1] == {"x": 3, "y": 4}
    intervals = [b[0] - a[0] for a, b in zip(received, received[1:])]
    assert all(0.17 <= interval <= 0.35 for interval in intervals)


def test_send_failure_reconnects_with_backoff():
    class Flaky:
        def __init__(self, fail):
            self.fail = fail
            self.messages = []

        def send(self, message):
            if self.fail:
                self.fail = False
                raise ConnectionError("closed")
            self.messages.append(message)

        def close(self):
            pass

    transports = []

    def connect(_url):
        transport = Flaky(not transports)
        transports.append(transport)
        return transport

    relay = PoseRelay(
        "ws://test", rate=5, input_timeout=1, reconnect_initial=0.05,
        reconnect_max=0.1, transport_factory=connect,
    )
    relay.update_pose(1, 2, 1)
    relay.start()
    deadline = time.monotonic() + 1
    while len(transports) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    relay.stop()
    assert len(transports) >= 2
    assert transports[1].messages
