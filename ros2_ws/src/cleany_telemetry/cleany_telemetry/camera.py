"""Bounded latest-frame preview cache, independent of ROS and external consumers."""

import io
import threading
import time
from dataclasses import dataclass

IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    received: float
    stamp: float | None
    width: int
    height: int


class Cameras:
    """최대 두 토픽의 관측 lease를 유지하며 오래된 프레임을 실시간 영상으로 표시하지 않습니다."""

    def __init__(self, limit=2, fps=5.0, clock=time.monotonic, lease_seconds=3.0, stale_seconds=2.0):
        if not 1 <= limit <= 4 or not 1 <= fps <= 10 or not 0 < stale_seconds < lease_seconds:
            raise ValueError("Invalid camera limits")
        self.limit, self.fps, self.clock = limit, fps, clock
        self.lease_seconds, self.stale_seconds = lease_seconds, stale_seconds
        self.topics = {}
        self.lock = threading.Lock()
        self.leases = {}
        self.frames = {}
        self.errors = {}
        self.last_encode = {}

    def discover(self, topics):
        """Invalidate leases when a publisher disappears or changes its message type."""
        selected = {name: types[0] for name, types in topics.items()
                    if len(types) == 1 and types[0] in IMAGE_TYPES}
        with self.lock:
            for name in list(self.leases):
                if self.topics.get(name) != selected.get(name):
                    self.leases[name] = 0
            self._expire(self.clock())
            self.topics = selected

    def catalog(self):
        with self.lock:
            return dict(self.topics)

    def _expire(self, now):
        for topic, until in list(self.leases.items()):
            if until <= now:
                self.leases.pop(topic, None)
                self.frames.pop(topic, None)
                self.errors.pop(topic, None)
                self.last_encode.pop(topic, None)

    def request(self, topic):
        with self.lock:
            now = self.clock()
            self._expire(now)
            if topic not in self.leases and len(self.leases) >= self.limit:
                raise ValueError("카메라 구독 한도 초과")
            self.leases[topic] = now + self.lease_seconds
            frame = self.frames.get(topic)
            return frame, self.errors.get(topic), now

    def selected(self):
        with self.lock:
            self._expire(self.clock())
            return set(self.leases)

    def receive(self, topic, message):
        with self.lock:
            now = self.clock()
            self._expire(now)
            if (
                topic not in self.leases
                or now - self.last_encode.get(topic, -float("inf")) < 1 / self.fps
            ):
                return
            self.last_encode[topic] = now
        try:
            jpeg, width, height = encode_image(message)
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            frame = Frame(
                jpeg,
                now,
                stamp.sec + stamp.nanosec / 1e9 if stamp else None,
                width,
                height,
            )
            error = None
        except (ValueError, OSError, TypeError, AttributeError) as exc:
            frame, error = None, str(exc)
        with self.lock:
            if topic in self.leases:
                self.frames[topic] = frame
                self.errors[topic] = error


def encode_image(message):
    """메모리 할당 전에 디코딩 크기를 제한하고 ROS의 행 간격과 채널 순서를 따릅니다."""
    from PIL import Image

    if len(message.data) > 16 * 1024 * 1024:
        raise ValueError("영상 크기 제한 초과 (16 MiB)")
    if hasattr(message, "encoding"):
        modes = {
            "rgb8": ("RGB", "RGB", 3),
            "bgr8": ("RGB", "BGR", 3),
            "rgba8": ("RGBA", "RGBA", 4),
            "bgra8": ("RGBA", "BGRA", 4),
            "mono8": ("L", "L", 1),
        }
        if message.encoding not in modes:
            raise ValueError(f"지원하지 않는 인코딩: {message.encoding}")
        mode, raw, channels = modes[message.encoding]
        width, height, step = message.width, message.height, message.step
        if width < 1 or height < 1 or width * height > 2097152:
            raise ValueError("영상 해상도 제한 초과 (2 MP)")
        if step < width * channels or len(message.data) != height * step:
            raise ValueError("영상 행 크기 불일치")
        image = Image.frombytes(
            mode, (width, height), bytes(message.data), "raw", raw, step
        )
    else:
        if "compressedDepth" in message.format:
            raise ValueError("Depth 영상 미지원")
        try:
            image = Image.open(io.BytesIO(bytes(message.data)))
        except Image.DecompressionBombError as exc:
            raise ValueError("영상 해상도 제한 초과") from exc
        if image.format not in {"JPEG", "PNG"}:
            raise ValueError("JPEG / PNG만 지원")
        if image.width * image.height > 2097152:
            raise ValueError("영상 해상도 제한 초과 (2 MP)")
    image.load()
    if not hasattr(message, "encoding") and image.format == "JPEG" and image.width <= 1280 and image.height <= 720 and len(message.data) <= 2 * 1024 * 1024:
        return bytes(message.data), image.width, image.height
    image.thumbnail((1280, 720))
    output = io.BytesIO()
    image.convert("RGB").save(output, format="JPEG", quality=75)
    return output.getvalue(), image.width, image.height
