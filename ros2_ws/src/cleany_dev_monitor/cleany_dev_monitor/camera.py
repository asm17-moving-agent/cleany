"""On-demand camera preview cache, independent of ROS and durable recordings."""

import io
import threading
import time
from dataclasses import dataclass

IMAGE_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}


def preview_supported(topic: str) -> bool:
    """Known metric-depth transports are not displayable RGB previews."""
    return not (
        "/compressedDepth" in topic
        or any(topic == base or topic.startswith(base + "/")
               for base in ("/camera/depth/image_raw", "/camera/raw_depth/image_raw"))
    )


def preview_source(topic: str, topics: list[dict]) -> str:
    """Prefer source JPEG previews without enabling a camera or changing raw RGB-D."""
    sources = {
        "/camera/color/image_raw": "/camera/color/image_raw/compressed",
        **{f"/{arm}_wrist_camera/image_raw": f"/{arm}_wrist_camera/preview/compressed"
           for arm in ("left", "right")},
    }
    preview = sources.get(topic)
    if preview and any(item["name"] == preview and item.get("types") == ["sensor_msgs/msg/CompressedImage"]
                       for item in topics):
        return preview
    return topic


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    received: float
    stamp: float | None
    width: int
    height: int


class Cameras:
    """At most two leased topics; stale frames never masquerade as live images."""

    def __init__(self, limit=2, fps=5.0, clock=time.monotonic):
        self.limit, self.fps, self.clock = limit, fps, clock
        self.lock = threading.Lock()
        self.leases = {}
        self.frames = {}
        self.errors = {}
        self.last_encode = {}

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
            self.leases[topic] = now + 3
            frame = self.frames.get(topic)
            return frame, self.errors.get(topic), now

    def selected(self):
        with self.lock:
            self._expire(self.clock())
            return set(self.leases)

    def receive(self, topic, message):
        # Native JPEG previews need no pixel encoding. Keep the freshest frame,
        # including slightly early 5 Hz arrivals, instead of halving their rate.
        passthrough = "jpeg" in getattr(message, "format", "").lower()
        with self.lock:
            now = self.clock()
            self._expire(now)
            if (
                topic not in self.leases
                or (not passthrough
                    and now - self.last_encode.get(topic, -float("inf")) < 1 / self.fps)
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
    """Bound decoding before allocation; honor ROS row stride and channel order."""
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
        if image.format == "JPEG" and image.width <= 1280 and image.height <= 720:
            # Validate the container without decoding/re-encoding JPEG pixels.
            width, height = image.size
            image.verify()
            return bytes(message.data), width, height
    image.thumbnail((1280, 720))
    output = io.BytesIO()
    image.convert("RGB").save(output, format="JPEG", quality=75)
    return output.getvalue(), image.width, image.height
