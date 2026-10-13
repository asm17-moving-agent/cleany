import asyncio
import io
from types import SimpleNamespace as NS

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_dev_monitor.camera import Cameras, encode_image
from cleany_dev_monitor.core import Hub, Recorder
from cleany_dev_monitor.server import create_app
from PIL import Image


def raw(**changes):
    data = {
        "width": 1,
        "height": 2,
        "step": 4,
        "encoding": "bgr8",
        "data": bytes([0, 0, 255, 0, 0, 0, 255, 0]),
    }
    return NS(**{**data, **changes})


def test_camera_decode_stride_channel_order_and_bounded_formats():
    jpeg, width, height = encode_image(raw())
    image = Image.open(io.BytesIO(jpeg))
    assert (width, height) == (1, 2)
    red, green, blue = image.getpixel((0, 1))
    assert red > 240 and green < 10 and blue < 10
    png = io.BytesIO()
    Image.new("RGB", (3, 2), "blue").save(png, format="PNG")
    assert encode_image(NS(format="rgb8; png compressed bgr8", data=png.getvalue()))[
        1:
    ] == (3, 2)
    for message in [
        raw(encoding="16UC1"),
        raw(step=2),
        raw(width=3000, height=3000),
        NS(format="jpeg", data=b"broken"),
        NS(format="16UC1; compressedDepth png", data=b""),
    ]:
        with pytest.raises((ValueError, OSError)):
            encode_image(message)


def test_camera_http_staleness_leases_and_no_recorded_frames(tmp_path):
    async def scenario():
        now = [0.0]
        hub = Hub()
        hub.cameras = Cameras(limit=1, clock=lambda: now[0])
        hub.put(
            "graph",
            "ros",
            {
                "topics": [
                    {"name": t, "types": ["sensor_msgs/msg/Image"]}
                    for t in ["/a", "/b"]
                ]
            },
        )
        recorder = Recorder(tmp_path)
        async with (
            TestServer(create_app(hub, {}, recorder, tmp_path)) as server,
            ClientSession() as client,
        ):

            async def get(topic):
                return await client.get(
                    server.make_url("/api/camera"), params={"topic": topic}
                )

            assert (await get("/cmd_vel")).status == 400
            assert (await get("/a")).status == 202
            assert (await get("/b")).status == 429
            hub.cameras.receive("/a", raw())
            response = await get("/a")
            assert response.status == 200
            assert response.headers["Cache-Control"] == "no-store"
            assert Image.open(io.BytesIO(await response.read())).size == (1, 2)
            now[0] = 2.1
            response = await get("/a")
            assert (
                response.status == 202 and (await response.json())["state"] == "stale"
            )
            hub.cameras.receive("/a", raw(encoding="16UC1"))
            assert (await get("/a")).status == 422  # old valid frame was cleared
            recorder.append(hub.snapshot())
            assert {
                r["kind"]
                for r in recorder.replay(recorder.session_id, float("inf"))["rows"]
            } == {"graph"}
            now[0] = 6
            assert hub.cameras.selected() == set()
            assert not hub.cameras.frames
            assert (await get("/b")).status == 202
        recorder.close()

    asyncio.run(scenario())
