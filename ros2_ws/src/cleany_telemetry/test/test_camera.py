import asyncio
import io
from types import SimpleNamespace as NS

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_telemetry.camera import Cameras, encode_image, preview_source, preview_supported
from cleany_telemetry.camera_http import create_app
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


def test_preview_lease_stale_errors_and_catalog():
    async def scenario():
        now = [0.0]
        cameras = Cameras(limit=1, clock=lambda: now[0])
        cameras.discover({'/a':['sensor_msgs/msg/Image'], '/b':['sensor_msgs/msg/Image']})
        async with TestServer(create_app(cameras)) as server, ClientSession() as client:
            async def get(topic):
                return await client.get(server.make_url('/api/camera'), params={'topic':topic})
            assert (await get('/cmd_vel')).status == 400
            assert (await get('/a')).status == 202
            assert (await get('/b')).status == 429
            cameras.receive('/a', raw())
            response = await get('/a')
            assert response.status == 200
            assert response.headers['Cache-Control'] == 'no-store'
            assert Image.open(io.BytesIO(await response.read())).size == (1,2)
            now[0] = 2.1
            response = await get('/a')
            assert response.status == 202 and (await response.json())['state'] == 'stale'
            cameras.receive('/a', raw(encoding='16UC1'))
            assert (await get('/a')).status == 422
            now[0] = 6
            assert not cameras.selected() and not cameras.frames
            assert (await get('/b')).status == 202
            cameras.discover({})
            assert not cameras.selected()
            assert (await get('/b')).status == 400
    asyncio.run(scenario())


def test_small_jpeg_is_validated_without_reencoding_and_large_image_is_resized():
    for size in [(640,480),(1600,900)]:
        stream = io.BytesIO()
        Image.new('RGB',size,'blue').save(stream,format='JPEG')
        encoded = stream.getvalue()
        output,width,height = encode_image(NS(format='jpeg',data=encoded))
        if size == (640,480):
            assert output == encoded
        else:
            assert (width,height)==(1280,720)
    with pytest.raises(OSError):
        encode_image(NS(format='jpeg',data=encoded[:len(encoded)//2]))


def test_encoding_throttle_and_invalid_limits():
    now=[0.0]
    cameras=Cameras(clock=lambda:now[0])
    cameras.request('/a')
    cameras.receive('/a',raw())
    first=cameras.frames['/a']
    now[0]=.1
    cameras.receive('/a',raw())
    assert cameras.frames['/a'] is first
    for kwargs in ({'limit':0},{'fps':float('nan')},{'lease_seconds':1,'stale_seconds':2}):
        with pytest.raises(ValueError): Cameras(**kwargs)


def test_jpeg_preview_preserves_encoded_bytes_without_second_lossy_encode(monkeypatch):
    encoded = io.BytesIO()
    Image.new("RGB", (640, 480), "green").save(encoded, format="JPEG", quality=72)
    payload = encoded.getvalue()
    monkeypatch.setattr(Image.Image, "save", lambda *_args, **_kwargs: pytest.fail("JPEG was re-encoded"))
    result, width, height = encode_image(NS(format="bgr8; jpeg compressed bgr8", data=payload))
    assert result == payload and (width, height) == (640, 480)


def test_native_jpeg_cache_keeps_latest_frame_between_encode_intervals():
    now = [0.0]
    cache = Cameras(clock=lambda: now[0])
    cache.request("/camera/preview/compressed")
    for timestamp, color in ((0.0, "red"), (.19, "blue")):
        now[0] = timestamp
        encoded = io.BytesIO()
        Image.new("RGB", (8, 8), color).save(encoded, format="JPEG")
        cache.receive("/camera/preview/compressed", NS(format="jpeg", data=encoded.getvalue()))
    frame = cache.request("/camera/preview/compressed")[0]
    assert frame.received == .19 and frame.jpeg == encoded.getvalue()


def test_camera_selects_low_rate_preview_and_rejects_metric_depth():
    raw_topic = "/left_wrist_camera/image_raw"
    preview = "/left_wrist_camera/preview/compressed"
    assert preview_source(raw_topic, []) == raw_topic
    assert preview_source(raw_topic, [{"name": preview, "types": ["sensor_msgs/msg/CompressedImage"]}]) == preview
    assert preview_source(raw_topic, [{"name": preview, "types": ["std_msgs/msg/String"]}]) == raw_topic
    assert preview_source("/camera/color/image_raw", [
        {"name": "/camera/color/image_raw/compressed", "types": ["sensor_msgs/msg/CompressedImage"]}
    ]) == "/camera/color/image_raw/compressed"
    assert preview_supported(raw_topic)
    assert preview_supported("/camera/color/image_raw")
    for topic in ("/camera/depth/image_raw", "/camera/raw_depth/image_raw/compressed",
                  "/other/image_raw/compressedDepth"):
        assert not preview_supported(topic)


def test_provider_resolves_preview_and_rejects_depth_without_leasing():
    async def scenario():
        raw_topic = '/left_wrist_camera/image_raw'
        preview = '/left_wrist_camera/preview/compressed'
        depth = '/camera/depth/image_raw/compressed'
        cameras = Cameras()
        cameras.discover({raw_topic: ['sensor_msgs/msg/Image'],
                          preview: ['sensor_msgs/msg/CompressedImage'],
                          depth: ['sensor_msgs/msg/CompressedImage']})
        async with TestServer(create_app(cameras)) as server, ClientSession() as client:
            response = await client.get(server.make_url('/api/camera'), params={'topic': raw_topic})
            assert response.status == 202
            assert response.headers['X-Camera-Topic'] == preview
            assert cameras.selected() == {preview}
            response = await client.get(server.make_url('/api/camera'), params={'topic': depth})
            assert response.status == 400
            assert cameras.selected() == {preview}
    asyncio.run(scenario())
