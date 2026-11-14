"""Read-only preview HTTP contract and bounded consumer proxy."""
from urllib.parse import urlsplit

from aiohttp import ClientError, ClientSession, ClientTimeout, web

from .camera import Cameras


def loopback_origin(value: str) -> str:
    url = urlsplit(value)
    if (url.scheme != 'http' or url.hostname not in {'127.0.0.1', 'localhost', '::1'}
            or url.username or url.password or url.path not in {'', '/'} or url.query or url.fragment):
        raise ValueError('camera_preview_url must be a loopback HTTP origin')
    return value.rstrip('/')


def create_app(cameras: Cameras) -> web.Application:
    app = web.Application()

    async def catalog(request):
        return web.json_response({'topics': cameras.catalog()})

    async def frame(request):
        topic = request.query.get('topic', '')
        headers = {'Cache-Control': 'no-store'}
        if topic not in cameras.catalog():
            return web.json_response({'state': 'unknown', 'message': '카메라 토픽을 선택하세요'}, status=400, headers=headers)
        try:
            image, error, now = cameras.request(topic)
        except ValueError as exc:
            return web.json_response({'state': 'busy', 'message': str(exc)}, status=429, headers=headers)
        if error:
            return web.json_response({'state': 'error', 'message': error}, status=422, headers=headers)
        if image is None or now - image.received > cameras.stale_seconds:
            return web.json_response({'state': 'stale' if image else 'waiting'}, status=202, headers=headers)
        headers.update({'X-Frame-Age': str(now-image.received), 'X-Image-Width': str(image.width),
                        'X-Image-Height': str(image.height), 'X-ROS-Stamp': str(image.stamp or 0)})
        return web.Response(body=image.jpeg, content_type='image/jpeg', headers=headers)

    app.router.add_get('/api/cameras', catalog)
    app.router.add_get('/api/camera', frame)
    return app


async def proxy_frame(session: ClientSession, origin: str, topic: str) -> web.Response:
    """Forward only the preview contract, never arbitrary URLs or cached fallback frames."""
    try:
        async with session.get(origin + '/api/camera', params={'topic': topic},
                               allow_redirects=False, timeout=ClientTimeout(total=2)) as upstream:
            if upstream.status not in {200, 202, 400, 422, 429}:
                raise ValueError('Preview unavailable')
            body = bytearray()
            async for chunk in upstream.content.iter_chunked(65536):
                body.extend(chunk)
                if len(body) > 2 * 1024 * 1024:
                    raise ValueError('Preview too large')
            headers = {key: upstream.headers[key] for key in
                       ('Content-Type', 'X-Frame-Age', 'X-Image-Width', 'X-Image-Height', 'X-ROS-Stamp')
                       if key in upstream.headers}
            headers['Cache-Control'] = 'no-store'
            return web.Response(body=bytes(body), status=upstream.status, headers=headers)
    except (ClientError, TimeoutError, ValueError):
        return web.json_response({'state': 'unavailable', 'message': '카메라 미리보기 서버 연결 대기'},
                                 status=503, headers={'Cache-Control': 'no-store'})
