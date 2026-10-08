"""Local HTTP/WebSocket reader. No ROS publishers, clients or control endpoints."""

import asyncio
import contextlib
import json
import math
import os
import zlib
import sqlite3
import tempfile
from pathlib import Path
from urllib.parse import urlsplit
from uuid import uuid4

from aiohttp import WSMsgType, web

from .camera import IMAGE_TYPES
from .core import Recorder
from .recording import validate, RAW


@web.middleware
async def local_origin(request, handler):
    origin = request.headers.get("Origin")
    if origin and urlsplit(origin).netloc != request.host:
        raise web.HTTPForbidden(text="Same-origin requests only")
    try:
        return await handler(request)
    except (ValueError, FileNotFoundError, json.JSONDecodeError) as exc:
        raise web.HTTPBadRequest(text=str(exc)) from exc


def create_app(hub, config, recorder, assets):
    app = web.Application(middlewares=[local_origin], client_max_size=16384)
    sockets = set()
    controls = hub.recording
    recorder.controls = controls
    settings_file = recorder.directory / 'recording-settings.json'
    if settings_file.exists():
        try:
            controls.configure(json.loads(settings_file.read_text()))
            recorder.session_limit = controls.settings["limit_mib"]*1024**2
        except (ValueError, OSError, TypeError):
            pass
    hub.put('recording_policy', 'settings', controls.view()['settings'], critical=True)

    def apply_settings(settings, *, new_session=False):
        recorder.append(hub.snapshot(recorder.last))
        recorder.append_raw()
        temporary = settings_file.with_suffix('.tmp')
        with temporary.open('w') as handle:
            json.dump(settings, handle)
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(settings_file)
        controls.configure(settings)
        recorder.session_limit = settings['limit_mib']*1024**2
        boundary = hub.snapshot()['sequence']
        if new_session:
            directory, total_limit = recorder.directory, recorder.total_limit
            recorder.close()
            recorder.__init__(directory, settings['limit_mib']*1024**2, total_limit, controls=controls)
            controls.writer_error = recorder.reason
        hub.put('recording_policy', 'settings', settings, critical=True)
        snapshot = hub.snapshot(recorder.last)
        if new_session:
            snapshot['rows'] = [r for r in snapshot['rows'] if r['kind'] not in ('event','log','result','recording_policy') or r['seq']>boundary]
        recorder.append(snapshot)

    async def recording_settings(request):
        if request.method == 'PUT':
            apply_settings(validate(await request.json()))
        return web.json_response(dict(controls.view(recorder.reason), session_id=recorder.session_id))

    async def new_recording(request):
        settings = validate(await request.json())
        settings['active'] = True
        apply_settings(settings, new_session=True)
        return web.json_response(dict(controls.view(recorder.reason), session_id=recorder.session_id))

    async def raw_message(request):
        at = float(request.query.get('at', '0'))
        key = request.query.get('category', '')
        if not math.isfinite(at) or key not in RAW:
            raise ValueError('Invalid raw recording request')
        with sqlite3.connect(f"file:{recorder.resolve(request.match_info['session'])}?mode=ro", uri=True) as db:
            if not db.execute("SELECT name FROM sqlite_master WHERE name='raw_messages'").fetchone():
                raise web.HTTPNotFound(text='원본 녹화 없음')
            row = db.execute('SELECT payload,type,topic FROM raw_messages WHERE category=? AND received<=? ORDER BY received DESC,id DESC LIMIT 1', (key,at)).fetchone()
        if not row:
            raise web.HTTPNotFound(text='이 시점 이전에 원본 녹화가 없습니다.')
        return web.Response(body=zlib.decompress(row[0]), content_type='application/octet-stream',
                            headers={'Content-Disposition': f'attachment; filename="{key}.cdr"', 'X-ROS-Message-Type':row[1], 'X-ROS-Topic':row[2]})

    async def snapshot(request):
        return web.json_response(hub.snapshot())

    async def sessions(request):
        return web.json_response(
            {
                "sessions": recorder.sessions(),
                "active": recorder.session_id,
                "reason": recorder.reason,
            }
        )

    async def navigation_runs(request):
        return web.json_response({"runs": recorder.navigation_runs()})

    async def replay(request):
        at = float(request.query.get("at", "0"))
        if not math.isfinite(at):
            raise ValueError("Invalid replay time")
        return web.json_response(recorder.replay(request.match_info["session"], at))

    async def export(request):
        source = recorder.resolve(request.match_info["session"])
        # SQLite backup includes committed data consistently even for active recordings.
        with tempfile.TemporaryDirectory(prefix="cleany-monitor-export-") as directory:
            target = Path(directory) / "recording.sqlite3"
            with (
                sqlite3.connect(f"file:{source}?mode=ro", uri=True) as src,
                sqlite3.connect(target) as dst,
            ):
                src.backup(dst)
            response = web.StreamResponse(
                headers={
                    "Content-Type": "application/vnd.sqlite3",
                    "Content-Disposition": f'attachment; filename="{source.name}"',
                }
            )
            await response.prepare(request)
            with target.open("rb") as stream:
                while chunk := stream.read(65536):
                    await response.write(chunk)
            await response.write_eof()
            return response

    async def websocket(request):
        if len(sockets) >= 8:
            raise web.HTTPServiceUnavailable(text="Maximum 8 monitor clients")
        ws = web.WebSocketResponse(heartbeat=15, max_msg_size=16384)
        await ws.prepare(request)
        sockets.add(ws)
        client = str(uuid4())
        hub.select(client, [])

        async def send():
            since = 0
            first = True
            while not ws.closed:
                payload = hub.snapshot(since)
                if not first:
                    payload["rows"] = [
                        row for row in payload["rows"] if row["seq"] > since
                    ]
                payload["initial"] = first
                payload["recording"] = {
                    "id": recorder.session_id,
                    "reason": recorder.reason,
                }
                # Slow clients disconnect and obtain a fresh snapshot + explicit gap on reconnect.
                await asyncio.wait_for(ws.send_json(payload), timeout=2)
                since, first = payload["sequence"], False
                await asyncio.sleep(0.1)

        sender = asyncio.create_task(send())
        sender.add_done_callback(
            lambda future: (
                asyncio.create_task(ws.close()) if not future.cancelled() else None
            )
        )
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                data = json.loads(message.data)
                if data.get("op") != "select":
                    await ws.send_json(
                        {"error": "Only read-only topic selection is supported"}
                    )
                    continue
                topics = data.get("topics")
                graph = hub.latest_data("graph", "ros")
                available = {item["name"] for item in graph.get("topics", [])}
                if (
                    not isinstance(topics, list)
                    or len(topics) > config["max_selected_topics"]
                    or any(
                        not isinstance(topic, str) or topic not in available
                        for topic in topics
                    )
                ):
                    await ws.send_json(
                        {"error": "Select known topics within the configured limit"}
                    )
                    continue
                if not hub.select(client, topics, limit=config["max_selected_topics"]):
                    await ws.send_json({"error": "Shared subscription limit reached"})
                    continue
                await ws.send_json({"selection": topics})
        finally:
            hub.release(client)
            sockets.discard(ws)
            sender.cancel()
            with contextlib.suppress(
                asyncio.CancelledError, ConnectionError, asyncio.TimeoutError
            ):
                await sender
        return ws

    async def camera(request):
        topic = request.query.get("topic", "")
        graph = hub.latest_data("graph", "ros")
        item = next(
            (item for item in graph.get("topics", []) if item["name"] == topic), None
        )
        if (
            not item
            or len(item.get("types", [])) != 1
            or item["types"][0] not in IMAGE_TYPES
        ):
            raise web.HTTPBadRequest(text="카메라 토픽을 선택하세요")
        try:
            frame, error, now = hub.cameras.request(topic)
        except ValueError as exc:
            raise web.HTTPTooManyRequests(text=str(exc)) from exc
        headers = {"Cache-Control": "no-store"}
        if error:
            return web.json_response(
                {"state": "error", "message": error}, status=422, headers=headers
            )
        if not frame or now - frame.received > 2:
            return web.json_response(
                {"state": "stale" if frame else "waiting"}, status=202, headers=headers
            )
        headers.update(
            {
                "X-Frame-Age": str(now - frame.received),
                "X-Image-Width": str(frame.width),
                "X-Image-Height": str(frame.height),
                "X-ROS-Stamp": str(frame.stamp or 0),
            }
        )
        return web.Response(body=frame.jpeg, content_type="image/jpeg", headers=headers)

    async def index(request):
        target = assets / "index.html"
        if not target.exists():
            raise web.HTTPServiceUnavailable(
                text="Frontend not built. Run make build-dev-monitor-web."
            )
        return web.FileResponse(target)

    async def cleanup(app):
        for ws in list(sockets):
            await ws.close()

    app.router.add_get("/api/camera", camera)
    app.router.add_get("/api/snapshot", snapshot)
    app.router.add_get("/api/recordings", sessions)
    app.router.add_get("/api/recording/settings", recording_settings)
    app.router.add_put("/api/recording/settings", recording_settings)
    app.router.add_post("/api/recording/session", new_recording)
    app.router.add_get("/api/recordings/{session}/raw", raw_message)
    app.router.add_get("/api/navigation/runs", navigation_runs)
    app.router.add_get("/api/recordings/{session}/replay", replay)
    app.router.add_get("/api/recordings/{session}/export", export)
    app.router.add_get("/ws", websocket)
    app.router.add_get("/", index)
    if (assets / "assets").is_dir():
        app.router.add_static("/assets", assets / "assets", follow_symlinks=True)
    app.on_cleanup.append(cleanup)
    return app


def serve(hub, config, stop):
    async def run():
        from ament_index_python.packages import get_package_share_directory

        assets = Path(get_package_share_directory("cleany_dev_monitor")) / "web"
        recorder = Recorder(
            config["state_directory"],
            config["session_limit_bytes"],
            config["total_limit_bytes"],
        )
        runner = web.AppRunner(create_app(hub, config, recorder, assets))
        previous_status = None
        await runner.setup()
        try:
            await web.TCPSite(runner, config["host"], config["port"]).start()
            while not stop.is_set():
                try:
                    recorder.append(hub.snapshot(recorder.last))
                    recorder.append_raw()
                    hub.recording.writer_error = recorder.reason
                    status = hub.recording.view(recorder.reason)
                    status = {key: {k:v for k,v in value.items() if k != 'received_at'} for key,value in status['status'].items()}
                    if status != previous_status:
                        hub.put('recording_status', 'writer', {'status':status, 'reason':recorder.reason})
                        previous_status = status
                except (OSError, sqlite3.Error, ValueError) as exc:
                    recorder.reason = f"Recording stopped: {exc}"
                    hub.recording.writer_error = recorder.reason
                await asyncio.sleep(0.1)
        finally:
            await runner.cleanup()
            recorder.append(hub.snapshot(recorder.last))
            recorder.append_raw()
            recorder.close()

    try:
        asyncio.run(run())
    except (OSError, RuntimeError, ValueError) as exc:
        hub.put(
            "event",
            "server_error",
            {"kind": "server_error", "message": str(exc)},
            critical=True,
        )
        from rclpy.logging import get_logger

        get_logger("cleany_dev_monitor").error(f"HTTP worker failed: {exc}")
