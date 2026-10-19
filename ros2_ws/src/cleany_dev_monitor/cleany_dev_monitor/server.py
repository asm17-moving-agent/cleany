"""Local HTTP/WebSocket reader. No ROS publishers, clients or control endpoints."""

import asyncio
import contextlib
import json
import math
import os
import sqlite3
import tempfile
import zipfile
from pathlib import Path
from urllib.parse import urlsplit
from uuid import uuid4

from aiohttp import WSMsgType, web

from .camera import IMAGE_TYPES
from .core import Recorder
from .recording import RAW, observation_policies, rate_profiles, validate
from .resources import ResourceWorker
from .sources import (
    SCHEMA,
    maximum_reference,
    raw_maximum_references,
    resolved,
    validate_sources,
)


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
    controls.source_config = dict(config)
    recorder.controls = controls
    settings_file = recorder.directory / 'recording-settings.json'
    if settings_file.exists():
        try:
            loaded = validate(json.loads(settings_file.read_text()))
            validate_sources({**resolved(config), **loaded['sources']})
            controls.configure(loaded)
            recorder.session_limit = controls.settings["limit_mib"]*1024**2
        except (ValueError, OSError, TypeError):
            pass
    hub.put('recording_policy', 'settings', controls.view()['settings'], critical=True)

    def apply_settings(settings, *, new_session=False):
        validate_sources({**resolved(config), **settings['sources']})
        with controls.transition():
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
                directory, total_limit, bag_factory = recorder.directory, recorder.total_limit, recorder.bag_factory
                recorder.close()
                recorder.__init__(directory, settings['limit_mib']*1024**2, total_limit, controls=controls, bag_factory=bag_factory)
                controls.writer_error = recorder.reason
            hub.put('recording_policy', 'settings', settings, critical=True)
            # A new session starts at this boundary. Eviction before it is not a loss
            # in the new recording; latest state still provides its initial context.
            snapshot = hub.snapshot(boundary if new_session else recorder.last)
            if new_session:
                snapshot['rows'] = [r for r in snapshot['rows'] if r['kind'] not in ('event','log','result','recording_policy') or r['seq']>boundary]
            recorder.append(snapshot)
            if not settings["active"]:
                recorder.close()

    def recording_view():
        view = controls.view(recorder.reason)
        current = hub.latest_data('config', 'monitor') or config
        effective = resolved(current)
        requested = validate_sources({**resolved(config), **view['settings']['sources']})
        view['settings']['sources'] = requested
        return dict(view, session_id=recorder.session_id, observation=observation_policies(current),
                    source_fields=SCHEMA, source_applied=effective, rate_profiles=rate_profiles(current.get('resource_interval_s', 1.)),
                    rate_maximums={**raw_maximum_references(), 'resources': maximum_reference(1 / current.get('resource_interval_s', 1.), 'config', '호스트 리소스 수집 간격 기준')},
                    source_pending=requested != effective, source_live=bool(current.get('read_only')))


    async def recording_settings(request):
        if request.method == 'PUT':
            settings = validate(await request.json())
            # Starting after a stop always creates a separate file, including older clients.
            start = settings["active"] and not controls.settings["active"]
            apply_settings(settings, new_session=start)
        return web.json_response(recording_view())

    async def new_recording(request):
        settings = validate(await request.json())
        settings['active'] = True
        apply_settings(settings, new_session=True)
        return web.json_response(recording_view())

    async def raw_message(request):
        at = float(request.query.get('at', '0'))
        key = request.query.get('category', '')
        if not math.isfinite(at) or key not in RAW:
            raise ValueError('Invalid raw recording request')
        try:
            data, message_type, topic = recorder.raw_message(request.match_info['session'], key, at)
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        return web.Response(body=data, content_type='application/octet-stream',
                            headers={'Content-Disposition': f'attachment; filename="{key}.cdr"', 'X-ROS-Message-Type':message_type, 'X-ROS-Topic':topic})

    async def export_bag(request):
        session = request.match_info['session']
        directory = recorder.resolve_bag(session)
        if session == recorder.session_id and recorder.bag is not None:
            raise web.HTTPConflict(text='rosbag2 내보내기는 녹화를 정지한 뒤 가능합니다.')
        if not recorder.bag_available(session):
            raise web.HTTPNotFound(text='완료된 rosbag2 기록이 없습니다.')
        # A closed bag includes native metadata and all storage segments. No CDR
        # conversion or payload duplication into the dashboard database occurs.
        with tempfile.TemporaryDirectory(prefix='cleany-bag-export-') as temporary:
            archive = Path(temporary) / 'rosbag2.zip'
            with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_STORED) as target:
                for path in sorted(directory.iterdir()):
                    if path.is_file() and (path.name == 'metadata.yaml' or path.suffix == '.db3'):
                        target.write(path, f'{session}/{path.name}')
            response = web.StreamResponse(headers={
                'Content-Type': 'application/zip',
                'Content-Disposition': f'attachment; filename="{session}-rosbag2.zip"',
            })
            await response.prepare(request)
            with archive.open('rb') as stream:
                while chunk := stream.read(65536):
                    await response.write(chunk)
            await response.write_eof()
            return response

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
            revision = hub.topic_revision
            while not ws.closed:
                payload = hub.snapshot(since)
                changed = revision != payload["topic_revision"]
                if changed:
                    payload = hub.snapshot(0)
                if not first and not changed:
                    payload["rows"] = [
                        row for row in payload["rows"] if row["seq"] > since
                    ]
                payload["initial"] = first or changed
                payload["sources_changed"] = changed
                revision = payload["topic_revision"]
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
    app.router.add_get("/api/recordings/{session}/bag", export_bag)
    app.router.add_get("/ws", websocket)
    app.router.add_get("/", index)
    if (assets / "assets").is_dir():
        app.router.add_static("/assets", assets / "assets", follow_symlinks=True)
    worker = ResourceWorker(hub, config)

    async def start_resources(app):
        worker.thread.start()

    async def stop_resources(app):
        worker.stop.set()
        await asyncio.to_thread(worker.thread.join)

    app.on_startup.append(start_resources)
    app.on_cleanup.append(stop_resources)
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
            try:
                await runner.cleanup()
                recorder.append(hub.snapshot(recorder.last))
                recorder.append_raw()
            finally:
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
