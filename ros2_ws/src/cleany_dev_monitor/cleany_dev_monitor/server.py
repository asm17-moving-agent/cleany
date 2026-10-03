"""Local HTTP/WebSocket reader. No ROS publishers, clients or control endpoints."""

import asyncio
import contextlib
import json
import math
import sqlite3
import tempfile
from pathlib import Path
from urllib.parse import urlsplit
from uuid import uuid4

from aiohttp import WSMsgType, web

from .camera import IMAGE_TYPES
from .core import Recorder


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
                graph = next(
                    (
                        row["data"]
                        for row in hub.snapshot()["rows"]
                        if row["kind"] == "graph"
                    ),
                    {},
                )
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
        graph = next(
            (row["data"] for row in hub.snapshot()["rows"] if row["kind"] == "graph"),
            {},
        )
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
        await runner.setup()
        try:
            await web.TCPSite(runner, config["host"], config["port"]).start()
            while not stop.is_set():
                try:
                    recorder.append(hub.snapshot(recorder.last))
                except (OSError, sqlite3.Error, ValueError) as exc:
                    recorder.reason = f"Recording stopped: {exc}"
                await asyncio.sleep(0.1)
        finally:
            await runner.cleanup()
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
        print(f"Dev monitor HTTP worker failed: {exc}", flush=True)
