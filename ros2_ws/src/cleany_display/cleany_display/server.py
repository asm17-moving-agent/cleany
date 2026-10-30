"""Serve the local display and read-only state snapshot."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from threading import Lock
from typing import Any


class StateStore:
    def __init__(self) -> None:
        self.lock = Lock()
        self.value: dict[str, Any] = {'mode': 'idle', 'connected': False}

    def set(self, value: dict[str, Any]) -> None:
        with self.lock:
            self.value = value

    def get(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.value)


def make_server(host: str, port: int, web_root: Path, store: StateStore) -> ThreadingHTTPServer:
    assets = {'/': ('index.html', 'text/html'), '/face.js': ('face.js', 'text/javascript'),
              '/face.css': ('face.css', 'text/css')}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = self.path.split('?', 1)[0]
            if path in ('/api/state', '/healthz'):
                body = json.dumps(store.get(), allow_nan=False).encode()
                content_type = 'application/json'
            elif path in assets:
                name, content_type = assets[path]
                body = (web_root / name).read_bytes()
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data:; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'; object-src 'none'")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            pass

    return ThreadingHTTPServer((host, port), Handler)
