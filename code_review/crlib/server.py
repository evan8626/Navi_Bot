"""Live report server: rebuild when a file changes and push the update to the page.

Bound to 127.0.0.1 only. State-changing requests must carry a custom header,
which a web page on another origin cannot send without a CORS preflight this
server never grants - so another site open in the browser cannot triage
findings or start test runs here.
"""

from __future__ import annotations

import contextlib
import json
import queue
import threading
import time
import urllib.request
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import config, findings, report, sources, testsuite

LOCAL_HOSTS = {"127.0.0.1", "localhost"}
REQUIRED_HEADER = "X-Navi-Review"
MAX_BODY_BYTES = 64 * 1024
HEARTBEAT_SECONDS = 15
WATCH_LIST_REFRESH_TICKS = 10
SETTLE_SECONDS = 0.3


def _log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}")


class LiveReport:
    """Holds the latest report and the browsers waiting to hear about a new one."""

    def __init__(self) -> None:
        self._cache: report.AnalysisCache = {}
        self._build_lock = threading.Lock()
        self._subscribers: list[queue.Queue] = []
        self._subscribers_lock = threading.Lock()
        self._tests_running = False
        self.payload = "{}"

    # -- events ---------------------------------------------------------
    def subscribe(self) -> queue.Queue:
        """Register a browser connection; returns the queue its events arrive on."""
        channel: queue.Queue = queue.Queue()
        with self._subscribers_lock:
            self._subscribers.append(channel)
        return channel

    def unsubscribe(self, channel: queue.Queue) -> None:
        """Forget a browser connection that has gone away."""
        with self._subscribers_lock:
            if channel in self._subscribers:
                self._subscribers.remove(channel)

    def publish(self, kind: str, **fields) -> None:
        """Send one event to every connected page."""
        event = json.dumps(dict(fields, type=kind))
        with self._subscribers_lock:
            for channel in self._subscribers:
                channel.put(event)

    # -- building -------------------------------------------------------
    def rebuild(self, reason: str, *, force_probes: bool = False) -> None:
        """Rebuild the report and tell every open page."""
        with self._build_lock:
            self.publish("building", reason=reason)
            try:
                data = report.build(self._cache, force_probes=force_probes, live=True)
                report.write(data)
                self.payload = report.to_json(data)
            except Exception as error:  # noqa: BLE001 - keep serving the previous report
                self.publish("failed", message=f"{type(error).__name__}: {error}")
                _log(f"rebuild failed: {type(error).__name__}: {error}")
                return
            self.publish("updated", reason=reason)
            _log(f"rebuilt ({reason}) - {data['summary']['open']} open, "
                 f"{data['meta']['build_seconds']} s")

    def rebuild_in_background(self, reason: str, *, force_probes: bool = False) -> None:
        """Start a rebuild without making the HTTP request wait for it."""
        threading.Thread(target=self.rebuild, args=(reason,),
                         kwargs={"force_probes": force_probes}, daemon=True).start()

    def run_tests(self) -> bool:
        """Start the test suites in the background; False if already running."""
        if self._tests_running:
            return False
        self._tests_running = True

        def work() -> None:
            try:
                self.publish("tests-started")
                testsuite.run_all(sources.load_all())
            finally:
                self._tests_running = False
            self.rebuild("test suites finished")

        threading.Thread(target=work, name="test-suites", daemon=True).start()
        return True

    # -- watching -------------------------------------------------------
    @staticmethod
    def _watch_list() -> list[Path]:
        """Analysed files, review inputs, and files whose *absence* is a finding."""
        paths = sources.watched_paths()
        try:
            awaited = [finding["open_if_missing_file"] for finding in findings.load()
                       if finding.get("open_if_missing_file")]
        except (OSError, ValueError):
            awaited = []                            # a half-saved TOML file; next tick retries
        return paths + [config.REPO_ROOT / name for name in awaited]

    @staticmethod
    def _snapshot(paths: list[Path]) -> dict[Path, tuple[int, int] | None]:
        state: dict[Path, tuple[int, int] | None] = {}
        for path in paths:
            try:
                stat = path.stat()
                state[path] = (stat.st_mtime_ns, stat.st_size)
            except OSError:
                state[path] = None
        return state

    def watch(self) -> None:
        """Poll the watched files forever and rebuild when any of them changes."""
        watch_list = self._watch_list()
        known = self._snapshot(watch_list)
        tick = 0
        while True:
            time.sleep(config.WATCH_INTERVAL_SECONDS)
            tick += 1
            if tick % WATCH_LIST_REFRESH_TICKS == 0:
                watch_list = self._watch_list()
            current = self._snapshot(watch_list)
            changed = [path for path in current if current[path] != known.get(path)]
            changed += [path for path in known if path not in current]
            if not changed:
                continue
            time.sleep(SETTLE_SECONDS)              # let the editor finish writing
            names = ", ".join(sorted({path.name for path in changed})[:3])
            self.rebuild(f"{names} changed")
            watch_list = self._watch_list()
            known = self._snapshot(watch_list)


class ReviewHandler(BaseHTTPRequestHandler):
    """HTTP endpoints of the live report. ``live`` is bound by :func:`serve`."""

    live: LiveReport
    server_version = "NaviReview/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, *_args) -> None:
        """Requests are not worth a log line each."""

    # -- helpers ----------------------------------------------------------
    def _send(self, status: HTTPStatus, body: str, content_type: str) -> None:
        encoded = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(encoded)

    def _json(self, status: HTTPStatus, data: dict) -> None:
        self._send(status, json.dumps(data), "application/json")

    def _is_local(self) -> bool:
        return self.headers.get("Host", "").rsplit(":", 1)[0] in LOCAL_HOSTS

    def _route(self) -> str:
        """The request path without its query string."""
        return self.path.split("?", 1)[0]

    def _read_body(self) -> dict | None:
        length = int(self.headers.get("Content-Length") or 0)
        if not 0 <= length <= MAX_BODY_BYTES:
            return None
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            return None
        return body if isinstance(body, dict) else None

    # -- routes -----------------------------------------------------------
    def do_GET(self) -> None:
        """Serve the page, the report data, or the event stream."""
        route = self._route()
        if not self._is_local():
            self._json(HTTPStatus.FORBIDDEN, {"error": "local connections only"})
        elif route in ("/", "/index.html"):
            self._send(HTTPStatus.OK, report.embed(self.live.payload), "text/html")
        elif route == "/api/report":
            self._send(HTTPStatus.OK, self.live.payload, "application/json")
        elif route == "/api/events":
            self._stream_events()
        else:
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:
        """Handle triage decisions, rebuild requests and test runs."""
        body = self._read_body()
        route = self._route()
        trusted = self._is_local() and self.headers.get(REQUIRED_HEADER) == "1"
        if not trusted or body is None:
            self._json(HTTPStatus.FORBIDDEN, {"error": "request refused"})
        elif route == "/api/triage":
            self._triage(body)
        elif route == "/api/rebuild":
            self.live.rebuild_in_background("requested from the page",
                                            force_probes=bool(body.get("force_probes")))
            self._json(HTTPStatus.ACCEPTED, {"ok": True})
        elif route == "/api/tests":
            self._json(HTTPStatus.ACCEPTED, {"ok": True, "started": self.live.run_tests()})
        else:
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def _triage(self, body: dict) -> None:
        known = {finding["id"] for finding in findings.load()}
        finding_id, status = str(body.get("id", "")), str(body.get("status", ""))
        if finding_id not in known or status not in findings.TRIAGE_STATUSES | {"clear"}:
            self._json(HTTPStatus.BAD_REQUEST, {"error": "unknown finding or status"})
            return
        findings.record_triage(finding_id, status, str(body.get("note", "")))
        self._json(HTTPStatus.OK, {"ok": True})
        self.live.rebuild_in_background(f"{finding_id} triaged")

    def _stream_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        channel = self.live.subscribe()
        with contextlib.suppress(OSError):          # the browser tab was closed
            while True:
                try:
                    message = f"data: {channel.get(timeout=HEARTBEAT_SECONDS)}\n\n"
                except queue.Empty:
                    message = ": keep-alive\n\n"
                self.wfile.write(message.encode("utf-8"))
                self.wfile.flush()
        self.live.unsubscribe(channel)
        self.close_connection = True


def _already_serving(address: str) -> bool:
    """True when a review server is already answering at ``address``."""
    try:
        with urllib.request.urlopen(address + "api/report", timeout=2) as response:
            return b'"meta"' in response.read(64)
    except (OSError, ValueError):
        return False


def serve(port: int = config.DEFAULT_PORT, open_browser: bool = True) -> None:
    """Build once, then serve the live report until interrupted."""
    address = f"http://127.0.0.1:{port}/"
    if _already_serving(address):
        print(f"the live code review is already running at {address}")
        if open_browser:
            webbrowser.open(address)
        return
    live = LiveReport()
    print("building the report (the first build runs every probe; later ones are incremental)...")
    live.rebuild("server started")
    handler = type("BoundReviewHandler", (ReviewHandler,), {"live": live})
    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), handler)
    except OSError as error:
        raise SystemExit(f"port {port} is in use by another program ({error}); "
                         f"try: python code_review/review.py serve --port {port + 1}") from None
    server.daemon_threads = True
    threading.Thread(target=live.watch, name="file-watcher", daemon=True).start()
    print(f"live code review at {address}  (Ctrl+C or close this window to stop)")
    if open_browser:
        webbrowser.open(address)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        server.server_close()
