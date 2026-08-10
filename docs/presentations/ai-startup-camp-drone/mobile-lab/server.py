"""Optional, dependency-free score API for the AI startup camp mobile lab."""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import socket
import ssl
import threading
import unicodedata
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit


MAX_REQUEST_BYTES = 4096
MAX_UNIQUE_SUBMISSIONS = 500
TLS_HANDSHAKE_TIMEOUT_SECONDS = 1.0
HTTP_READ_TIMEOUT_SECONDS = 5.0
MAX_CONCURRENT_TLS_HANDSHAKES = 8
ALLOWED_FIELDS = frozenset(
    {"submission_id", "nickname", "score", "stability", "duration_ms", "mode"}
)
FONT_ALIAS_PREFIX = "/vendor/uos-slide-template/fonts/"
ALLOWED_FONT_FILENAMES = frozenset(
    {
        "NotoSansCJKkr-Regular.woff2",
        "NotoSansCJKkr-Medium.woff2",
        "NotoSansCJKkr-Bold.woff2",
    }
)
PUBLIC_SCORE_FIELDS = ("nickname", "score", "stability", "mode")


class SubmissionValidationError(ValueError):
    """The client supplied an invalid score payload."""


class SubmissionConflict(ValueError):
    """A submission ID was already accepted with a different payload."""


class SubmissionCapacityExceeded(ValueError):
    """The score service has reached its unique submission capacity."""


class MobileLabHTTPServer(ThreadingHTTPServer):
    """Threaded server sized to accept a full classroom submission burst."""

    request_queue_size = 64

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._tls_handshake_slots = threading.BoundedSemaphore(
            MAX_CONCURRENT_TLS_HANDSHAKES
        )

    def handle_error(self, request: object, client_address: object) -> None:
        del request, client_address

    def process_request_thread(
        self, request: socket.socket, client_address: tuple[str, int]
    ) -> None:
        if isinstance(request, ssl.SSLSocket):
            if not self._tls_handshake_slots.acquire(blocking=False):
                request.close()
                return
            try:
                request.settimeout(TLS_HANDSHAKE_TIMEOUT_SECONDS)
                request.do_handshake()
                request.settimeout(HTTP_READ_TIMEOUT_SECONDS)
            except (OSError, ssl.SSLError):
                request.close()
                return
            finally:
                self._tls_handshake_slots.release()

        try:
            self.finish_request(request, client_address)
        except Exception:
            self.handle_error(request, client_address)
        finally:
            self.shutdown_request(request)


def _invalid(message: str) -> None:
    raise SubmissionValidationError(message)


def _validate_payload(payload: object) -> dict[str, object]:
    if not isinstance(payload, dict):
        _invalid("payload must be a JSON object")
    if set(payload) != ALLOWED_FIELDS:
        _invalid("payload fields must exactly match the score schema")

    submission_id = payload["submission_id"]
    if not isinstance(submission_id, str):
        _invalid("submission_id must be a UUID string")
    try:
        canonical_id = str(uuid.UUID(submission_id))
    except (ValueError, AttributeError):
        _invalid("submission_id must be a UUID string")

    nickname = payload["nickname"]
    if not isinstance(nickname, str):
        _invalid("nickname must be a string")
    try:
        nickname.encode("utf-8")
    except UnicodeEncodeError:
        _invalid("nickname must be valid UTF-8")
    if any(
        unicodedata.category(character) in {"Cc", "Cf", "Cs"}
        for character in nickname
    ):
        _invalid("nickname must not contain control, format, or surrogate characters")
    nickname = nickname.strip() or "익명"
    if len(nickname) > 20:
        _invalid("nickname must contain at most 20 Unicode code points")

    score = payload["score"]
    if isinstance(score, bool) or not isinstance(score, int) or not 0 <= score <= 1000:
        _invalid("score must be an integer from 0 through 1000")

    stability = payload["stability"]
    if (
        isinstance(stability, bool)
        or not isinstance(stability, (int, float))
        or not math.isfinite(stability)
        or not 0 <= stability <= 100
    ):
        _invalid("stability must be a finite number from 0 through 100")

    duration_ms = payload["duration_ms"]
    if (
        isinstance(duration_ms, bool)
        or not isinstance(duration_ms, int)
        or not 1 <= duration_ms <= 600_000
    ):
        _invalid("duration_ms must be an integer from 1 through 600000")

    mode = payload["mode"]
    if not isinstance(mode, str) or mode not in {"touch", "motion"}:
        _invalid("mode must be touch or motion")

    return {
        "submission_id": canonical_id,
        "nickname": nickname,
        "score": score,
        "stability": float(stability),
        "duration_ms": duration_ms,
        "mode": mode,
    }


class ScoreStore:
    """Atomically accepts idempotent submissions and returns ranked snapshots."""

    def __init__(self, max_submissions: int = MAX_UNIQUE_SUBMISSIONS) -> None:
        if (
            isinstance(max_submissions, bool)
            or not isinstance(max_submissions, int)
            or max_submissions < 1
        ):
            raise ValueError("max_submissions must be a positive integer")
        self._lock = threading.Lock()
        self._by_id: dict[str, dict[str, object]] = {}
        self._next_sequence = 1
        self._max_submissions = max_submissions

    @staticmethod
    def _public_record(record: dict[str, object]) -> dict[str, object]:
        return {field: record[field] for field in PUBLIC_SCORE_FIELDS}

    def submit(self, payload: object) -> tuple[dict[str, object], bool]:
        canonical = _validate_payload(payload)
        submission_id = str(canonical["submission_id"])
        with self._lock:
            existing = self._by_id.get(submission_id)
            if existing is not None:
                if existing["payload"] != canonical:
                    raise SubmissionConflict(
                        "submission_id already belongs to a different payload"
                    )
                return self._public_record(existing["payload"]), False

            if len(self._by_id) >= self._max_submissions:
                raise SubmissionCapacityExceeded("score submission capacity reached")

            accepted_seq = self._next_sequence
            self._next_sequence += 1
            self._by_id[submission_id] = {
                "payload": dict(canonical),
                "accepted_seq": accepted_seq,
            }
            return self._public_record(canonical), True

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            best_by_nickname: dict[str, dict[str, object]] = {}
            for entry in self._by_id.values():
                record = {**entry["payload"], "accepted_seq": entry["accepted_seq"]}
                nickname = str(record["nickname"])
                current = best_by_nickname.get(nickname)
                if current is None or int(record["score"]) > int(current["score"]):
                    best_by_nickname[nickname] = record
            scores = list(best_by_nickname.values())
        scores.sort(key=lambda record: (-int(record["score"]), int(record["accepted_seq"])))
        return {
            "count": len(scores),
            "scores": [self._public_record(record) for record in scores[:10]],
        }


def build_server(
    host: str,
    port: int,
    static_root: str | Path,
    max_submissions: int = MAX_UNIQUE_SUBMISSIONS,
) -> MobileLabHTTPServer:
    """Build a thread-safe score API and static server rooted at ``static_root``."""
    root = Path(static_root).resolve()
    font_root = (root.parent / "vendor" / "uos-slide-template" / "fonts").resolve()
    store = ScoreStore(max_submissions=max_submissions)

    class ScoreRequestHandler(BaseHTTPRequestHandler):
        server_version = "MobileLabScoreServer/1.0"

        def log_message(self, format: str, *args: object) -> None:
            del format, args

        def _send_json(self, status: HTTPStatus, body: dict[str, object]) -> None:
            encoded = json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(encoded)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Permissions-Policy", "accelerometer=(self), gyroscope=(self)")
            self.end_headers()
            self.wfile.write(encoded)

        def _not_found(self) -> None:
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_GET(self) -> None:
            if urlsplit(self.path).path == "/api/scores":
                self._send_json(HTTPStatus.OK, store.snapshot())
                return
            self._serve_static()

        def _serve_static(self) -> None:
            request_path = unquote(urlsplit(self.path).path)
            if request_path == "/":
                request_path = "/index.html"
            try:
                if request_path.startswith(FONT_ALIAS_PREFIX):
                    font_filename = request_path.removeprefix(FONT_ALIAS_PREFIX)
                    if font_filename not in ALLOWED_FONT_FILENAMES:
                        self._not_found()
                        return
                    candidate = (font_root / font_filename).resolve()
                    candidate.relative_to(font_root)
                else:
                    candidate = (root / request_path.lstrip("/")).resolve()
                    candidate.relative_to(root)
            except (OSError, ValueError):
                self._not_found()
                return
            if not candidate.is_file():
                self._not_found()
                return
            try:
                content = candidate.read_bytes()
            except OSError:
                self._not_found()
                return
            content_type, _ = mimetypes.guess_type(candidate.name)
            if candidate.suffix == ".mjs":
                content_type = "text/javascript"
            self.send_response(HTTPStatus.OK)
            self.send_header(
                "Content-Type", (content_type or "application/octet-stream") + "; charset=utf-8"
            )
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Permissions-Policy", "accelerometer=(self), gyroscope=(self)")
            self.end_headers()
            self.wfile.write(content)

        def do_POST(self) -> None:
            if urlsplit(self.path).path != "/api/scores":
                self._not_found()
                return

            content_type = self.headers.get("Content-Type", "").split(";", 1)[0].lower()
            if content_type != "application/json":
                self._send_json(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, {"error": "application/json required"})
                return
            try:
                content_length = int(self.headers["Content-Length"])
            except (KeyError, TypeError, ValueError):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "valid Content-Length required"})
                return
            if content_length < 0:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "valid Content-Length required"})
                return
            if content_length > MAX_REQUEST_BYTES:
                self._send_json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": "request body too large"})
                return
            body = self.rfile.read(content_length)
            if len(body) != content_length:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "incomplete request body"})
                return
            try:
                payload = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "malformed JSON"})
                return
            try:
                record, created = store.submit(payload)
            except SubmissionValidationError as error:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
                return
            except SubmissionConflict as error:
                self._send_json(HTTPStatus.CONFLICT, {"error": str(error)})
                return
            except SubmissionCapacityExceeded as error:
                self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(error)})
                return
            self._send_json(
                HTTPStatus.CREATED if created else HTTPStatus.OK,
                {"accepted": True, "duplicate": not created, "record": record},
            )

    server = MobileLabHTTPServer((host, port), ScoreRequestHandler)
    server.daemon_threads = True
    server.score_store = store  # type: ignore[attr-defined]
    return server


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8000, type=int)
    parser.add_argument("--static-root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    arguments = parser.parse_args()
    if bool(arguments.cert) != bool(arguments.key):
        parser.error("--cert and --key must be supplied together")

    httpd = build_server(arguments.host, arguments.port, arguments.static_root)
    if arguments.cert:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(arguments.cert, arguments.key)
        httpd.socket = context.wrap_socket(
            httpd.socket,
            server_side=True,
            do_handshake_on_connect=False,
        )
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
