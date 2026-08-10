"""Black-box HTTP tests for the optional mobile-lab score server."""

from __future__ import annotations

import contextlib
import io
import json
import signal
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from http.client import HTTPResponse, HTTPSConnection, RemoteDisconnected
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen


MOBILE_LAB = (
    Path(__file__).resolve().parents[1]
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "mobile-lab"
)
SIBLING_FONTS = MOBILE_LAB.parent / "vendor" / "uos-slide-template" / "fonts"
sys.path.insert(0, str(MOBILE_LAB))

from server import build_server  # noqa: E402


# Self-signed localhost material used only by the black-box TLS regression test.
TEST_CERTIFICATE = """-----BEGIN CERTIFICATE-----
MIIDCTCCAfGgAwIBAgIUAOfYBdBPCJ5Zt6LAtEz31NWkf64wDQYJKoZIhvcNAQEL
BQAwFDESMBAGA1UEAwwJbG9jYWxob3N0MB4XDTI2MDgwOTIyNTcyNloXDTM2MDgw
NjIyNTcyNlowFDESMBAGA1UEAwwJbG9jYWxob3N0MIIBIjANBgkqhkiG9w0BAQEF
AAOCAQ8AMIIBCgKCAQEA8Q2Rgy+vRRJpRTYih3yUqp4nwl8yquiv9guZTAAAFjpt
U0HNf4qbCS/ZGdQoNyRx7AmSv2b/i8kO0lRv1ehg+locmBc8ND2r+iOcR46OGcXn
qXdQq/2gorU26weFThE69EYBBLkXXPSXOKOTRvm0ZDfK10+XMRxOWtQzvi5eN6Bc
GZaDVKjs54+w8Dl9IRPRB+vsK7qa8VmNl9P1W6uWXNd/xmxRWiRhxmL3pkeifi40
3+liLOm4Hh5ir5vi3LKRNK6goqrnJs4GhnYbl8yrrsd4xV7klEEWsfrAB7JbyM0i
yLH3+RHy4xFQfLRIgJu+va9aGuKcCNB4RdOZynVY2QIDAQABo1MwUTAdBgNVHQ4E
FgQUX/KjNPKumvVd9ViwQbHTAlVsnQEwHwYDVR0jBBgwFoAUX/KjNPKumvVd9Viw
QbHTAlVsnQEwDwYDVR0TAQH/BAUwAwEB/zANBgkqhkiG9w0BAQsFAAOCAQEATmoi
YG6TZRfhJEYn/6+LFQ8Ysi+i6w0+3FwpRRy7Z0OeNX2tljEIUCWzRQA2JgzuthE2
83G5nvET/4LJIufX302ppnns+peFY+0XkpFQFltXvW9z9PS5h5Tba8Tl6C5+oWfE
t4fNguKVQt9wJYS00B6/W51iqnmqhxCosk7AXJj5PBUX1dN4NR8SFG14gDbXuI9d
eLeV6eDrmNiBle3qJ3fdowA6FBYuc8HCbWLAYcdq/wVNn8s9R24nQQp6Q/o4xrU7
WyLVT/ouktRsIL1KC/s94ytTZ90f95Uc4L/ecXNZAbS/lC4jGDSoPJhPIERuwRsr
BF0SIBHabNXmSiOHHA==
-----END CERTIFICATE-----
"""

TEST_PRIVATE_KEY = """-----BEGIN PRIVATE KEY-----
MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQDxDZGDL69FEmlF
NiKHfJSqnifCXzKq6K/2C5lMAAAWOm1TQc1/ipsJL9kZ1Cg3JHHsCZK/Zv+LyQ7S
VG/V6GD6WhyYFzw0Pav6I5xHjo4Zxeepd1Cr/aCitTbrB4VOETr0RgEEuRdc9Jc4
o5NG+bRkN8rXT5cxHE5a1DO+Ll43oFwZloNUqOznj7DwOX0hE9EH6+wruprxWY2X
0/Vbq5Zc13/GbFFaJGHGYvemR6J+LjTf6WIs6bgeHmKvm+LcspE0rqCiqucmzgaG
dhuXzKuux3jFXuSUQRax+sAHslvIzSLIsff5EfLjEVB8tEiAm769r1oa4pwI0HhF
05nKdVjZAgMBAAECggEAA2uDra8nGCkqMZ7fSSy29uxE4QpZKTKyYM+ZSKPn0+Bf
xQFx2KC/HiAZnQO/xurFo1ZKCKmXWosTs6PNnA+j+lN1RLLOfnGVWquxGnJ2+gZ3
iNXEFDmssNpHos365Qves86wYxwvF7CUnt6dQG10W22T0K1yuEdN8tIvJp4fOpWM
/ZbjbuQhn3n42gx5he35eElOW9cWeqKeBZka3Di5HobC7bhA6CkDtNP/8CkpA7Wl
FgcmjPmsar8DIejdus+cNjMIr+/nxUeD3OD95MRTViZUIJiFFslSjpbP7Fo7hT+j
Wu4bvnVbn3E0BFcFm9HelH2HMQFYaEELD9o+eiF53QKBgQD+AqNNa5AoqfhjPPZv
86joJAiaoCFpr+YfSVRl2vIhA7OtlU1AmqB22QD6s5UgtLKqBDGB11p2CxzhM6Zn
IpKcQkyGznw/Hq1b5M7ND+4ELBfRHq+jpnLjqpSDCxeHkMzmAlScYLlZWaNrzrvh
Twi2kzwuxWGEv5bNga1vz0K0DQKBgQDy8PKNgpznbLt39z23y0B22AKCaDFbsemH
XbF+Ib9FLjjtg4HtNsB5mzhXStswLQOOzYnIr57MMZ95RgheaoEQ8vnx/QYc5/ER
5ct0f//ZIpbyRU23JqNeu7Zc7xblojDQAg9nER21bn7ZxP2K+DZTeSFhYUJ18AxI
6bhImE0I/QKBgQCtN/dNEJEaae6tHiGgbrU8uXX0nEas3/s6UrNvUkPUJ8YcFbi8
2bWb1phIXrbPuuor7vgj50wVO7bSDHrp0jXQwZWWSLGKCc2G4R310WsrBTaosRht
rVCj2Ou2AZZmKGTSZbx1d7BuMiazmiOdnlv+xaFA8/FwqYaZVlmD6f8+8QKBgQDT
M9O6YJWlv+qO1dvLACFv8ETmOEzIybgbHcIjxJTzQMbu/cgjgNj+H4pwoTxC+q9A
I8IoPT2RiYZ5uP+njXHdWU8gKHd7A82ZYKxrAKhdjeuqfcOdeTLINerJinXedw12
mIPpd4DbbU9MZSyC91zLXuA9N3++5kzXypCSVA2MxQKBgFWvVjvuDd5m9k6HqQJm
13moVL5tWK+9DrQq8pO7WqGfEvNsMLz+5ZASLffD0MHMoH8tRqrllRrmfy0HYEEy
fa8bo+KLIjcq87AYIVDk9xjA7v2h6HJ8lKN7HWYUsYRCkjr1YFbmLW05GcDs8VbT
gFrMthb5tMH0DI1k9YL8tbyr
-----END PRIVATE KEY-----
"""


def payload_for(index: int = 1) -> dict[str, object]:
    return {
        "submission_id": f"01234567-89ab-4cde-8fab-{index:012d}",
        "nickname": "하늘01",
        "score": 876,
        "stability": 87.6,
        "duration_ms": 20000,
        "mode": "touch",
    }


class MobileLabServerTest(unittest.TestCase):
    """Each test uses a real, independently bound threaded HTTP server."""

    def setUp(self) -> None:
        self._temporary_directory = tempfile.TemporaryDirectory()
        static_root = Path(self._temporary_directory.name)
        (static_root / "index.html").write_text(
            "<!doctype html><title>student mobile lab</title>", encoding="utf-8"
        )
        (static_root / "presenter.html").write_text(
            "<!doctype html><title>presenter mobile lab</title>", encoding="utf-8"
        )
        (static_root / "src").mkdir()
        (static_root / "src" / "score-client.mjs").write_text(
            "export const scoreClient = true;\n", encoding="utf-8"
        )
        self.httpd = build_server("127.0.0.1", 0, static_root)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        host, port = self.httpd.server_address[:2]
        self.base_url = f"http://{host}:{port}"

    def tearDown(self) -> None:
        self.httpd.shutdown()
        self.thread.join(timeout=5)
        self.httpd.server_close()
        self._temporary_directory.cleanup()

    def request(
        self,
        path: str,
        *,
        method: str = "GET",
        body: bytes | None = None,
        content_type: str | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        headers = {} if content_type is None else {"Content-Type": content_type}
        request = Request(
            f"{self.base_url}{path}", data=body, headers=headers, method=method
        )
        try:
            response: HTTPResponse = urlopen(request, timeout=5)
        except HTTPError as error:
            response = error
        with response:
            return response.status, dict(response.headers.items()), response.read()

    def post_json(
        self, payload: object, *, ensure_ascii: bool = False
    ) -> tuple[int, dict[str, str], dict[str, object]]:
        status, headers, body = self.request(
            "/api/scores",
            method="POST",
            body=json.dumps(payload, ensure_ascii=ensure_ascii).encode("utf-8"),
            content_type="application/json",
        )
        return status, headers, json.loads(body)

    def get_scores(self) -> dict[str, object]:
        status, headers, body = self.request("/api/scores")
        self.assertEqual(200, status)
        self.assertEqual("no-store", headers["Cache-Control"])
        return json.loads(body)

    def test_valid_post_is_idempotent_and_conflicting_reuse_is_rejected(self) -> None:
        """A changed canonical payload must never overwrite an accepted submission."""
        payload = payload_for()

        status, headers, created = self.post_json(payload)
        self.assertEqual(201, status)
        self.assertEqual("no-store", headers["Cache-Control"])
        self.assertTrue(created["accepted"])
        self.assertFalse(created["duplicate"])
        self.assertEqual("하늘01", created["record"]["nickname"])
        self.assertEqual(1, self.get_scores()["count"])

        status, _, duplicate = self.post_json(payload)
        self.assertEqual(200, status)
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(created["record"], duplicate["record"])
        self.assertEqual(1, self.get_scores()["count"])

        conflicting = dict(payload)
        conflicting["score"] = 875
        status, _, rejected = self.post_json(conflicting)
        self.assertEqual(409, status)
        self.assertEqual("submission_id already belongs to a different payload", rejected["error"])
        self.assertEqual(1, self.get_scores()["count"])

    def test_invalid_payloads_do_not_change_the_score_count(self) -> None:
        """Malformed and invalid requests must be rejected before the store mutates."""
        cases = (
            (b"{", "application/json", 400),
            (json.dumps({**payload_for(), "extra": 1}).encode(), "application/json", 400),
            (json.dumps({**payload_for(), "score": True}).encode(), "application/json", 400),
            (json.dumps({**payload_for(), "mode": []}).encode(), "application/json", 400),
            (b"x" * 4097, "application/json", 413),
            (json.dumps(payload_for()).encode(), "text/plain", 415),
        )

        for body, content_type, expected_status in cases:
            status, _, _ = self.request(
                "/api/scores",
                method="POST",
                body=body,
                content_type=content_type,
            )
            self.assertEqual(expected_status, status)
            self.assertEqual(0, self.get_scores()["count"])

    def test_empty_nickname_is_normalized_and_score_list_is_ranked(self) -> None:
        """Normalization and ranking are observable through the public HTTP API."""
        lower = payload_for(2)
        lower.update({"nickname": "", "score": 12, "stability": 1.2})
        higher = payload_for(3)
        higher.update({"nickname": "다람쥐", "score": 900, "stability": 90.0})

        self.assertEqual(201, self.post_json(lower)[0])
        self.assertEqual(201, self.post_json(higher)[0])
        snapshot = self.get_scores()
        self.assertEqual(2, snapshot["count"])
        self.assertEqual([900, 12], [record["score"] for record in snapshot["scores"]])
        self.assertEqual("익명", snapshot["scores"][1]["nickname"])

    def test_nickname_rejects_raw_unsafe_unicode_before_normalization(self) -> None:
        """Trimming must not hide control, format, or non-UTF-8 surrogate input."""
        for index, nickname in enumerate(("\n팀", "팀\x00", "팀\u200d", "\ud800"), 10):
            with self.subTest(nickname=ascii(nickname)):
                status, _, _ = self.post_json(
                    {**payload_for(index), "nickname": nickname}, ensure_ascii=True
                )
                self.assertEqual(400, status)
                self.assertEqual({"count": 0, "scores": []}, self.get_scores())

        spaced = {**payload_for(20), "nickname": "  하늘  "}
        anonymous = {**payload_for(21), "nickname": "   "}
        self.assertEqual(201, self.post_json(spaced)[0])
        self.assertEqual(201, self.post_json(anonymous)[0])
        snapshot = self.get_scores()
        self.assertEqual(2, snapshot["count"])
        self.assertEqual({"하늘", "익명"}, {entry["nickname"] for entry in snapshot["scores"]})
        json.dumps(snapshot, ensure_ascii=False).encode("utf-8")

    def test_numeric_boundaries_booleans_and_non_finite_values(self) -> None:
        """Only the documented finite numeric domain may enter the score store."""
        valid_boundaries = (
            {"score": 0, "stability": 0, "duration_ms": 1},
            {"score": 1000, "stability": 100, "duration_ms": 600_000},
        )
        for index, values in enumerate(valid_boundaries, 30):
            with self.subTest(valid=values):
                self.assertEqual(201, self.post_json({**payload_for(index), **values})[0])

        invalid_values = (
            {"score": -1},
            {"score": 1001},
            {"score": 1.0},
            {"score": True},
            {"stability": -0.1},
            {"stability": 100.1},
            {"stability": True},
            {"stability": float("nan")},
            {"stability": float("inf")},
            {"stability": float("-inf")},
            {"duration_ms": 0},
            {"duration_ms": 600_001},
            {"duration_ms": 1.0},
            {"duration_ms": True},
        )
        for index, values in enumerate(invalid_values, 40):
            with self.subTest(invalid=values):
                self.assertEqual(400, self.post_json({**payload_for(index), **values})[0])
                self.assertEqual(2, self.get_scores()["count"])

    def test_post_response_does_not_expose_internal_score_metadata(self) -> None:
        """Submission responses must not leak identifiers, duration, or sequence."""
        status, _, body = self.post_json(payload_for(59))
        self.assertEqual(201, status)
        self.assertEqual(
            {"nickname", "score", "stability", "mode"}, set(body["record"])
        )

    def test_public_leaderboard_is_top_ten_minimal_and_ties_keep_acceptance_order(self) -> None:
        """The GET projection must be bounded, private, and deterministic."""
        scores = (900, 900, 1000, 950, 850, 800, 750, 700, 650, 600, 550, 500)
        for index, score in enumerate(scores, 60):
            status, _, _ = self.post_json(
                {**payload_for(index), "nickname": f"참짜{index}", "score": score}
            )
            self.assertEqual(201, status)

        snapshot = self.get_scores()
        self.assertEqual(12, snapshot["count"])
        self.assertEqual(10, len(snapshot["scores"]))
        self.assertEqual(
            ["참짜62", "참짜63", "참짜60", "참짜61"],
            [entry["nickname"] for entry in snapshot["scores"][:4]],
        )
        self.assertTrue(
            all(
                set(entry) == {"nickname", "score", "stability", "mode"}
                for entry in snapshot["scores"]
            )
        )

    def test_unexpected_handler_errors_are_quiet(self) -> None:
        """A classroom client fault must not print its address or a traceback."""
        def fail_snapshot() -> dict[str, object]:
            raise RuntimeError("synthetic handler failure")

        self.httpd.score_store.snapshot = fail_snapshot  # type: ignore[attr-defined]
        captured = io.StringIO()
        with contextlib.redirect_stderr(captured), self.assertRaises(RemoteDisconnected):
            urlopen(f"{self.base_url}/api/scores", timeout=5)
        self.assertEqual("", captured.getvalue())

    def test_static_routes_are_root_confined(self) -> None:
        """The static server must not turn encoded traversal into file access."""
        status, headers, body = self.request("/")
        self.assertEqual(200, status)
        self.assertIn(b"student mobile lab", body)
        self.assertEqual("accelerometer=(self), gyroscope=(self)", headers["Permissions-Policy"])

        status, _, body = self.request("/presenter.html")
        self.assertEqual(200, status)
        self.assertIn(b"presenter mobile lab", body)
        status, headers, body = self.request("/src/score-client.mjs")
        self.assertEqual(200, status)
        self.assertIn("text/javascript", headers["Content-Type"])
        self.assertIn(b"scoreClient", body)
        self.assertEqual(404, self.request("/missing.html")[0])
        self.assertEqual(404, self.request("/%2e%2e/%2e%2e/etc/passwd")[0])

    def test_fifty_unique_submissions_and_fifty_duplicate_retries_are_atomic(self) -> None:
        """A missing lock would lose IDs or accept more than one duplicate record."""
        unique_payloads = [
            {
                **payload_for(index),
                "score": index,
                "stability": float(index),
                "nickname": f"참가{index}",
            }
            for index in range(1, 51)
        ]

        with ThreadPoolExecutor(max_workers=50) as executor:
            unique_results = list(executor.map(self.post_json, unique_payloads))
        self.assertEqual([201] * 50, sorted(status for status, _, _ in unique_results))
        self.assertEqual(50, self.get_scores()["count"])

        retry_payload = payload_for(99)
        with ThreadPoolExecutor(max_workers=50) as executor:
            retry_results = list(executor.map(self.post_json, [retry_payload] * 50))
        statuses = [status for status, _, _ in retry_results]
        self.assertEqual(1, statuses.count(201))
        self.assertEqual(49, statuses.count(200))
        self.assertTrue(all(body["accepted"] for _, _, body in retry_results))
        self.assertEqual(51, self.get_scores()["count"])

    def test_submission_capacity_keeps_existing_retries_idempotent(self) -> None:
        """A full leaderboard must preserve retries, conflicts, and its last snapshot."""
        for index in range(1, 501):
            status, _, _ = self.post_json(payload_for(index))
            self.assertEqual(201, status, index)
        before_rejection = self.get_scores()
        self.assertEqual(500, before_rejection["count"])

        retry = payload_for(1)
        status, headers, duplicate = self.post_json(retry)
        self.assertEqual(200, status)
        self.assertEqual("no-store", headers["Cache-Control"])
        self.assertTrue(duplicate["duplicate"])

        conflict = {**retry, "score": 875}
        status, _, rejected = self.post_json(conflict)
        self.assertEqual(409, status)
        self.assertEqual(
            "submission_id already belongs to a different payload", rejected["error"]
        )

        status, headers, rejected = self.post_json(payload_for(501))
        self.assertEqual(503, status)
        self.assertEqual("no-store", headers["Cache-Control"])
        self.assertEqual({"error": "score submission capacity reached"}, rejected)
        self.assertEqual(before_rejection, self.get_scores())

        burst_httpd = build_server("127.0.0.1", 0, self._temporary_directory.name)
        burst_thread = threading.Thread(target=burst_httpd.serve_forever, daemon=True)
        burst_thread.start()
        burst_host, burst_port = burst_httpd.server_address[:2]
        burst_url = f"http://{burst_host}:{burst_port}"

        def submit_to_burst(payload: object) -> tuple[int, dict[str, object]]:
            request = Request(
                f"{burst_url}/api/scores",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                response: HTTPResponse = urlopen(request, timeout=5)
            except HTTPError as error:
                response = error
            with response:
                return response.status, json.loads(response.read())

        try:
            for index in range(1, 500):
                status, _ = submit_to_burst(payload_for(index))
                self.assertEqual(201, status, index)
            with ThreadPoolExecutor(max_workers=50) as executor:
                results = list(
                    executor.map(submit_to_burst, (payload_for(index) for index in range(500, 550)))
                )
            statuses = [status for status, _ in results]
            self.assertEqual(1, statuses.count(201))
            self.assertEqual(49, statuses.count(503))
            self.assertTrue(
                all(
                    body == {"error": "score submission capacity reached"}
                    for status, body in results
                    if status == 503
                )
            )
            snapshot = urlopen(f"{burst_url}/api/scores", timeout=5)
            with snapshot:
                self.assertEqual(500, json.loads(snapshot.read())["count"])
        finally:
            burst_httpd.shutdown()
            burst_thread.join(timeout=5)
            burst_httpd.server_close()


class MobileLabTLSConcurrencyTest(unittest.TestCase):
    """A stalled TLS client must not block the classroom accept loop."""

    @contextlib.contextmanager
    def tls_server(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            certificate_path = temporary_path / "localhost-cert.pem"
            private_key_path = temporary_path / "localhost-key.pem"
            certificate_path.write_text(TEST_CERTIFICATE, encoding="ascii")
            private_key_path.write_text(TEST_PRIVATE_KEY, encoding="ascii")

            reservation = socket.socket()
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
            reservation.close()

            process = subprocess.Popen(
                (
                    sys.executable,
                    str(MOBILE_LAB / "server.py"),
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(port),
                    "--cert",
                    str(certificate_path),
                    "--key",
                    str(private_key_path),
                ),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                yield process, port
            finally:
                shutdown_started = time.monotonic()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
                    self.fail("TLS server close exceeded two seconds")
                self.assertLess(time.monotonic() - shutdown_started, 2)

    def wait_for_raw_connection(self, process: subprocess.Popen[bytes], port: int) -> socket.socket:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if process.poll() is not None:
                self.fail(f"TLS server exited with status {process.returncode}")
            try:
                return socket.create_connection(("127.0.0.1", port), timeout=0.1)
            except OSError:
                time.sleep(0.02)
        self.fail("TLS server did not start")

    def get_scores(self, port: int) -> tuple[float, int, dict[str, object]]:
        connection = HTTPSConnection(
            "127.0.0.1",
            port,
            context=ssl._create_unverified_context(),
            timeout=2,
        )
        started = time.monotonic()
        try:
            connection.request("GET", "/api/scores")
            response = connection.getresponse()
            return time.monotonic() - started, response.status, json.loads(response.read())
        finally:
            connection.close()

    def assert_socket_closed(self, client: socket.socket, timeout_seconds: float) -> None:
        deadline = time.monotonic() + timeout_seconds
        client.settimeout(0.1)
        while time.monotonic() < deadline:
            try:
                received = client.recv(1)
            except socket.timeout:
                continue
            except OSError:
                return
            self.assertEqual(b"", received)
            return
        self.fail("idle TLS socket remained open")

    def test_idle_tcp_client_does_not_block_a_valid_tls_request(self) -> None:
        """Wrapping the listener with eager handshakes would serialize all clients."""
        with self.tls_server() as (process, port):
            idle_client = self.wait_for_raw_connection(process, port)
            try:
                elapsed, status, body = self.get_scores(port)
                self.assertLess(elapsed, 2)
                self.assertEqual(200, status)
                self.assertEqual({"count": 0, "scores": []}, body)
            finally:
                idle_client.close()

    def test_idle_tls_handshakes_are_bounded_reclaimed_and_do_not_delay_shutdown(self) -> None:
        """Excess raw TCP clients must not retain unbounded TLS workers."""
        with self.tls_server() as (process, port):
            idle_clients = [self.wait_for_raw_connection(process, port)]
            idle_clients.extend(
                socket.create_connection(("127.0.0.1", port), timeout=1)
                for _ in range(11)
            )
            try:
                self.assert_socket_closed(idle_clients[-1], 0.75)
                for client in idle_clients[:-1]:
                    self.assert_socket_closed(client, 1.5)

                elapsed, status, body = self.get_scores(port)
                self.assertLess(elapsed, 2)
                self.assertEqual(200, status)
                self.assertEqual({"count": 0, "scores": []}, body)
            finally:
                for client in idle_clients:
                    client.close()


class DefaultMobileLabTopologyTest(unittest.TestCase):
    """The product's default static root may expose only its explicit sibling-font alias."""

    def setUp(self) -> None:
        self.httpd = build_server("127.0.0.1", 0, MOBILE_LAB)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        host, port = self.httpd.server_address[:2]
        self.base_url = f"http://{host}:{port}"

    def tearDown(self) -> None:
        self.httpd.shutdown()
        self.thread.join(timeout=5)
        self.httpd.server_close()

    def request(self, path: str) -> tuple[int, dict[str, str], bytes]:
        try:
            response: HTTPResponse = urlopen(f"{self.base_url}{path}", timeout=5)
        except HTTPError as error:
            response = error
        with response:
            return response.status, dict(response.headers.items()), response.read()

    def test_default_product_server_serves_only_the_three_sibling_fonts(self) -> None:
        """Removing the exact alias breaks binding typography; widening it leaks parent files."""
        for filename in (
            "NotoSansCJKkr-Regular.woff2",
            "NotoSansCJKkr-Medium.woff2",
            "NotoSansCJKkr-Bold.woff2",
        ):
            status, headers, body = self.request(
                f"/vendor/uos-slide-template/fonts/{filename}"
            )
            self.assertEqual(200, status, filename)
            self.assertIn("font/woff2", headers["Content-Type"])
            self.assertEqual((SIBLING_FONTS / filename).read_bytes(), body)

        self.assertEqual(404, self.request("/vendor/uos-slide-template/styles.css")[0])
        self.assertEqual(404, self.request("/vendor/uos-slide-template/fonts/fonts.css")[0])
        self.assertEqual(
            404,
            self.request(
                "/vendor/uos-slide-template/fonts/%2e%2e/styles.css"
            )[0],
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
