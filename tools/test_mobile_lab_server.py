"""Black-box HTTP tests for the optional mobile-lab score server."""

from __future__ import annotations

import json
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from http.client import HTTPResponse
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
sys.path.insert(0, str(MOBILE_LAB))

from server import build_server  # noqa: E402


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

    def post_json(self, payload: object) -> tuple[int, dict[str, str], dict[str, object]]:
        status, headers, body = self.request(
            "/api/scores",
            method="POST",
            body=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
