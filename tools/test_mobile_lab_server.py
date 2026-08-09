"""Black-box HTTP tests for the optional mobile-lab score server."""

from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from http.client import HTTPResponse, RemoteDisconnected
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
