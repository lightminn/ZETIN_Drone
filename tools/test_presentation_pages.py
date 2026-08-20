#!/usr/bin/env python3
"""Contract tests for the static presentation site published to Pages."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "tools" / "build_presentation_site.py"


class PresentationPagesBuildTests(unittest.TestCase):
    def _build_site(self, root: Path) -> Path:
        output = root / "site"
        result = subprocess.run(
            [sys.executable, str(BUILD_SCRIPT), "--output", str(output)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            result.returncode,
            0,
            msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
        )
        return output

    def test_build_contains_every_browser_runtime_asset(self) -> None:
        with tempfile.TemporaryDirectory(prefix="zetin-pages-test-") as temp_dir:
            site = self._build_site(Path(temp_dir))
            required = {
                ".nojekyll",
                "index.html",
                "support.js",
                "deck-stage.js",
                "vendor/uos-slide-template/_ds_bundle.js",
                "vendor/uos-slide-template/_ds_bundle.css",
                "vendor/uos-slide-template/fonts/NotoSansCJKkr-Medium.woff2",
                "assets/accelerometer.mp4",
                "assets/hover_demo.mp4",
                "assets/mobile-lab-qr.svg",
                "10min/index.html",
                "10min/support.js",
                "10min/deck-stage.js",
                "10min/vendor/uos-slide-template/_ds_bundle.js",
                "10min/vendor/uos-slide-template/fonts/NotoSansCJKkr-Medium.woff2",
                "10min/assets/hover_demo.mp4",
            }
            missing = sorted(path for path in required if not (site / path).is_file())

            self.assertEqual(missing, [])

    def test_build_excludes_delivery_and_source_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="zetin-pages-test-") as temp_dir:
            site = self._build_site(Path(temp_dir))
            forbidden_suffixes = {".cjs", ".md", ".pdf", ".pptx", ".py", ".sh", ".zip"}
            forbidden = sorted(
                path.relative_to(site).as_posix()
                for path in site.rglob("*")
                if path.is_file() and path.suffix.lower() in forbidden_suffixes
            )

            self.assertEqual(forbidden, [])


if __name__ == "__main__":
    unittest.main()
