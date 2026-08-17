#!/usr/bin/env python3
"""Delivery checks for the audience-facing presentation visualizations."""

from __future__ import annotations

import json
import shutil
import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone" / "assets"
VISUALIZATION_FILES = (
    "accelerometer.mp4",
    "gyro.mp4",
    "complementary-filter.mp4",
    "gyro-bias.mp4",
    "imu-axis-signs.mp4",
    "pi-error-correction.mp4",
    "cascade-loop-timing.mp4",
    "yaw-correction.mp4",
    "landing-ambiguity.mp4",
)


@unittest.skipUnless(shutil.which("ffprobe"), "ffprobe is required")
class PresentationVisualizationDeliveryTests(unittest.TestCase):
    def test_explainer_videos_use_readable_browser_delivery_profile(self) -> None:
        for filename in VISUALIZATION_FILES:
            with self.subTest(filename=filename):
                completed = subprocess.run(
                    [
                        "ffprobe",
                        "-v",
                        "error",
                        "-select_streams",
                        "v:0",
                        "-show_entries",
                        "stream=codec_name,width,height,r_frame_rate,pix_fmt",
                        "-show_entries",
                        "format=duration",
                        "-of",
                        "json",
                        str(ASSET_DIR / filename),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                metadata = json.loads(completed.stdout)
                stream = metadata["streams"][0]
                duration = float(metadata["format"]["duration"])

                self.assertEqual(stream["codec_name"], "h264")
                self.assertEqual((stream["width"], stream["height"]), (1280, 720))
                self.assertEqual(stream["r_frame_rate"], "30/1")
                self.assertEqual(stream["pix_fmt"], "yuv420p")
                self.assertGreaterEqual(duration, 5.0)
                self.assertLessEqual(duration, 12.0)


if __name__ == "__main__":
    unittest.main()
