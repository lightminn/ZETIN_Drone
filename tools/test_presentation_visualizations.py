#!/usr/bin/env python3
"""Delivery checks for the audience-facing presentation visualizations."""

from __future__ import annotations

import json
import importlib.util
import shutil
import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone" / "assets"
GEOMETRY_PATH = (
    REPO_ROOT
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "visualizations"
    / "geometry.py"
)
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


def load_geometry_module():
    if not GEOMETRY_PATH.is_file():
        return None
    spec = importlib.util.spec_from_file_location(
        "presentation_visualization_geometry", GEOMETRY_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PresentationVisualizationGeometryTests(unittest.TestCase):
    def test_tilted_axis_components_reconstruct_downward_gravity(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)

        gravity, horizontal, vertical = module.gravity_components_2d(28.0, 2.5)

        for actual, expected in zip(gravity, (0.0, -2.5)):
            self.assertAlmostEqual(actual, expected, places=4)
        for actual, expected in zip(horizontal, (-1.0363, -0.5510)):
            self.assertAlmostEqual(actual, expected, places=4)
        for actual, expected in zip(vertical, (1.0363, -1.9490)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertAlmostEqual(horizontal[0] + vertical[0], gravity[0], places=4)
        self.assertAlmostEqual(horizontal[1] + vertical[1], gravity[1], places=4)

    def test_bias_example_matches_slide_value_after_sixty_seconds(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)
        self.assertTrue(
            hasattr(module, "integrated_bias_angle_deg"),
            "the bias scene must use a testable integration model",
        )

        self.assertAlmostEqual(
            module.integrated_bias_angle_deg(0.1, 60.0), 6.0, places=6
        )

    def test_sensor_to_body_mapping_matches_firmware_signs(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)
        self.assertTrue(
            hasattr(module, "transform_sensor_axes"),
            "the axis-sign scene must use the firmware sensor-to-body mapping",
        )

        sensor_sample = (1.0, 2.0, 3.0)
        self.assertEqual(
            module.transform_sensor_axes(sensor_sample, "gyro"),
            (2.0, -1.0, -3.0),
        )
        self.assertEqual(
            module.transform_sensor_axes(sensor_sample, "accel"),
            (2.0, -1.0, 3.0),
        )

    def test_magnetic_reference_preserves_heading_at_capture(self) -> None:
        module = load_geometry_module()
        self.assertIsNotNone(module)
        self.assertTrue(
            hasattr(module, "capture_heading_reference"),
            "the yaw scene must model a relative heading reference",
        )

        offset = module.capture_heading_reference(25.0, 62.0)
        self.assertAlmostEqual(offset, -37.0, places=6)
        self.assertAlmostEqual(
            module.referenced_heading_deg(62.0, offset), 25.0, places=6
        )
        self.assertAlmostEqual(
            module.referenced_heading_deg(82.0, offset), 45.0, places=6
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
