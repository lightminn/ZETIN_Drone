#!/usr/bin/env python3
"""Delivery checks for the audience-facing presentation visualizations."""

from __future__ import annotations

import json
import importlib.util
import shutil
import subprocess
import sys
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
VISUALIZATION_SOURCE_PATH = GEOMETRY_PATH.with_name("audience_visualizations.py")
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


def load_visualization_module():
    if not VISUALIZATION_SOURCE_PATH.is_file():
        return None
    source_dir = str(VISUALIZATION_SOURCE_PATH.parent)
    if source_dir not in sys.path:
        sys.path.insert(0, source_dir)
    spec = importlib.util.spec_from_file_location(
        "presentation_audience_visualizations", VISUALIZATION_SOURCE_PATH
    )
    if spec is None or spec.loader is None:
        return None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PresentationVisualizationGeometryTests(unittest.TestCase):
    def test_video_text_is_rendered_ten_percent_larger(self) -> None:
        module = load_visualization_module()
        self.assertIsNotNone(module)

        rendered = module.text("가독성", 20)

        self.assertAlmostEqual(rendered.font_size, 22.0, places=6)

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


class PresentationVisualizationLayoutTests(unittest.TestCase):
    @staticmethod
    def _rendered_scene(scene_name: str):
        from manim import tempconfig

        module = load_visualization_module()
        if module is None:
            raise AssertionError("presentation visualization module is missing")
        with tempconfig(
            {
                "dry_run": True,
                "disable_caching": True,
                "verbosity": "ERROR",
                "frame_rate": 1,
                "progress_bar": "none",
            }
        ):
            scene = getattr(module, scene_name)()
            scene.hold_and_clear = lambda *args, **kwargs: None
            scene.render()
        return scene

    @staticmethod
    def _mobjects(scene):
        def descendants(mobject):
            yield mobject
            for child in mobject.submobjects:
                yield from descendants(child)

        for root in scene.mobjects:
            yield from descendants(root)

    @classmethod
    def _text(cls, scene, normalized_text: str):
        for mobject in cls._mobjects(scene):
            if getattr(mobject, "text", None) == normalized_text:
                return mobject
        raise AssertionError(f"missing rendered text: {normalized_text}")

    def test_accelerometer_component_labels_use_the_explanation_zone(self) -> None:
        scene = self._rendered_scene("AccelerometerAudience")

        horizontal = self._text(scene, "수평축성분")
        vertical = self._text(scene, "수직축성분")

        self.assertGreater(horizontal.get_left()[0], 2.8)
        self.assertGreater(vertical.get_left()[0], 2.8)

    def test_pi_curve_labels_stay_above_the_plot_lines(self) -> None:
        scene = self._rendered_scene("PiErrorAudience")

        p_label = self._text(scene, "P만:오차가남음")
        pi_label = self._text(scene, "P+I:오차가0으로복귀")

        self.assertGreater(p_label.get_bottom()[1], 1.3)
        self.assertGreater(pi_label.get_bottom()[1], 1.3)

    def test_complementary_filter_descriptions_clear_the_plot_area(self) -> None:
        scene = self._rendered_scene("ComplementaryFilterAudience")
        descriptions = [
            self._text(scene, "빠르지만서서히표류"),
            self._text(scene, "장기기준이지만순간진동"),
            self._text(scene, "빠르고기준에서벗어나지않음"),
        ]
        curves = sorted(
            (
                item
                for item in self._mobjects(scene)
                if type(item).__name__ == "VMobject"
                and len(item.get_all_points()) > 500
                and str(item.get_color()) in {"#FF6474", "#FF9F43", "#55D68B"}
            ),
            key=lambda item: item.get_center()[1],
            reverse=True,
        )

        self.assertEqual(len(curves), 3)
        for description, curve in zip(descriptions, curves):
            self.assertLess(
                description.get_right()[0] + 0.18,
                curve.get_left()[0],
            )

    def test_cascade_row_labels_clear_the_first_target_box(self) -> None:
        scene = self._rendered_scene("CascadeTimingAudience")

        outer = self._text(scene, "바깥자세루프")
        inner = self._text(scene, "안쪽각속도루프")
        cadence = self._text(scene, "자세목표사이에서여러번보정")

        self.assertLess(outer.get_right()[0], -4.4)
        self.assertLess(inner.get_right()[0], -4.4)
        self.assertGreater(cadence.get_bottom()[1], -3.2)

    def test_gyro_bias_axis_label_is_horizontal_above_the_plot(self) -> None:
        scene = self._rendered_scene("GyroBiasAudience")

        axis_label = self._text(scene, "누적각도오차")

        self.assertGreater(axis_label.width, axis_label.height)
        self.assertGreater(axis_label.get_bottom()[1], 1.25)

    def test_landing_comparison_label_sits_above_both_panels(self) -> None:
        scene = self._rendered_scene("LandingAmbiguityAudience")

        comparison = self._text(scene, "움직임은다르지만센서값은같다")

        self.assertGreater(comparison.get_bottom()[1], 2.05)


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
