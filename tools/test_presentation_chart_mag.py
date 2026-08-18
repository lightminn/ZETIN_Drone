"""Contract tests for the reproducible magnetic-interference chart."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from PIL import Image


REPO_ROOT = Path(__file__).resolve().parents[1]
DECK_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone"
RENDERER_PATH = DECK_DIR / "visualizations" / "render_chart_mag.py"
CHARTDATA_PATH = DECK_DIR / "chartdata.json"


def load_renderer():
    spec = importlib.util.spec_from_file_location("render_chart_mag", RENDERER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PresentationMagChartTests(unittest.TestCase):
    def test_renderer_exposes_semantics_and_renders_deterministic_hd_png(self) -> None:
        """The delivered chart states its evidence, comparison, and encodings."""
        renderer = load_renderer()
        chartdata = json.loads(CHARTDATA_PATH.read_text(encoding="utf-8"))
        specification = renderer.build_chart_spec(CHARTDATA_PATH)

        self.assertEqual(renderer.CANVAS_SIZE, (1280, 720))
        self.assertGreaterEqual(
            renderer.LAYOUT_METADATA["plot_top"]
            - renderer.LAYOUT_METADATA["plot_bottom"],
            0.54,
        )
        self.assertLessEqual(1.0 - renderer.LAYOUT_METADATA["plot_top"], 0.22)
        self.assertGreaterEqual(
            renderer.LAYOUT_METADATA["title_y"]
            - renderer.LAYOUT_METADATA["normalization_caption_y"],
            0.05,
        )
        critical_type_sizes = [
            renderer.TYPOGRAPHY[key]
            for key in (
                "panel_title_pt",
                "axis_label_pt",
                "tick_label_pt",
                "legend_pt",
                "slope_box_pt",
            )
        ]
        rendered_font_pixels = min(critical_type_sizes) * 100 / 72
        effective_font_pixels = (
            rendered_font_pixels
            * renderer.TYPOGRAPHY["slide_display_width_px"]
            / renderer.CANVAS_SIZE[0]
        )
        self.assertGreaterEqual(effective_font_pixels, 17.0)
        self.assertEqual(
            renderer.PRESENTATION_TEXT["normalization_caption"],
            "기준선: 스로틀 1000 µs 구간 평균을 0°로 정규화",
        )
        displayed_text = " ".join(renderer.PRESENTATION_TEXT.values())
        self.assertNotIn("2026-07-27", displayed_text)
        self.assertNotIn("chartdata.json", displayed_text)
        self.assertEqual(
            renderer.SEMANTIC_METADATA["panel_titles"],
            ["전류 간섭 보정 OFF", "전류 간섭 보정 ON"],
        )
        self.assertEqual(renderer.SEMANTIC_METADATA["point_label"], "실측 샘플")
        self.assertEqual(renderer.SEMANTIC_METADATA["line_label"], "회귀 추세선")
        self.assertEqual(renderer.SEMANTIC_METADATA["x_label"], "스로틀 (µs)")
        self.assertEqual(
            renderer.SEMANTIC_METADATA["y_label"], "기준 대비 MagHeading 변화 (°)"
        )
        self.assertIn("throttle=1000", renderer.SEMANTIC_METADATA["normalization"])
        self.assertEqual(
            renderer.SEMANTIC_METADATA["mark_encoding"],
            {"points": "실측 샘플", "solid_line": "회귀 추세선"},
        )

        for key, panel in zip(("magOff", "magOn"), specification["panels"]):
            series = chartdata[key]["series"]
            self.assertEqual(panel["sample_count"], len(series["thr"]))
            self.assertEqual(panel["sample_count"], len(series["head"]))
            self.assertEqual(panel["stored_slope"], chartdata[key]["slope"])
            self.assertEqual(
                panel["slope_label"],
                f"{chartdata[key]['slope']:+.2f}°/100µs",
            )
            self.assertGreater(panel["baseline_sample_count"], 0)

        with tempfile.TemporaryDirectory() as temporary_directory:
            output_a = Path(temporary_directory) / "chart-a.png"
            output_b = Path(temporary_directory) / "chart-b.png"
            renderer.render_chart(output_a, CHARTDATA_PATH)
            renderer.render_chart(output_b, CHARTDATA_PATH)

            for output in (output_a, output_b):
                self.assertTrue(output.is_file())
                with Image.open(output) as image:
                    self.assertEqual(image.size, (1280, 720))
                    self.assertIn(image.mode, {"RGB", "RGBA"})
            self.assertEqual(
                hashlib.sha256(output_a.read_bytes()).digest(),
                hashlib.sha256(output_b.read_bytes()).digest(),
            )

    def test_stored_slope_label_changes_when_chartdata_is_mutated(self) -> None:
        """A literal must come from chartdata, rather than a stale renderer constant."""
        renderer = load_renderer()
        with tempfile.TemporaryDirectory() as temporary_directory:
            mutated_path = Path(temporary_directory) / "chartdata.json"
            shutil.copyfile(CHARTDATA_PATH, mutated_path)
            chartdata = json.loads(mutated_path.read_text(encoding="utf-8"))
            chartdata["magOn"]["slope"] = 0.99
            mutated_path.write_text(
                json.dumps(chartdata, ensure_ascii=False), encoding="utf-8"
            )

            specification = renderer.build_chart_spec(mutated_path)

        self.assertEqual(specification["panels"][1]["stored_slope"], 0.99)
        self.assertEqual(specification["panels"][1]["slope_label"], "+0.99°/100µs")


if __name__ == "__main__":
    unittest.main()
