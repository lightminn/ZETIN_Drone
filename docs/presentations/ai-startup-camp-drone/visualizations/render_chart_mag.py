#!/usr/bin/env python3
"""Render the reproducible current-interference MagHeading comparison chart.

The displayed changes are normalized by subtracting the mean MagHeading of
each dataset's throttle=1000 µs samples.  The stored slopes remain historical
analysis values from the 2026-07-27 benchmark; the solid lines are separately
fitted least-squares trends through the plotted samples.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


CANVAS_SIZE = (1280, 720)
DEFAULT_DATA_PATH = Path(__file__).resolve().parents[1] / "chartdata.json"
DEFAULT_OUTPUT_PATH = Path(__file__).resolve().parents[1] / "assets" / "chart_mag.png"

# This constant lets delivery checks verify chart semantics without OCR.
SEMANTIC_METADATA = {
    "panel_titles": ["전류 간섭 보정 OFF", "전류 간섭 보정 ON"],
    "point_label": "실측 샘플",
    "line_label": "회귀 추세선",
    "x_label": "스로틀 (µs)",
    "y_label": "기준 대비 MagHeading 변화 (°)",
    "normalization": "각 데이터셋의 throttle=1000 µs MagHeading 평균을 0° 기준으로 뺍니다.",
    "mark_encoding": {"points": "실측 샘플", "solid_line": "회귀 추세선"},
    "slope_note": "저장 분석 기울기: 2026-07-27 벤치마크",
}

# Visible text excludes internal filenames and dates; provenance remains in
# source data and the semantic metadata above for reproducible verification.
PRESENTATION_TEXT = {
    "title": "전류 간섭 보정에 따른 MagHeading 변화",
    "normalization_caption": "기준선: 스로틀 1000 µs 구간 평균을 0°로 정규화",
    "slope_heading": "벤치 기록 기울기",
    "legend_caption": "점: 실측 샘플   ·   실선: 회귀 추세선",
}

# At the smallest intended slide placement (1100 px wide), each critical
# 15 pt label renders at about 17.9 px: 15 × 100 / 72 × 1100 / 1280.
TYPOGRAPHY = {
    "slide_display_width_px": 1100,
    "title_pt": 25,
    "normalization_caption_pt": 15,
    "panel_title_pt": 23,
    "axis_label_pt": 19,
    "tick_label_pt": 15,
    "legend_pt": 15,
    "slope_box_pt": 15,
    "footer_pt": 14,
}

LAYOUT_METADATA = {
    "plot_left": 0.085,
    "plot_right": 0.975,
    "plot_bottom": 0.25,
    "plot_top": 0.79,
    "plot_spacing": 0.14,
    "title_y": 0.95,
    "normalization_caption_y": 0.895,
}

PANEL_KEYS = ("magOff", "magOn")
PANEL_COLORS = ("#174F87", "#2C6B99")
POINT_COLORS = ("#8A9AAF", "#9AACBD")


def build_chart_spec(data_path: Path | str = DEFAULT_DATA_PATH) -> dict[str, Any]:
    """Load chartdata and return the normalized points and their visible labels."""
    chartdata_path = Path(data_path)
    raw_data = json.loads(chartdata_path.read_text(encoding="utf-8"))
    panels: list[dict[str, Any]] = []

    for key, title in zip(PANEL_KEYS, SEMANTIC_METADATA["panel_titles"]):
        stored = raw_data[key]
        series = stored["series"]
        throttle = np.asarray(series["thr"], dtype=float)
        heading = np.asarray(series["head"], dtype=float)
        if throttle.size != heading.size:
            raise ValueError(f"{key}: throttle and heading sample counts differ")
        baseline_mask = throttle == 1000.0
        baseline_sample_count = int(baseline_mask.sum())
        if baseline_sample_count == 0:
            raise ValueError(f"{key}: no throttle=1000 baseline samples")

        baseline_heading = float(heading[baseline_mask].mean())
        normalized_heading = heading - baseline_heading
        stored_slope = float(stored["slope"])
        panels.append(
            {
                "key": key,
                "title": title,
                "throttle": throttle,
                "normalized_heading": normalized_heading,
                "sample_count": int(throttle.size),
                "baseline_sample_count": baseline_sample_count,
                "baseline_heading": baseline_heading,
                "stored_slope": stored_slope,
                "slope_label": f"{stored_slope:+.2f}°/100µs",
            }
        )

    return {"data_path": chartdata_path, "panels": panels}


def _configure_fonts() -> None:
    """Prefer installed Korean fonts, retaining a portable fallback for CI."""
    plt.rcParams.update(
        {
            "font.family": ["Noto Sans CJK KR", "NanumGothic", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "savefig.facecolor": "white",
            "savefig.edgecolor": "white",
        }
    )


def render_chart(
    output_path: Path | str = DEFAULT_OUTPUT_PATH,
    data_path: Path | str = DEFAULT_DATA_PATH,
) -> Path:
    """Render a deterministic 1280×720 PNG from tracked chartdata.json only."""
    _configure_fonts()
    specification = build_chart_spec(data_path)
    panels = specification["panels"]
    all_changes = np.concatenate([panel["normalized_heading"] for panel in panels])
    minimum = float(all_changes.min())
    maximum = float(all_changes.max())
    padding = max((maximum - minimum) * 0.10, 1.0)
    shared_y_limits = (minimum - padding, maximum + padding)

    figure, axes = plt.subplots(
        1,
        2,
        figsize=(CANVAS_SIZE[0] / 100, CANVAS_SIZE[1] / 100),
        dpi=100,
        sharey=True,
    )
    figure.patch.set_facecolor("white")
    figure.subplots_adjust(
        left=LAYOUT_METADATA["plot_left"],
        right=LAYOUT_METADATA["plot_right"],
        bottom=LAYOUT_METADATA["plot_bottom"],
        top=LAYOUT_METADATA["plot_top"],
        wspace=LAYOUT_METADATA["plot_spacing"],
    )
    figure.text(
        0.5,
        LAYOUT_METADATA["title_y"],
        PRESENTATION_TEXT["title"],
        ha="center",
        va="center",
        fontsize=TYPOGRAPHY["title_pt"],
        fontweight="bold",
        color="#0B315B",
    )
    figure.text(
        0.5,
        LAYOUT_METADATA["normalization_caption_y"],
        PRESENTATION_TEXT["normalization_caption"],
        ha="center",
        va="center",
        fontsize=TYPOGRAPHY["normalization_caption_pt"],
        color="#536577",
    )

    for axis, panel, line_color, point_color in zip(
        axes, panels, PANEL_COLORS, POINT_COLORS
    ):
        throttle = panel["throttle"]
        changes = panel["normalized_heading"]
        regression = np.polyfit(throttle, changes, 1)
        line_x = np.array([float(throttle.min()), float(throttle.max())])
        line_y = np.polyval(regression, line_x)

        axis.set_facecolor("#F7F9FC")
        axis.grid(axis="y", color="#D8E0E8", linewidth=0.9)
        axis.axhline(0, color="#B3C0CC", linewidth=1.0, zorder=1)
        axis.scatter(
            throttle,
            changes,
            s=28,
            color=point_color,
            edgecolors="white",
            linewidths=0.35,
            alpha=0.92,
            label=SEMANTIC_METADATA["point_label"],
            zorder=2,
        )
        axis.plot(
            line_x,
            line_y,
            color=line_color,
            linewidth=3.0,
            label=SEMANTIC_METADATA["line_label"],
            zorder=3,
        )
        axis.set_title(
            panel["title"],
            fontsize=TYPOGRAPHY["panel_title_pt"],
            fontweight="bold",
            color="#173E67",
            pad=14,
        )
        axis.set_xlim(980, 1420)
        axis.set_ylim(shared_y_limits)
        axis.set_xlabel(
            SEMANTIC_METADATA["x_label"],
            fontsize=TYPOGRAPHY["axis_label_pt"],
            labelpad=10,
            color="#23384D",
        )
        axis.tick_params(
            axis="both", labelsize=TYPOGRAPHY["tick_label_pt"], colors="#40566C"
        )
        for spine in ("top", "right"):
            axis.spines[spine].set_visible(False)
        axis.spines["left"].set_color("#AAB8C6")
        axis.spines["bottom"].set_color("#AAB8C6")
        axis.text(
            0.03,
            0.94,
            f"{PRESENTATION_TEXT['slope_heading']}\n{panel['slope_label']}",
            transform=axis.transAxes,
            ha="left",
            va="top",
            fontsize=TYPOGRAPHY["slope_box_pt"],
            fontweight="bold",
            linespacing=1.45,
            color="#163E68",
            bbox={"boxstyle": "round,pad=0.45", "facecolor": "white", "edgecolor": "#C5D2DF"},
        )

    axes[0].set_ylabel(
        SEMANTIC_METADATA["y_label"],
        fontsize=TYPOGRAPHY["axis_label_pt"],
        labelpad=10,
        color="#23384D",
    )
    legend_handles, legend_labels = axes[0].get_legend_handles_labels()
    figure.legend(
        legend_handles,
        legend_labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.135),
        ncol=2,
        frameon=False,
        fontsize=TYPOGRAPHY["legend_pt"],
        handlelength=2.3,
        columnspacing=2.2,
    )
    figure.text(
        0.5,
        0.06,
        PRESENTATION_TEXT["legend_caption"],
        ha="center",
        va="center",
        fontsize=TYPOGRAPHY["footer_pt"],
        color="#536577",
    )

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=100, metadata={"Software": "render_chart_mag.py"})
    plt.close(figure)
    return destination


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA_PATH)
    arguments = parser.parse_args()
    print(render_chart(arguments.output, arguments.data))


if __name__ == "__main__":
    main()
