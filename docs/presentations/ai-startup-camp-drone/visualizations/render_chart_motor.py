#!/usr/bin/env python3
"""Render the audience-facing M1/M3 tether-output chart.

The plotted lines use the 0.4 s display samples in chartdata.json. Exact means
for the complete 2,135-row interval belong in the slide copy, not this legend.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib import font_manager


DECK_DIR = Path(__file__).resolve().parents[1]
CHART_DATA = DECK_DIR / "chartdata.json"
OUTPUT = DECK_DIR / "assets" / "chart_motor.png"
FONT_PATH = Path("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc")


def render() -> None:
    hover = json.loads(CHART_DATA.read_text(encoding="utf-8"))["hover"]
    series = hover["series"]

    if not (len(series["t"]) == len(series["m1"]) == len(series["m3"])):
        raise ValueError("chartdata hover series lengths do not match")

    if FONT_PATH.exists():
        family = font_manager.FontProperties(fname=FONT_PATH).get_name()
        plt.rcParams["font.family"] = family
    plt.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=100)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    ax.plot(
        series["t"],
        series["m1"],
        color="#0050A4",
        linewidth=2.8,
        label="M1 · 전-좌",
    )
    ax.plot(
        series["t"],
        series["m3"],
        color="#8F98A3",
        linewidth=2.8,
        label="M3 · 전-우",
    )

    ax.set_xlim(min(series["t"]), max(series["t"]))
    ax.set_ylim(1050, 1475)
    ax.set_xticks([0, 20, 40, 60, 80, 100])
    ax.set_yticks([1100, 1200, 1300, 1400])
    ax.grid(axis="y", color="#D6D3CE", linewidth=1.4)
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color("#7C8793")
    ax.spines[["left", "bottom"]].set_linewidth(1.4)
    ax.tick_params(axis="both", colors="#5F6770", labelsize=22, length=0, pad=10)
    ax.set_ylabel("모터 출력 (µs)", fontsize=26, color="#5F6770", labelpad=18)
    ax.set_xlabel("시간 (초)", fontsize=26, color="#5F6770", labelpad=14, loc="right")
    ax.legend(
        loc="upper right",
        ncol=2,
        frameon=False,
        fontsize=25,
        handlelength=2.2,
        columnspacing=1.8,
    )

    fig.subplots_adjust(left=0.11, right=0.975, top=0.89, bottom=0.17)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT, dpi=100, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    render()
