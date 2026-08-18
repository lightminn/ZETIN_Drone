# Drone Deck Python Static Diagram Implementation Plan

> **For agentic workers:** Use `superpowers:subagent-driven-development` and execute each
> task with RED→GREEN evidence and an independent review.

**Goal:** Replace the interim 13 explanatory animations with simultaneous-comparison PNGs
rendered by Python/Manim, while retaining the original 14 instructional and demonstration videos.

**Architecture:** A dedicated static Manim module renders 13 1280×720 PNG assets. The live HTML
uses one responsive image per mapped slide. Existing video lifecycle code remains responsible only
for the original 14 videos. Browser and media tests verify the real DOM and generated files;
PPTX/PDF remain untouched.

**Spec:** `docs/superpowers/specs/2026-08-19-drone-deck-manim-diagrams-design.md`

## Global constraints

- Preserve exactly 84 slides and the current order.
- Convert slides 4, 5, 6, 7, 9, 10, 11, 12, 46, 63, 64, 71, 81 to Python PNGs.
- Preserve `mobile-lab-qr.svg` and the existing 14 videos.
- Use Noto Sans CJK KR with major labels at least 26px.
- Preserve legal and evidence boundaries from `SOURCES.md`.
- Do not regenerate or modify PPTX/PDF deliverables.
- Preserve unrelated deleted and untracked workspace files.

### Task 1: Build and render 13 static Manim scenes

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/visualizations/static_diagram_visualizations.py`
- Modify: `docs/presentations/ai-startup-camp-drone/visualizations/render_visualizations.sh`
- Modify: `tools/test_presentation_visualizations.py`
- Create: 13 same-stem PNGs under `docs/presentations/ai-startup-camp-drone/assets/`

- [ ] Add literal class→PNG mappings and fail first because the module/assets are absent.
- [ ] Implement all 13 approved simultaneous-comparison scenes without internal slide titles.
- [ ] Render PNGs at 1280×720 and assert dimension, color mode, nonblank pixels, text floor,
      no concrete dates, and no commit hashes.
- [ ] Inspect a contact sheet and full-size images; correct overlaps, cropped text, inconsistent
      geometry, and sequential-animation remnants.
- [ ] Run focused GREEN and a content mutation that must fail, then restore.

### Task 2: Integrate PNGs and remove interim explanatory MP4s

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`
- Modify: `tools/test_presentation_video_autoplay.py`
- Modify: `tools/test_presentation_pptx_export.py`
- Delete: the 13 interim same-stem MP4 files

- [ ] Change browser/markup contracts first to expect 13 mapped `img[data-python-static]`,
      exactly 14 videos, loaded 1280×720 images, and logical width at least 1040px; record RED.
- [ ] Replace the 13 `<video data-python-visual>` elements with accessible static images.
- [ ] Remove only the 13 interim explanatory MP4 files; preserve all original video assets.
- [ ] Run the full browser module and verify original video autoplay/reset still works.
- [ ] Mutate one mapped image source in a temporary copy and confirm the contract fails; restore.

### Task 3: Document and verify the complete deck

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`
- Modify: `docs/presentations/ai-startup-camp-drone/BRIEF.md`
- Modify: `docs/presentations/ai-startup-camp-drone/SOURCES.md`
- Modify: `docs/presentations/ai-startup-camp-drone/visualizations/README.md`
- Modify: this design and plan only where final implementation differs

- [ ] Document 9 reproducible Python videos, 13 Python PNGs, and 14 linked deck videos.
- [ ] Run all presentation source/browser/visualization/page/launcher/10-minute tests.
- [ ] Capture all 84 slides at 1280×720 and inspect all 13 changed slides at full size.
- [ ] Confirm 84 slides, 14 videos, 13 mapped PNGs, preserved QR SVG, absent interim MP4s,
      clean `git diff --check`, and no PPTX/PDF modification.
- [ ] Record any unrun artifact checks explicitly; do not claim stale PPTX/PDF as current.

## Interim history

Commits `bc04d1e` through `646c24d` established and validated the technical meanings of the
13 diagrams as animations. The user then selected static simultaneous-comparison images because
sequential playback made category comparison harder. This plan supersedes only their final media
format and HTML integration, not the verified technical content.
