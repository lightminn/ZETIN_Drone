# Presentation Technical-First Business Framing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the 77-slide technical presentation while adding concise education-business context to the opening, hardware execution story, and multi-drone roadmap.

**Architecture:** `index.html` remains the editable source of truth. `BRIEF.md` and `SOURCES.md` preserve the editorial intent and evidence boundary without brittle prose-string tests; the existing browser, exporter, and artifact tests verify playback behavior, slide rendering, notes, and embedded media.

**Tech Stack:** HTML custom elements, Python `unittest` and `html.parser`, Node.js, PptxGenJS, Chrome DevTools Protocol, FFmpeg/FFprobe, LibreOffice.

## Global Constraints

- Keep exactly 77 HTML/PPTX slides, 77 speaker notes, and 10 embedded H.264 videos.
- Keep the main order `의의 → 하드웨어 → 소프트웨어 → 시연·실습`.
- Keep technical problem solving and measured evidence as the main content.
- Do not add market sizing, revenue forecasts, investment requests, or claims of a finished education business.
- State that multi-drone sim-to-real path planning and swarm control are unverified expansion goals.
- Preserve the verified/in-progress/unverified boundary on slide 75.
- Do not overwrite the canonical PPTX while an OnlyOffice lock file exists.
- Do not stage or modify `docs/cascade_vs_single_pid.pdf` or the OnlyOffice lock file.

---

### Task 1: Capture the presentation baseline

**Files:**
- Test: `tools/test_presentation_video_autoplay.py`
- Test: `tools/test_presentation_pptx_export.py`

**Interfaces:**
- Consumes: the current HTML and canonical PPTX.
- Produces: baseline evidence that 10 videos autoplay in HTML and that the canonical artifact contains 77 slides, 77 notes, and 10 H.264 videos before editorial changes.

- [x] **Step 1: Run the existing HTML playback tests**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-baseline \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_video_autoplay -v
```

Expected: 3 tests pass.

- [x] **Step 2: Run the existing PPTX artifact test**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-baseline-pptx \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_pptx_export -v
```

Expected: 1 test passes for the pre-change canonical artifact.

---

### Task 2: Revise the source deck and evidence trail

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/BRIEF.md`
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone/SOURCES.md`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`
- Test: `tools/test_presentation_video_autoplay.py`

**Interfaces:**
- Consumes: the user-provided 2026-08-10 presentation brief and the existing 77-slide source.
- Produces: the same HTML deck structure with revised slide copy and an auditable source for current execution status.

- [x] **Step 1: Preserve the presentation brief**

Write `BRIEF.md` with two sections: the original direction bullets and the final
interpretation that technology is primary and business context is secondary.
Record `PCB 주문 진행`, `납땜`, and `알리익스프레스 부품 납기` as stakeholder-
provided current status rather than repository-verified technical evidence.

- [x] **Step 2: Update slides 1, 11, and 12**

Change slide 1 subtitle to `AI 창업 캠프 · 기술 발표 및 체험 · 3시간 과정`.
On slide 11, add `이후부터는 기술이 본론` to the speaker note and frame
business value as reproducible design, failure, measurement, and correction.
On slide 12, describe the presentation and exercises as an education-module
prototype without claiming a finished product or revenue.

- [x] **Step 3: Update slide 23 procurement status**

Keep the board-design lesson and add one compact status callout:

```html
<div style="background:#eaf2fb;border-radius:4px;padding:18px 20px;font-size:18.667px;line-height:1.45">
  <strong>현재 실행 상태</strong> · 차기 PCB는 주문 진행 중입니다.
  납땜 작업과 알리 부품 납기는 일정 위험으로 관리하고 있습니다.
</div>
```

Update the speaker note so the status illustrates real hardware execution rather
than functioning as an excuse for technical maturity.

- [x] **Step 4: Update slides 76 and 77**

Rename slide 76 step 3 to `다중 드론 sim-to-real`, describe simulation-validated
path planning transferred to multiple real drones and swarm control, and label
it `현재 상태 — 아직 검증하지 않음`. Keep single-drone hover and optical-flow
control as prerequisite steps. Give slide 77 the subtitle
`ZETIN Drone · 비행제어 기술 · 실습 · 교육 확장`.

- [x] **Step 5: Update source documentation**

Link `BRIEF.md` from `README.md`. Add a `SOURCES.md` row for slides 1, 11, 12,
23, 76, and 77 that identifies the brief as stakeholder direction/current status
and explicitly says it is not flight-verification evidence.

- [x] **Step 6: Review the editorial checklist**

Read slides 1, 11, 12, 23, 75, 76, and 77 against `BRIEF.md` and `SOURCES.md`.
Confirm that technology remains the main presentation, that the business context
is confined to the opening/closing bridge, and that the multi-drone goal is
explicitly unverified.

- [x] **Step 7: Run focused regression checks**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-green \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_video_autoplay -v
```

Expected: all video-autoplay tests still pass after the editorial changes.

---

### Task 3: Regenerate and validate the PPTX safely

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/ZETIN_Drone_AI_Startup_Camp.pptx`
- Modify: `docs/superpowers/plans/2026-08-10-presentation-technical-first-business-framing.md`
- Modify: `tools/test_presentation_pptx_export.py`
- Test: `tools/test_presentation_pptx_export.py`

**Interfaces:**
- Consumes: revised `index.html` and existing local assets.
- Produces: a visually validated 77-slide PPTX with 77 notes and 10 embedded H.264 videos.

- [x] **Step 1: Generate an unlocked candidate**

If `.~lock.ZETIN_Drone_AI_Startup_Camp.pptx#` exists and an OnlyOffice process
has the canonical path open, run:

```bash
node docs/presentations/ai-startup-camp-drone/export_pptx.cjs \
  --output /tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx
```

Otherwise generate the canonical path directly. Never terminate the user's
OnlyOffice process.

- [x] **Step 2: Make the artifact test accept an explicit candidate**

Import `os` and replace the fixed constant with:

```python
DEFAULT_PPTX_PATH = (
    REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone"
    / "ZETIN_Drone_AI_Startup_Camp.pptx"
)
PPTX_PATH = Path(os.environ.get("ZETIN_PRESENTATION_PPTX", DEFAULT_PPTX_PATH))
```

Run with `ZETIN_PRESENTATION_PPTX=/tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx`
and require 77 slides, 77 notes, 10 H.264 videos, and ZIP integrity.

- [x] **Step 3: Validate the candidate structure**

```bash
ZETIN_PRESENTATION_PPTX=/tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx \
PYTHONPYCACHEPREFIX=/tmp/zetin-business-pptx \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_pptx_export -v
unzip -t /tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx
```

- [x] **Step 4: Inspect representative pages**

Convert the candidate to PDF with an isolated LibreOffice profile, require 77
pages, and render pages 1, 11, 12, 23, 76, and 77. Inspect for clipping, excessive
business copy, and an intact technical visual hierarchy.

- [x] **Step 5: Replace the canonical artifact only when unlocked**

Recheck the lock and process. If unlocked, regenerate the canonical PPTX and run
the fixed-path artifact test. If still locked, keep the candidate in `/tmp`, do
not commit a stale canonical PPTX, and report the exact blocker without deleting
or overwriting the user's open file.

---

### Task 4: Complete verification and version control

**Files:**
- Modify: `docs/superpowers/plans/2026-08-10-presentation-technical-first-business-framing.md`

**Interfaces:**
- Consumes: all scoped source, test, and regenerated artifact changes.
- Produces: verified commits pushed to `feat/magcal-ellipsoid-fit` without staging unrelated files.

- [x] **Step 1: Run the complete repository suite**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-full \
MPLCONFIGDIR=/tmp/zetin-business-full-mpl \
  /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
```

- [x] **Step 2: Check scope and whitespace**

Run `git diff --check` and inspect `git status --short`. Confirm that
`docs/cascade_vs_single_pid.pdf` and the OnlyOffice lock are untracked and
unstaged.

- [ ] **Step 3: Commit and push scoped files**

Stage only `BRIEF.md`, `README.md`, `SOURCES.md`, `index.html`, the updated PPTX
artifact test, this plan, the design spec, and the canonical PPTX if it was safely regenerated. Commit with
`feat: refocus startup camp deck on technical execution`, push without force,
fetch, and require local/remote divergence `0 0`.
