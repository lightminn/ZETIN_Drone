# Presentation Technical-First Business Framing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the 77-slide technical presentation while adding concise education-business context to the opening, hardware execution story, and multi-drone roadmap.

**Architecture:** `index.html` remains the editable source of truth. A small standard-library Python test locks the four-part technical structure, required business-context copy, procurement status, and the unverified status of the multi-drone roadmap; the existing exporter then regenerates and independently validates the PPTX.

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

### Task 1: Lock the technical-first presentation contract

**Files:**
- Create: `tools/test_presentation_business_framing.py`
- Test: `tools/test_presentation_business_framing.py`

**Interfaces:**
- Consumes: `docs/presentations/ai-startup-camp-drone/index.html`.
- Produces: assertions for slide count, chapter order, business-to-technical handoff, PCB execution status, and the unverified multi-drone roadmap.

- [ ] **Step 1: Write the failing contract test**

Create a standard-library `unittest.TestCase`. Extract `<section>...</section>`
blocks with `re.compile(r"<section\\b([^>]*)>(.*?)</section>", re.DOTALL)`, strip
tags with `HTMLParser`, and index slides by `data-screen-label`. Assert:

```python
self.assertEqual(len(slides), 77)
self.assertIn("의의", slides["04"])
self.assertIn("하드웨어", slides["14"])
self.assertIn("소프트웨어", slides["26"])
self.assertIn("시연", slides["65"])
self.assertIn("이후부터는 기술이 본론", slides["11"])
self.assertIn("교육 모듈의 시제품", slides["12"])
for phrase in ("주문 진행", "납땜", "알리익스프레스"):
    self.assertIn(phrase, slides["23"])
for phrase in ("sim-to-real", "경로 계획", "군집 제어", "아직 검증하지 않음"):
    self.assertIn(phrase, slides["76"])
self.assertIn("경로 계획 · 군집 제어", slides["75"])
self.assertNotIn("투자설명회", html)
self.assertNotRegex(html, r"예상 매출|투자 요청|시장 규모\\s*[:：]\\s*\\d")
```

- [ ] **Step 2: Run the test to verify RED**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-red \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_business_framing -v
```

Expected: FAIL because slides 11, 23, and 76 do not yet contain the required
technical-first handoff, procurement status, and explicit roadmap boundary.

---

### Task 2: Revise the source deck and evidence trail

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/BRIEF.md`
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone/SOURCES.md`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`
- Test: `tools/test_presentation_business_framing.py`

**Interfaces:**
- Consumes: the user-provided 2026-08-10 presentation brief and the existing 77-slide source.
- Produces: the same HTML deck structure with revised slide copy and an auditable source for current execution status.

- [ ] **Step 1: Preserve the presentation brief**

Write `BRIEF.md` with two sections: the original direction bullets and the final
interpretation that technology is primary and business context is secondary.
Record `PCB 주문 진행`, `납땜`, and `알리익스프레스 부품 납기` as stakeholder-
provided current status rather than repository-verified technical evidence.

- [ ] **Step 2: Update slides 1, 11, and 12**

Change slide 1 subtitle to `AI 창업 캠프 · 기술 발표 및 체험 · 3시간 과정`.
On slide 11, add `이후부터는 기술이 본론` to the speaker note and frame
business value as reproducible design, failure, measurement, and correction.
On slide 12, describe the presentation and exercises as an education-module
prototype without claiming a finished product or revenue.

- [ ] **Step 3: Update slide 23 procurement status**

Keep the board-design lesson and add one compact status callout:

```html
<div style="background:#eaf2fb;border-radius:4px;padding:18px 20px;font-size:18.667px;line-height:1.45">
  <strong>현재 실행 상태</strong> · 차기 PCB는 주문 진행 중입니다.
  납땜 작업과 알리익스프레스 부품 납기는 일정 위험으로 관리하고 있습니다.
</div>
```

Update the speaker note so the status illustrates real hardware execution rather
than functioning as an excuse for technical maturity.

- [ ] **Step 4: Update slides 76 and 77**

Rename slide 76 step 3 to `다중 드론 sim-to-real`, describe simulation-validated
path planning transferred to multiple real drones and swarm control, and label
it `현재 상태 — 아직 검증하지 않음`. Keep single-drone hover and optical-flow
control as prerequisite steps. Give slide 77 the subtitle
`ZETIN Drone · 비행제어 기술 · 실습 · 교육 확장`.

- [ ] **Step 5: Update source documentation**

Link `BRIEF.md` from `README.md`. Add a `SOURCES.md` row for slides 1, 11, 12,
23, 76, and 77 that identifies the brief as stakeholder direction/current status
and explicitly says it is not flight-verification evidence.

- [ ] **Step 6: Run focused GREEN checks**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-green \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_business_framing \
  tools.test_presentation_video_autoplay -v
```

Expected: the framing contract and all video-autoplay tests pass.

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

- [ ] **Step 1: Generate an unlocked candidate**

If `.~lock.ZETIN_Drone_AI_Startup_Camp.pptx#` exists and an OnlyOffice process
has the canonical path open, run:

```bash
node docs/presentations/ai-startup-camp-drone/export_pptx.cjs \
  --output /tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx
```

Otherwise generate the canonical path directly. Never terminate the user's
OnlyOffice process.

- [ ] **Step 2: Make the artifact test accept an explicit candidate**

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

- [ ] **Step 3: Validate the candidate structure**

```bash
ZETIN_PRESENTATION_PPTX=/tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx \
PYTHONPYCACHEPREFIX=/tmp/zetin-business-pptx \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_presentation_pptx_export -v
unzip -t /tmp/ZETIN_Drone_AI_Startup_Camp_candidate.pptx
```

- [ ] **Step 4: Inspect representative pages**

Convert the candidate to PDF with an isolated LibreOffice profile, require 77
pages, and render pages 1, 11, 12, 23, 76, and 77. Inspect for clipping, excessive
business copy, and an intact technical visual hierarchy.

- [ ] **Step 5: Replace the canonical artifact only when unlocked**

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

- [ ] **Step 1: Run the complete repository suite**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-business-full \
MPLCONFIGDIR=/tmp/zetin-business-full-mpl \
  /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
```

- [ ] **Step 2: Check scope and whitespace**

Run `git diff --check` and inspect `git status --short`. Confirm that
`docs/cascade_vs_single_pid.pdf` and the OnlyOffice lock are untracked and
unstaged.

- [ ] **Step 3: Commit and push scoped files**

Stage only `BRIEF.md`, `README.md`, `SOURCES.md`, `index.html`, the framing test,
this plan, and the canonical PPTX if it was safely regenerated. Commit with
`feat: refocus startup camp deck on technical execution`, push without force,
fetch, and require local/remote divergence `0 0`.
