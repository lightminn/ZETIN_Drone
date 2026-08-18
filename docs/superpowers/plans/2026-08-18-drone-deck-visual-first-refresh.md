# Drone Deck Visual-First Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the 84-slide drone deck from text-card-heavy exposition to an evidence-led visual presentation while preserving technical claims and slide order.

**Architecture:** Keep `index.html` as the editable source and add self-contained SVG assets under the existing `assets/` directory. Browser regression tests validate visual loading, motion-mode coverage, clipping, word wrapping, and the unchanged video contract. Generated PPTX/PDF files remain untouched until the user explicitly requests regeneration.

**Tech Stack:** HTML/CSS, standalone SVG, local Noto Sans CJK KR fonts, Chrome DevTools Protocol, Python `unittest`.

**Spec:** `docs/superpowers/specs/2026-08-18-drone-deck-visual-first-design.md`

## Global Constraints

- Preserve exactly 84 slides and the current four-part presentation order.
- Preserve all evidence boundaries in `SOURCES.md`; do not describe host/SIL evidence as bench or flight evidence.
- Use original local SVG/video/photo assets; no network dependency at presentation time.
- Minimum SVG text size is 24px and all Korean words must remain unbroken.
- Do not regenerate PPTX or PDF in this task.

---

### Task 1: Lock the visual contracts with browser tests

**Files:**
- Modify: `tools/test_presentation_video_autoplay.py`

**Interfaces:**
- Consumes: `deck-stage._slides`, `img.complete`, `img.naturalWidth`, SVG `data-*` markers.
- Produces: browser tests for slides 4~11 and the full-deck render contract.

- [ ] **Step 1: Write the failing test**

  Add a test that opens the deck and checks slides 4, 5, 6, 7, and 11 for a loaded
  `img[data-visual-first]` with logical width at least 1040px. Check slide 9 for
  `data-state="climb|hover|descend|translate"` and slide 10 for the existing
  helicopter and quadcopter torque markers.

- [ ] **Step 2: Run the test to verify it fails**

  Run:
  `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_video_autoplay.PresentationVideoBrowserTests.test_professor_feedback_slides_are_visual_first -v`

  Expected: FAIL because slides 4~7 and 11 do not yet load the new SVG assets and
  slide 9 lacks the four motion-mode markers.

- [ ] **Step 3: Keep the existing render safety tests in the same suite**

  The new test must exercise the actual browser DOM. Do not replace it with source-text
  assertions. Existing clipping, Korean token wrapping, autoplay, and 84-slide tests remain.

### Task 2: Replace professor-feedback text grids with original SVG diagrams

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/assets/drone-classification-visual.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/qualification-weight-visual.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/aircraft-uam-visual.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/mission-specs-visual.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/swarm-system-simple.svg`
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`

**Interfaces:**
- Consumes: UOS colors, Noto Sans CJK KR, claims mapped in `SOURCES.md`.
- Produces: five offline 1180px-wide diagrams loaded through `img[data-visual-first]`.

- [ ] **Step 1: Create and XML-validate each SVG**

  Use explicit view boxes, accessible `aria-label` text, and semantic markers for
  qualification bands, aircraft types, mission types, and swarm layers.

- [ ] **Step 2: Replace the HTML grids on slides 4~7 and 11**

  Keep one short headline or evidence-boundary caption around each SVG. Remove repeated
  prose already encoded in the diagram. Keep all speaker notes factual and complete.

- [ ] **Step 3: Run the focused browser test**

  Expected: all five assets load at the intended size and the slide count stays 84.

### Task 3: Make the force and motion explanation explicit

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`

**Interfaces:**
- Consumes: the current correct X-geometry force SVG and slide 10 torque comparison.
- Produces: four compact motion conditions with `data-state` markers.

- [ ] **Step 1: Add climb, hover, descend, and translation conditions to slide 9**

  Use `TΣ > mg`, `TΣ = mg`, `TΣ < mg`, and tilted `TΣ` with a horizontal component.
  Label rotation as a handoff to the reaction-torque diagram on slide 10.

- [ ] **Step 2: Preserve the existing slide 9 geometry tests**

  The four rotors must retain both horizontal and vertical separation and all four thrust
  arrows. Slide 10 retains four motor markers and the helicopter tail-force markers.

### Task 4: Reduce text density with evidence and simple diagrams

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/assets/attitude-correction-simple.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/sil-closed-loop-simple.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/failsafe-timeline-simple.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/landing-observability-simple.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/shared-state-race-simple.svg`
- Create: `docs/presentations/ai-startup-camp-drone/assets/telemetry-motor-balance-simple.svg`
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`

**Interfaces:**
- Consumes: current photos, Python-rendered MP4 files, bench charts, SIL/code evidence, failsafe state and validation boundaries.
- Produces: an evidence collage on slide 8, the existing timing video on slide 33,
  the existing magnetometer chart on slide 57, and simple diagrams on slides 12,
  46, 63, 64, 71, and 81.

- [ ] **Step 1: Create the six one-purpose diagrams**

  Each diagram uses at most six major labels. Attitude correction shows disturbance→tilt→motor
  correction; SIL shows physics→sensor→actual controller→motor loop; failsafe shows RC timeout→
  controlled descent→configured upper-limit stop with a separate immediate-cut branch; landing
  observability compares rest and constant descent; the race diagram shows two cores touching one
  shared state; telemetry places the aggregate M1/M3 balance relation on the X-frame without
  presenting a synthesized raw-log row.

- [ ] **Step 2: Integrate evidence first and remove duplicated cards**

  Build slide 8 from `image5.png`, `image12.png`, `chart_attitude.png`, and
  `mobile-lab-student.png`. Move `cascade-loop-timing.mp4` to slide 33 and replace slide 52's
  timing explanation with a simple static loop treatment so the total number of videos stays 14.
  Put `chart_mag.png` on slide 57. Keep only captions that identify what to notice.

- [ ] **Step 3: Verify source claims**

  Compare each diagram label with current firmware/docs and update `SOURCES.md` only where
  a new slide-to-source mapping is needed.

### Task 5: Full render, mutation, and repository verification

**Files:**
- Modify if needed: `docs/presentations/ai-startup-camp-drone/BRIEF.md`
- Modify if needed: `docs/presentations/ai-startup-camp-drone/SOURCES.md`

**Interfaces:**
- Consumes: final HTML, SVGs, videos, photos, and browser tests.
- Produces: a verified HTML source commit with no generated PPTX/PDF changes.

- [ ] **Step 1: Run focused and full presentation tests**

  Run:
  `/home/light/anaconda3/bin/python -m unittest tools.test_presentation_video_autoplay -v`

- [ ] **Step 2: Run the negative control**

  Temporarily remove one `data-visual-first` marker in a copy of the HTML and confirm the
  focused test fails for the expected missing visual. Restore the original source.

- [ ] **Step 3: Capture all 84 slides and inspect contact sheets**

  Check text clipping, word splitting, diagram readability, and media prominence at 1280×720.

- [ ] **Step 4: Run structural checks**

  Run `git diff --check`, validate all new SVG XML, and confirm exactly 84 sections and 14 videos.

- [ ] **Step 5: Commit and push only the task files**

  Preserve unrelated deleted and untracked workspace files. Use a presentation-focused commit
  message and verify local HEAD equals the remote branch after push.

### Task 6: Naturalize all slide titles and ambiguous gain wording

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`
- Modify: `docs/presentations/ai-startup-camp-drone/BRIEF.md`

**Interfaces:**
- Consumes: all 84 slide labels/titles and current firmware gain declarations.
- Produces: short noun-phrase titles and evidence-bounded Roll/Pitch integral-gain wording.

- [ ] **Step 1: Replace metaphorical and promotional titles**

  Use direct titles such as `드론의 법적 분류`, `최대이륙중량과 조종자 증명`,
  `비행체별 장단점과 UAM`, `SIL 폐루프 구조`, and `RC 두절 Failsafe`.

- [ ] **Step 2: Correct the Ki wording**

  Explain that host SIL showed a long-lived steady-state error while the integral limit still had
  margin, so only the Roll/Pitch candidate was adjusted conservatively and Yaw was left unchanged.
  State that no fixed multiplier is a universal answer and that the candidate remains a starting
  point for tethered flight tuning. Keep exact before/after constants in `SOURCES.md` and code, not
  in the audience-facing slide.

- [ ] **Step 3: Read every rendered title at 1280×720**

  Confirm titles fit without splitting Korean words and that explanatory lines end as noun phrases
  unless they are complete sentences ending in `~이다` style.
