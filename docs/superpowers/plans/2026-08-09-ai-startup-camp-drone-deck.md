# AI Startup Camp Drone Deck Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct the 77-slide AI startup camp drone deck against the current repository, make it self-contained under `docs/presentations/`, and push the maintained HTML source to GitHub.

**Architecture:** Keep the original UOS HTML slide runtime and its 16:9 visual system, but relocate all runtime dependencies into one repository-owned presentation directory. Update claims in place without changing the slide count, then add a source map and deterministic structural/content checks so future edits can be audited against current firmware and dated measurements.

**Tech Stack:** HTML, CSS, JavaScript, local WOFF2 fonts, MP4, Python 3 standard library for read-only validation, Chromium/Playwright or an available headless browser for rendering checks, Git.

## Global Constraints

- The editable HTML is the source of truth; do not generate PPTX or PDF in this implementation.
- Preserve exactly 77 `<section>` slides and the existing UOS blue/Noto Sans CJK KR/16:9 design.
- Include the video, images, charts, runtime JavaScript, CSS, and fonts required for offline presentation.
- Do not copy `uploads/`, `.thumbnail`, `scratchpad.md`, CSV logs, PDF, or PPTX into the presentation directory.
- Use current firmware and dated correction documents before stale overview prose.
- Keep verified, experimental, unmeasured, and telemetry-only states distinct.
- Do not modify, stage, or commit the user's existing `docs/README.md`, `docs/cascade_vs_single_pid.typ`, or `docs/cascade_vs_single_pid.pdf` changes.
- Stage only the named plan file or files under `docs/presentations/` for implementation commits.
- Push `feat/magcal-ellipsoid-fit` to `origin`; do not merge main or create a PR.

---

### Task 1: Import the self-contained presentation runtime

**Files:**
- Create: `docs/presentations/README.md`
- Create: `docs/presentations/ai-startup-camp-drone/index.html`
- Create: `docs/presentations/ai-startup-camp-drone/deck-stage.js`
- Create: `docs/presentations/ai-startup-camp-drone/support.js`
- Create: `docs/presentations/ai-startup-camp-drone/chartdata.json`
- Create: `docs/presentations/ai-startup-camp-drone/assets/*`
- Create: `docs/presentations/ai-startup-camp-drone/vendor/uos-slide-template/*`

**Interfaces:**
- Consumes: `/tmp/ai-drone-handoff.wcVPb9/ai/project/` from the extracted user handoff.
- Produces: a local `index.html` whose relative asset, runtime, stylesheet, font, and video references resolve inside the presentation directory.

- [ ] **Step 1: Record the failing precondition**

Run:

```bash
test -f docs/presentations/ai-startup-camp-drone/index.html
```

Expected: exit 1 because the repository package does not exist yet.

- [ ] **Step 2: Copy only the approved source and binary assets**

Create the target directories and mechanically copy:

```text
ZETIN Drone 기술 교안.dc.html -> index.html
deck-stage.js                  -> deck-stage.js
support.js                     -> support.js
chartdata.json                 -> chartdata.json
assets/                        -> assets/
_ds/uos-slide-template-*/      -> vendor/uos-slide-template/
```

Do not copy `logs/`, `uploads/`, `.thumbnail`, or `scratchpad.md`.

- [ ] **Step 3: Rewrite runtime paths**

Use exact in-file replacements in `index.html` and vendor CSS/JS so references that started with `_ds/uos-slide-template-5a739b63-48c6-4189-9c30-cab2fcc37300/` resolve to `vendor/uos-slide-template/`. Keep `assets/`, `deck-stage.js`, `support.js`, and `chartdata.json` relative to `index.html`.

- [ ] **Step 4: Run structural validation**

Run a Python standard-library checker that asserts:

```python
assert html.count("<section ") == 77
assert not any(root.rglob("*.csv"))
assert not any(root.rglob("*.pdf"))
assert not any(root.rglob("*.pptx"))
assert not (root / "uploads").exists()
assert max(path.stat().st_size for path in root.rglob("*") if path.is_file()) < 100_000_000
```

Parse local `src=` and `href=` references and fail if any referenced file is missing.

- [ ] **Step 5: Commit the imported runtime**

```bash
git add docs/presentations
git diff --cached --check
git commit -m "docs: import editable AI startup camp drone deck"
```

Expected: only `docs/presentations/` files are committed; existing user changes remain unstaged.

---

### Task 2: Correct sensor, estimator, telemetry, and magnetometer claims

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`

**Interfaces:**
- Consumes: current values and behavior from `dual_imu_cascade_pwm.ino`, `telemetry_schema.py`, `control_dualsense.py`, and `bmm350_yaw_bench_test.md`.
- Produces: corrected slide body copy and speaker notes for slides 27, 31, 32, 37, and 49 through 53.

- [ ] **Step 1: Capture stale-claim RED evidence**

Run `rg` for these exact strings and confirm they are present before editing:

```text
34개 항목
현재 사용값 — α = 0.98
기체 roll 각속도 = − 센서 Y
기체 pitch 각속도 = + 센서 X
data-label="기본값 OFF"
```

- [ ] **Step 2: Update telemetry and estimator slides**

- Slide 27: say the drone sends 65 state fields at 20Hz and records a separate 1kHz raw dual-IMU stream.
- Slide 31: replace the fixed 0.98 split with adaptive α values 0.999, 0.9995, and 0.9998 selected by acceleration deviation.
- Slide 32: show the actual 250Hz magnetometer correction, `K=0.001`, with an approximately four-second time constant.
- Slide 37: use Roll `+sensor Y`, Pitch `-sensor X`, Yaw `-sensor Z`; retain accelerometer `+Y/-X/+Z`.

- [ ] **Step 3: Update magnetometer slides**

- Preserve the dated July 27 `18.3° → 2.4°` and `+3.64 → +0.02°/100µs` results.
- Add the August 4 constrained ellipsoid hard/soft-iron calibration and board/ramp revalidation.
- Retitle slide 53 to an operational-default-ON message. Explain that firmware initializes OFF, while the ground station sends the operator's default ON choice before arming; `--no-mag` or `mag off` disables it.

- [ ] **Step 4: Verify corrected content GREEN**

Run a Python assertion script against `index.html`:

```python
for stale in stale_strings:
    assert stale not in html
for current in ["0.999", "0.9995", "0.9998", "65개", "1kHz", "250Hz", "K = 0.001", "제약 타원체", "기본 ON"]:
    assert current in html
assert html.count("<section ") == 77
```

- [ ] **Step 5: Commit the claim corrections**

```bash
git add docs/presentations/ai-startup-camp-drone/index.html
git diff --cached --check
git commit -m "docs: correct estimator telemetry and mag deck claims"
```

---

### Task 3: Replace the obsolete landing-probe story with measured current behavior

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`

**Interfaces:**
- Consumes: `failsafe_land_research.md`, `power_on_bench_procedure.md`, current failsafe code, and the 3901-L0X telemetry contract.
- Produces: an evidence-ordered `attempt → flight disproof → current timer descent` narrative on slides 57 through 61.

- [ ] **Step 1: Capture obsolete-story RED evidence**

Confirm these strings are present before editing:

```text
고도를 재는 센서는 아직 달지 않았습니다
1.794 → 0.959초
0.011g
지면 접촉 시 0.038~0.051g, 공중에서 0.063~0.139g
벤치에서 프로빙 반응을 반복 측정하며 임계값을 다듬고 있습니다
```

- [ ] **Step 2: Keep the valid safety split on slide 57**

Retain RC-loss controlled descent versus immediate cut for IMU loss, over-tilt, or manual stop. Remove any claim that a validated touchdown detector ends the descent.

- [ ] **Step 3: Rewrite slides 58 through 61**

- Slide 58: introduce the original IMU-only probe hypothesis as a discarded experiment, not a current solution.
- Slide 59: show that the first labels were insufficient and that an actual airborne probe distribution was never measured.
- Slide 60: explain the August 1 real-flight contradiction: ground response did not vanish, so threshold tuning could not restore the premise.
- Slide 61: state the current behavior—probe telemetry only, no touchdown decision, timer-based descent for three seconds, and low-altitude indoor auto-land not trusted.
- Add the 3901-L0X distinction: range/flow is now acquired in fields 60–64 but does not yet influence landing or position control.

- [ ] **Step 4: Verify landing content GREEN**

Assert all obsolete strings are absent and all current concepts exist:

```text
공중 분포는 미측정
프로브는 기록에만 사용
착지 결정에는 사용하지 않음
3초
3901-L0X
제어에는 아직 사용하지 않음
실내 저고도 자동착륙을 신뢰하지 않음
```

Also assert the slide count remains 77.

- [ ] **Step 5: Commit the landing narrative**

```bash
git add docs/presentations/ai-startup-camp-drone/index.html
git diff --cached --check
git commit -m "docs: update deck with measured autoland limitations"
```

---

### Task 4: Update measured performance, status, and next steps

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/index.html`

**Interfaces:**
- Consumes: the bundled `chartdata.json`, the bundled July 27 CSV only as an external audit input, and the August 1 flight record in `power_on_bench_procedure.md`.
- Produces: reproducible historical figures and current status on slides 68, 69, 72, 75, and 76.

- [ ] **Step 1: Preserve validated historical results with their scope**

Keep slide 68 at 106.4 seconds, Roll σ=1.65°, Pitch σ=1.23°, and label it explicitly as a July 27 tethered segment rather than stable-flight completion.

- [ ] **Step 2: Make motor differential reproducible**

For the full 2,135-row chart interval, display M1 approximately 1334.2µs, M3 approximately 1360.0µs, and the rounded difference `약 26µs`. Replace the center-of-gravity proof claim with a persistent control bias consistent with mass, thrust-train, tether, frame, or aerodynamic asymmetry.

- [ ] **Step 3: Update the telemetry exercise**

Change slide 72 from `34개 항목 중 일부` to `65개 필드 중 일부`. Keep the example's learning goal but remove the claim that the numbers uniquely locate the center of gravity.

- [ ] **Step 4: Update the status and roadmap**

- Slide 75: add the 176.2-second August 1 real-flight record, zero fault flags, PID average 1000Hz/minimum 999Hz, magnetometer ON, and yaw-hold lock 91.9%.
- Keep stable hover and reliable auto-land incomplete; do not call the flight untethered because the source does not establish that.
- Move 3901-L0X integration to implemented/observable but not control-validated.
- Slide 76: replace `add optical flow and altitude sensor` with `use and validate the already connected range/flow measurements for control`.
- Update the deck date from 2026.07 to 2026.08.

- [ ] **Step 5: Verify status content GREEN**

Assert:

```python
assert "176.2초" in html
assert "91.9%" in html
assert "약 26µs" in html
assert "1334.2µs" in html and "1360.0µs" in html
assert "34개 항목" not in html
assert "센서 미탑재" not in html
assert "바닥을 보는 카메라(옵티컬 플로우)와 고도 센서를 추가" not in html
assert html.count("<section ") == 77
```

- [ ] **Step 6: Commit status updates**

```bash
git add docs/presentations/ai-startup-camp-drone/index.html
git diff --cached --check
git commit -m "docs: refresh drone deck flight status and roadmap"
```

---

### Task 5: Add usage and evidence documentation

**Files:**
- Modify: `docs/presentations/README.md`
- Create: `docs/presentations/ai-startup-camp-drone/README.md`
- Create: `docs/presentations/ai-startup-camp-drone/SOURCES.md`

**Interfaces:**
- Consumes: the final slide numbers, repository source paths, and local presentation runtime.
- Produces: an entry point for repository readers, exact preview instructions, export policy, and a claim-to-evidence map.

- [ ] **Step 1: Write the presentation catalog**

`docs/presentations/README.md` names the deck, audience, source-of-truth policy, and current `index.html` entry point.

- [ ] **Step 2: Write the deck README**

Document:

```bash
cd docs/presentations/ai-startup-camp-drone
/home/light/anaconda3/bin/python -m http.server 8000
```

Then open `http://127.0.0.1:8000/`. Explain keyboard navigation, video inclusion, edit workflow, and that PPTX/PDF exports are intentionally deferred until content freeze.

- [ ] **Step 3: Write SOURCES.md**

Map slide groups to exact repository files and evidence dates. Include the axis transform, alpha values, telemetry schema, magnetometer calibration, landing-probe correction, 3901-L0X telemetry-only status, July 27 tethered chart, August 1 flight record, and maturity boundary.

- [ ] **Step 4: Check documentation links**

Resolve every relative Markdown link from its containing document and fail if any repository target is missing. Scan for unfinished placeholder markers or ambiguous `검증 완료` statements.

- [ ] **Step 5: Commit documentation**

```bash
git add docs/presentations/README.md \
  docs/presentations/ai-startup-camp-drone/README.md \
  docs/presentations/ai-startup-camp-drone/SOURCES.md
git diff --cached --check
git commit -m "docs: document AI startup camp deck sources and workflow"
```

---

### Task 6: Verify rendering, video, content, and Git scope

**Files:**
- Verify: `docs/presentations/ai-startup-camp-drone/**`

**Interfaces:**
- Consumes: the complete self-contained deck package.
- Produces: fresh structural, content, media, and visual evidence for the final report.

- [ ] **Step 1: Run the full structural/content checker**

Run one fresh Python command that checks all required and forbidden strings, 77 sections, all local `src`/`href` paths, forbidden extensions/directories, and the 100MB file limit. Expected: every named check prints PASS and exits 0.

- [ ] **Step 2: Verify the video stream**

```bash
ffprobe -v error -select_streams v:0 \
  -show_entries stream=codec_name,width,height,duration \
  -of default=noprint_wrappers=1 \
  docs/presentations/ai-startup-camp-drone/assets/hover_demo.mp4
```

Expected: one readable video stream with nonzero dimensions and duration.

- [ ] **Step 3: Render targeted slides**

Serve the deck locally and use an available headless Chromium/Playwright path to capture slides 1, 31, 37, 53, 58, 59, 60, 61, 68, 69, 75, and 76. If the runtime uses a slide query or keyboard navigation, use that supported path rather than editing source for screenshots.

- [ ] **Step 4: Inspect visual output**

Inspect a contact sheet or the individual screenshots for missing fonts/images, clipping, overlap, undersized copy, incorrect slide titles, and stale visible text. Correct only presentation-local files, then rerun Steps 1 through 3.

- [ ] **Step 5: Verify repository cleanliness boundaries**

```bash
git status --short
git diff --check
git log --oneline -8
```

Expected: only the pre-existing `docs/README.md`, `docs/cascade_vs_single_pid.typ`, and `docs/cascade_vs_single_pid.pdf` remain uncommitted; presentation work is committed.

---

### Task 7: Push the versioned deck to GitHub

**Files:**
- No file changes expected.

**Interfaces:**
- Consumes: verified local commits on `feat/magcal-ellipsoid-fit`.
- Produces: an upstream GitHub branch containing the latest magnetometer commits, design/plan, and maintained presentation source.

- [ ] **Step 1: Confirm the exact push target**

```bash
git status --short --branch
git branch --show-current
git log --oneline origin/main..HEAD
```

Expected branch: `feat/magcal-ellipsoid-fit`. Confirm the uncommitted files are only the pre-existing user changes.

- [ ] **Step 2: Push and set upstream**

```bash
git push -u origin feat/magcal-ellipsoid-fit
```

- [ ] **Step 3: Verify remote parity**

```bash
git fetch origin feat/magcal-ellipsoid-fit
git rev-parse HEAD
git rev-parse origin/feat/magcal-ellipsoid-fit
git rev-list --left-right --count HEAD...origin/feat/magcal-ellipsoid-fit
```

Expected: local and remote hashes match and the left/right count is `0 0`.
