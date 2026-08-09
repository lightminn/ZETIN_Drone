# AI Startup Camp Mobile Drone Lab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Korean, mobile-first, installation-free educational drone simulation with sensor/touch IMU exploration, a deterministic hovering challenge, optional local leaderboard, and presenter QR page.

**Architecture:** A static HTML/CSS/ES-module student app performs all sensor conversion, simulation, and scoring in the browser. A separate presenter page polls an optional same-origin Python standard-library server that serves static files and stores validated scores in memory; all student flows remain complete when that API is unavailable.

**Tech Stack:** Semantic HTML, CSS, browser ES modules, Node 22 built-in test runner, Python 3.11 standard library, Python `unittest`, Chrome DevTools Protocol through the repository's existing `websocket-client` environment.

## Global Constraints

- Work only under `docs/presentations/ai-startup-camp-drone/mobile-lab/`, new task-specific files under `tools/`, and this task's design/plan documents.
- Do not modify or regenerate `docs/presentations/ai-startup-camp-drone/index.html` or `ZETIN_Drone_AI_Startup_Camp.pptx`.
- Do not modify, delete, stage, or commit `docs/cascade_vs_single_pid.pdf`.
- Runtime core behavior must remain static-client-side and require no framework, build system, login, account, CDN, or external service.
- Student code must not use UDP, serial, WebUSB, WebBluetooth, WebSocket, actual drone APIs, arm, throttle, disarm, or gain commands.
- Every student screen must display or retain the clear boundary `교육용 시뮬레이션이며 실제 기체와 연결되지 않습니다`.
- Sensor permission methods, when present, must be invoked synchronously from the sensor-start button's click handler before unrelated asynchronous work.
- Static HTTP or missing/denied sensor access must preserve a complete touch path.
- Challenge duration is 20 seconds; score is deterministic and ranges from 0 to 1000.
- Target mobile viewports are 360×800 and 390×844 with no horizontal overflow or clipped primary controls.
- Keep all work on the current `feat/magcal-ellipsoid-fit` branch, commit only task files, and push the current branch after fresh full verification.

---

### Task 1: Pure browser-domain modules with RED→GREEN tests

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/imu.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/joystick.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/scoring.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/challenge.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/score-client.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/imu.test.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/joystick.test.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/scoring.test.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/challenge.test.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/score-client.test.mjs`

**Interfaces:**
- Produces: `detectSensorCapability(env) -> {available, secure, reason}`.
- Produces: `requestSensorAccess(env) -> Promise<{orientation, motion, reason}>`.
- Produces: `OrientationModel({smoothing})` with `updateOrientation`, `updateMotion`, `calibrate`, `snapshot`, and `reset`.
- Produces: `joystickVector(clientX, clientY, rect) -> {x, y, magnitude}`.
- Produces: `calculateStability(attitude)`, `accumulateScore(metrics, stability, dt)`, and `calculateScore(metrics)` from the isolated `scoring.mjs` module.
- Produces: `createChallengeState()`, `stepChallenge(state, input, dt)`, and `restartChallenge()` from `challenge.mjs`, which consumes `scoring.mjs`.
- Produces: `submitScore(fetchFn, payload) -> Promise<{status, response?}>` without mutating `payload` or the local result object.

- [ ] **Step 1: Write failing sensor tests**

  Test the production changes that would break safe fallback or sensor meaning: missing APIs must select touch, insecure origins must explain HTTPS, request methods must resolve granted/denied/error branches, calibration must subtract the current neutral sample, and smoothing must use the literal hand-calculated result.

  ```js
  assert.deepEqual(detectSensorCapability({ isSecureContext: true }), {
    available: false,
    secure: true,
    reason: "unsupported",
  });
  const model = new OrientationModel({ smoothing: 1 });
  model.updateOrientation({ beta: 10, gamma: -5 });
  model.calibrate();
  model.updateOrientation({ beta: 14, gamma: 3 });
  assert.deepEqual(model.snapshot().orientation, { roll: 8, pitch: 4 });
  ```

- [ ] **Step 2: Run sensor tests and verify RED**

  Run:

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/imu.test.mjs
  ```

  Expected: FAIL because `src/imu.mjs` does not exist.

- [ ] **Step 3: Implement the minimal sensor model and permission adapter**

  Use portrait mapping `roll = gamma`, `pitch = beta`, shortest-angle neutral subtraction,
  ±45° display clamps, 0.18 default new-sample smoothing, optional smoothed AX/AY/AZ, and
  explicit Korean-facing reason codes kept separate from DOM copy.

  ```js
  const next = previous + smoothing * (sample - previous);
  const roll = clamp(shortestDelta(gamma, neutralGamma), -45, 45);
  const pitch = clamp(shortestDelta(beta, neutralBeta), -45, 45);
  ```

- [ ] **Step 4: Write failing joystick, challenge, and score-client tests**

  Cover center/edge/outside joystick vectors, pointer-neutral reset data, two identical fixed-step
  input sequences producing byte-for-byte equal terminal states, a 20-second terminal state that
  ignores further updates, restart clearing all state, score bounds, and rejected fetch resolving to
  `offline` while the caller's result object remains unchanged.

  ```js
  assert.deepEqual(joystickVector(150, 100, { left: 100, top: 50, width: 100, height: 100 }), { x: 0, y: 0, magnitude: 0 });
  assert.equal(runSequence(sequence).score, runSequence(sequence).score);
  assert.equal(stepChallenge(done, { roll: 1, pitch: 1 }, 1 / 60), done);
  assert.deepEqual(await submitScore(() => Promise.reject(new TypeError("offline")), payload), { status: "offline" });
  ```

- [ ] **Step 5: Run the new tests and verify RED**

  Run:

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
  ```

  Expected: FAIL for missing `joystick.mjs`, `scoring.mjs`, `challenge.mjs`, and `score-client.mjs` exports.

- [ ] **Step 6: Implement minimal deterministic domain modules**

  Use a fixed-step compatible pure update, time-derived sinusoidal disturbance, explicit damping
  and restoring constants, input clamps, and a score based only on accumulated stability metrics.

  ```js
  export function calculateScore({ stabilityIntegral, elapsed }) {
    if (!(elapsed > 0)) return 0;
    return Math.round(clamp(stabilityIntegral / elapsed, 0, 100) * 10);
  }
  ```

- [ ] **Step 7: Run all module tests and verify GREEN**

  Run:

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
  ```

  Expected: all tests pass with no warnings.

- [ ] **Step 8: Commit Task 1**

  ```bash
  git add docs/presentations/ai-startup-camp-drone/mobile-lab/src docs/presentations/ai-startup-camp-drone/mobile-lab/tests
  git commit -m "feat: add deterministic mobile lab core"
  ```

### Task 2: Thread-safe optional score server with RED→GREEN tests

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/server.py`
- Create: `tools/test_mobile_lab_server.py`

**Interfaces:**
- Consumes: static files rooted at `mobile-lab/`.
- Produces: `ScoreStore.submit(payload) -> (record, created)` and `ScoreStore.snapshot() -> dict`.
- Produces: `build_server(host, port, static_root) -> ThreadingHTTPServer`.
- Produces: `POST /api/scores` and `GET /api/scores` JSON endpoints.

- [ ] **Step 1: Write failing validation and idempotency tests**

  Use literal payloads with exact allowed keys `submission_id`, `nickname`, `score`, `stability`,
  `duration_ms`, and `mode`. Verify 201 for first valid submission, 200 with `duplicate: true` for
  an identical retry, 409 for conflicting reuse, 400/413/415 for malformed, oversized, or wrong
  content type, and an unchanged count after every rejection.

  ```python
  payload = {
      "submission_id": "01234567-89ab-4cde-8fab-0123456789ab",
      "nickname": "하늘01",
      "score": 876,
      "stability": 87.6,
      "duration_ms": 20000,
      "mode": "touch",
  }
  self.assertEqual(201, response.status)
  self.assertEqual(1, self.get_scores()["count"])
  ```

- [ ] **Step 2: Run server tests and verify RED**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-server-red /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server -v
  ```

  Expected: FAIL because `mobile-lab/server.py` does not exist.

- [ ] **Step 3: Implement validation, store, HTTP handler, and CLI**

  Validate booleans separately from integers, normalize an empty nickname to `익명`, cap nickname
  at 20 Unicode code points, reject control characters and unknown fields, cap request bodies at
  4096 bytes, and sort top entries by descending score then ascending acceptance sequence.

  ```python
  with self._lock:
      existing = self._by_id.get(submission_id)
      if existing is not None:
          if existing["payload"] != canonical:
              raise SubmissionConflict
          return existing["public"], False
      # allocate sequence and store atomically
  ```

- [ ] **Step 4: Add failing 50-client concurrency and static-route tests**

  Start on `127.0.0.1:0`, submit 50 unique payloads through `ThreadPoolExecutor(max_workers=50)`,
  assert 50 successful responses and a count of 50, then concurrently retry one ID 50 times and
  assert only one additional stored record. Verify `/`, `presenter.html`, one `.mjs` asset, 404,
  and traversal protection.

- [ ] **Step 5: Run concurrency tests and verify the new RED failure**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-server-red2 /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server -v
  ```

  Expected: concurrency/static tests fail until threaded server and static routing are complete.

- [ ] **Step 6: Complete threaded server and security headers**

  Serve with `ThreadingHTTPServer`, `Cache-Control: no-store` on APIs, explicit JSON lengths,
  `Permissions-Policy: accelerometer=(self), gyroscope=(self)`, and optional `--cert`/`--key`
  TLS wrapping using `ssl.SSLContext`. Do not log request bodies or client addresses.

- [ ] **Step 7: Run server tests and verify GREEN**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-server-green /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server -v
  ```

  Expected: all validation, idempotency, static, and 50-client tests pass.

- [ ] **Step 8: Commit Task 2**

  ```bash
  git add docs/presentations/ai-startup-camp-drone/mobile-lab/server.py tools/test_mobile_lab_server.py
  git commit -m "feat: add optional mobile lab score server"
  ```

### Task 3: Student and presenter interface with local QR asset

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/index.html`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/presenter.html`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/styles.css`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/app.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/presenter.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/vendor/qrcode-generator/qrcode.js`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/vendor/qrcode-generator/LICENSE`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/vendor/qrcode-generator/README.md`
- Create: `tools/test_mobile_lab_browser.py`

**Interfaces:**
- Consumes: Task 1 domain modules and Task 2 same-origin API.
- Produces: accessible screen states `start`, `permission`, `calibration`, `imu`, `challenge`, and `result` exposed through `main[data-screen]`.
- Produces: presenter QR and leaderboard with a calm optional-server state.
- Produces: real Chrome/CDP regression coverage at both required viewports.

- [ ] **Step 1: Vendor the official QR source and license**

  Fetch npm package `qrcode-generator@2.0.4` into a temporary directory, compare its package
  metadata to the official repository, then add its browser source and verbatim MIT `LICENSE` under
  `vendor/qrcode-generator/`. Record exact version, source URL, and unmodified/local wrapper status
  in the vendor README. Runtime code must not contact a CDN or QR API.

- [ ] **Step 2: Write failing Chrome tests for start and sensor fallback flows**

  Reuse the repository's raw CDP pattern. Before navigation, install `Page.addScriptToEvaluateOnNewDocument`
  shims for: no API, granted permission, denied permission, and synthetic orientation/motion events.
  Assert visible Korean reasons, a touch fallback button, that permission methods are called during
  button activation, calibration produces zeroed readings, and later synthetic values update Roll,
  Pitch, AX, AY, and AZ.

  ```python
  self.assertEqual("touch", self.evaluate("document.querySelector('main').dataset.mode"))
  self.assertIn("HTTPS", self.text("[data-sensor-reason]"))
  self.assertEqual("0.0°", self.text("[data-roll-value]"))
  ```

- [ ] **Step 3: Run browser tests and verify RED**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-browser-red /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_browser -v
  ```

  Expected: FAIL because student and presenter pages do not exist.

- [ ] **Step 4: Build semantic student markup and the flight-instrument visual system**

  Follow the design spec exactly: 4px geometry, no gradients/shadows/card grid, Noto Sans CJK KR
  loaded from the existing sibling vendor folder, `#004094` UOS blue, instrument navy surfaces,
  one dominant artificial horizon, tabular telemetry, 48px minimum controls, safe-area padding,
  visible focus, reduced-motion handling, and a persistent safety boundary.

  ```html
  <p class="safety-boundary">교육용 시뮬레이션이며 실제 기체와 연결되지 않습니다</p>
  <main id="student-app" data-screen="start" data-mode="none">
    <!-- one active semantic section at a time; inactive sections use hidden -->
  </main>
  ```

- [ ] **Step 5: Connect permissions, calibration, IMU display, joystick, fixed-step challenge, local result, submit, and restart**

  Call permission adapters as the first work of the sensor button click handler. Use Pointer Events
  and pointer capture for the joystick. Drive physics with an accumulator at 1/60 second, pause
  accumulation across hidden tabs, retain result before fetch, use a per-result UUID submission ID,
  and reset every challenge field on retry.

  ```js
  sensorButton.addEventListener("click", async () => {
    const accessPromise = requestSensorAccess(window);
    showScreen("permission");
    const access = await accessPromise;
    // route granted or fallback state
  });
  ```

- [ ] **Step 6: Build presenter QR and optional leaderboard**

  Derive the default student URL from the current presenter URL without `presenter.html`, allow
  editing/copying, render a local SVG QR, poll `/api/scores`, show count and top 10, and keep a
  neutral `점수판은 선택 기능입니다` state on fetch failure.

- [ ] **Step 7: Add failing Chrome tests for touch, end/restart, offline result, presenter, and responsive geometry**

  Dispatch real pointer events, confirm nonzero Roll/Pitch input, run or deterministically advance a
  complete challenge, block `/api/scores`, confirm the same local score remains visible, retry and
  confirm reset. At 360×800 and 390×844 assert `scrollWidth <= innerWidth`, every visible primary
  button stays within viewport width, the fixed action area is not clipped, and screenshots are
  captured to a temporary test directory for inspection. Assert presenter QR exists and API-missing
  status is calm and usable.

- [ ] **Step 8: Run browser tests, inspect screenshots, and iterate to GREEN**

  Run:

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-browser-green /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_browser -v
  ```

  Expected: all real-Chrome tests pass; screenshots show no overlap, horizontal overflow, or clipped
  buttons at either viewport. Record that orientation is synthetic desktop data, not phone hardware.

- [ ] **Step 9: Commit Task 3**

  ```bash
  git add docs/presentations/ai-startup-camp-drone/mobile-lab/index.html docs/presentations/ai-startup-camp-drone/mobile-lab/presenter.html docs/presentations/ai-startup-camp-drone/mobile-lab/styles.css docs/presentations/ai-startup-camp-drone/mobile-lab/src/app.mjs docs/presentations/ai-startup-camp-drone/mobile-lab/src/presenter.mjs docs/presentations/ai-startup-camp-drone/mobile-lab/vendor tools/test_mobile_lab_browser.py
  git commit -m "feat: build mobile drone lab experience"
  ```

### Task 4: Operations documentation and final evidence

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/README.md`
- Modify only if needed for test discovery: none; `tools/test_*.py` is already discovered.

**Interfaces:**
- Consumes: final student/presenter URLs, server CLI, test commands, HTTPS behavior, and verified boundaries.
- Produces: one event-ready operator guide with explicit automated-versus-field proof.

- [ ] **Step 1: Write the operator README**

  Include exact commands for touch-capable HTTP preview, optional score server, optional TLS cert/key,
  student `/` and presenter `/presenter.html` URLs, QR editing/copying, HTTPS rationale, iOS sensor
  button permission, Android browser behavior, touch fallback, static HTTPS deployment behind an
  existing trusted host, and no external service creation by this task.

- [ ] **Step 2: Add the 50-person preflight checklist and safety boundary**

  The checklist must cover trusted HTTPS from two real iOS and two Android devices, permissions,
  portrait layout, QR scan distance, touch fallback, 50-device Wi-Fi/load rehearsal, presenter
  projector, score reset/restart behavior, offline drill, charging, and a separate instructor-only
  real-flight area. State that no student input reaches actual hardware.

- [ ] **Step 3: Document proof boundaries**

  List automated Node/Python/Chrome coverage separately from unverified physical sensors, mobile
  permission prompts, venue network, projector, and actual flight. Do not describe synthetic
  `DeviceOrientationEvent` as hardware sensor validation.

- [ ] **Step 4: Run fresh focused and repository-wide verification**

  Run:

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-final /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server tools.test_mobile_lab_browser -v
  PYTHONPYCACHEPREFIX=/tmp/zetin-drone-full-final /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
  /home/light/anaconda3/bin/python tools/check_repo_layout.py
  git diff --check
  ```

  Expected: every command exits 0; focused outputs include 50 concurrent submissions and both mobile
  viewports.

- [ ] **Step 5: Audit exact scope and forbidden changes**

  Run:

  ```bash
  git status --short
  git diff --name-status d8d6b4a..HEAD
  git diff --name-only d8d6b4a..HEAD -- 'docs/presentations/ai-startup-camp-drone/*.pptx' 'docs/presentations/ai-startup-camp-drone/index.html'
  git diff --cached --name-only
  rg -n 'WebUSB|WebBluetooth|serial|dgram|udp|WebSocket|arm|disarm|throttle|gain' docs/presentations/ai-startup-camp-drone/mobile-lab --glob '!README.md' --glob '!vendor/**'
  ```

  Expected: only task files are staged/changed, existing HTML/PPTX query is empty, protected PDF is
  untracked and absent from the index, and no student runtime exposes forbidden control paths.

- [ ] **Step 6: Commit documentation and any reviewed final fixes**

  ```bash
  git add docs/presentations/ai-startup-camp-drone/mobile-lab/README.md
  git commit -m "docs: add mobile lab event guide"
  ```

After the task-scoped and final whole-branch reviews are clean, the controller—not a task
implementer—pushes `feat/magcal-ellipsoid-fit`, fetches that exact remote branch, and verifies
`git rev-list --left-right --count HEAD...origin/feat/magcal-ellipsoid-fit` is exactly `0 0`.
