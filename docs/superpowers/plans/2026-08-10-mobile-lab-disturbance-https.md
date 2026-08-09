# Mobile Lab Disturbance and Local HTTPS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make unattended hover score 400–550 on average with varied but reproducible wind, then serve the lab at a publicly trusted `https://uos-drone.kro.kr:8443/` URL.

**Architecture:** The client physics owns a seeded xorshift32 generator and evolves slow wind, turbulence, and gust state at the existing fixed 60 Hz step. The browser supplies one fresh seed per attempt; the score server remains outside the physics path. HTTPS uses a DNS-01 certificate stored outside the repository and the existing Python TLS flags.

**Tech Stack:** Static JavaScript ES modules, Node's built-in test runner, raw Chrome DevTools Protocol browser tests, Python standard-library HTTPS server, Certbot DNS-01.

## Global Constraints

- No `Math.random()` inside the physics step; fixed seed plus fixed inputs must be byte-identical.
- Across the literal calibration seed bank, no-input 20-second mean score must be 400–550.
- The game must remain recoverable by an independently specified feedback pilot.
- Challenge duration remains exactly 20 seconds and score remains 0–1000.
- No real-drone, firmware, serial, WebUSB, WebBluetooth, UDP, arm, throttle, or gain path may be added.
- No certificate, private key, ACME account file, or DNS credential may enter the repository.
- Preserve the untracked user file `docs/cascade_vs_single_pid.pdf`.

---

### Task 1: Seeded Wind, Turbulence, and Gust Physics

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/challenge.mjs`
- Test: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/challenge.test.mjs`

**Interfaces:**
- Consumes: existing `calculateStability`, `accumulateScore`, and `calculateScore` functions.
- Produces: `createChallengeState({ seed } = {})`, `stepChallenge(state, input, dt)`, and `restartChallenge(previousState, { seed } = {})`.

- [ ] **Step 1: Write failing seed and difficulty tests**

  Add the literal seed bank below and simulate exactly 1200 steps of 1/60 second:

  ```js
  const CALIBRATION_SEEDS = [
    0x10203040, 0x1badb002, 0x31415926, 0x5eed1234,
    0x7f4a7c15, 0x89abcdef, 0xc001d00d, 0xf00dcafe,
  ];
  ```

  Tests must prove:

  - the same seed and same literal input sequence produce byte-identical state;
  - at least two different seeds produce different Roll/Pitch histories;
  - the no-input mean is 400–550, every seed is 300–650, and at least one seed exceeds 8° resultant attitude;
  - the following independent pilot averages at least 700 and beats the corresponding no-input score by at least 180 points:

  ```js
  const input = {
    roll: Math.max(-1, Math.min(1, -0.075 * state.roll - 0.12 * state.rollRate)),
    pitch: Math.max(-1, Math.min(1, -0.075 * state.pitch - 0.12 * state.pitchRate)),
  };
  ```

  - restart with a new seed clears elapsed time, rates, wind, gust, score, and stored input while preserving the new normalized seed.

- [ ] **Step 2: Run the RED tests**

  Run:

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/challenge.test.mjs
  ```

  Expected: failures because seed-specific histories and the 400–550 difficulty envelope do not exist; the current no-input score remains about 959.

- [ ] **Step 3: Implement the minimal deterministic wind model**

  Normalize the seed to a nonzero unsigned 32-bit value and advance it with xorshift32:

  ```js
  function nextRandom(state) {
    let next = state >>> 0;
    next ^= next << 13;
    next ^= next >>> 17;
    next ^= next << 5;
    next >>>= 0;
    return { state: next || 0x6d2b79f5, value: next / 0x100000000 };
  }
  ```

  Store `seed`, `randomState`, `windRoll`, `windPitch`, `gustRoll`,
  `gustPitch`, and `gustRemaining` in challenge state. At each step:

  - update slow wind as a damped random walk using `sqrt(duration)` scaling;
  - start an independent 0.6–1.2 second gust with a duration-scaled probability when no gust is active;
  - apply a smooth sine envelope to the stored gust force;
  - combine slow wind, small cross-axis waves, and gust force before restoring and damping terms;
  - keep control torque high enough for the literal pilot to satisfy the recovery contract.

  Tune only force, damping, and gust constants; do not weaken the test contracts.

- [ ] **Step 4: Run GREEN and the complete Node suite**

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
  ```

  Expected: all tests pass with exact 20-second completion and no cancellations.

- [ ] **Step 5: Commit Task 1**

  ```bash
  git add docs/presentations/ai-startup-camp-drone/mobile-lab/src/challenge.mjs \
    docs/presentations/ai-startup-camp-drone/mobile-lab/tests/challenge.test.mjs
  git commit -m "feat: strengthen seeded hover disturbance"
  ```

### Task 2: Fresh Seed per Attempt and Browser Regression

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/app.mjs`
- Modify: `tools/test_mobile_lab_browser.py`

**Interfaces:**
- Consumes: Task 1's option-bearing `createChallengeState` and `restartChallenge` interfaces.
- Produces: a fresh unsigned seed supplied once for each first attempt and retry.

- [ ] **Step 1: Write the failing real-Chrome retry test**

  In the existing manual-clock touch challenge test, capture Roll/Pitch after
  the first 120 frames, finish the attempt, retry, run the same 120 neutral
  frames, and assert the second attitude pair differs. Preserve the existing
  assertions that result survives failed score submission and retry resets
  time, score, Roll, and Pitch before frames advance.

- [ ] **Step 2: Run the focused RED browser test**

  ```bash
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-seed-red \
    /home/light/anaconda3/bin/python -m unittest -v \
    tools.test_mobile_lab_browser.MobileLabBrowserTests.test_pointer_challenge_offline_result_and_restart
  ```

  Expected: the two attempts follow the same default disturbance because the app does not yet supply fresh seeds.

- [ ] **Step 3: Add one seed generator at the app boundary**

  Add a private `nextChallengeSeed()` that fills one `Uint32Array` through
  `crypto.getRandomValues`, replacing zero with a fixed nonzero fallback.
  Call `createChallengeState({ seed: nextChallengeSeed() })` for a first attempt
  and `restartChallenge(challenge, { seed: nextChallengeSeed() })` for retry.
  Do not send the seed to the score server or expose it as a hardware value.

- [ ] **Step 4: Run GREEN browser and Node suites**

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-seed-green \
    /home/light/anaconda3/bin/python -m unittest -v \
    tools.test_mobile_lab_browser tools.test_mobile_lab_server
  ```

  Expected: all Node, Chrome, and score-server tests pass.

- [ ] **Step 5: Commit Task 2**

  ```bash
  git add docs/presentations/ai-startup-camp-drone/mobile-lab/src/app.mjs \
    tools/test_mobile_lab_browser.py
  git commit -m "feat: vary hover wind between attempts"
  ```

### Task 3: DNS-Validated Local HTTPS Rehearsal

**Files:**
- Repository changes: none.
- External state: DNS A/TXT records and user-local Certbot state outside the repository.

**Interfaces:**
- Consumes: `server.py --cert PATH --key PATH` and user control of `uos-drone.kro.kr` DNS.
- Produces: trusted student and presenter HTTPS URLs on port 8443.

- [ ] **Step 1: Verify public DNS prerequisites**

  Confirm the public A answer is `192.168.0.6`. During issuance, give the user
  the exact `_acme-challenge.uos-drone.kro.kr` TXT value and wait until two
  independent public DNS-over-HTTPS resolvers return it.

- [ ] **Step 2: Install and run Certbot without repository state**

  Install the distribution Certbot package if absent. Run manual DNS-01 with
  `--agree-tos --register-unsafely-without-email` and explicit user-local
  `--config-dir`, `--work-dir`, and `--logs-dir` under `~/.local`/`~/.cache`.
  Never copy the key into the repository or `/tmp`.

  ```bash
  echo "$SUDO_PASS" | sudo -S pacman -S --needed --noconfirm certbot
  /usr/bin/certbot certonly --manual --preferred-challenges dns \
    --agree-tos --register-unsafely-without-email \
    --config-dir /home/light/.local/share/letsencrypt \
    --work-dir /home/light/.cache/letsencrypt \
    --logs-dir /home/light/.local/state/letsencrypt \
    --cert-name uos-drone.kro.kr -d uos-drone.kro.kr
  ```

- [ ] **Step 3: Start the existing TLS server**

  Keep the HTTP preview available only if it does not conflict. Launch:

  ```bash
  /home/light/anaconda3/bin/python server.py \
    --host 0.0.0.0 --port 8443 \
    --cert /home/light/.local/share/letsencrypt/live/uos-drone.kro.kr/fullchain.pem \
    --key /home/light/.local/share/letsencrypt/live/uos-drone.kro.kr/privkey.pem
  ```

- [ ] **Step 4: Verify TLS, pages, and API**

  Use `openssl s_client` with SNI and `curl` through the public hostname to
  verify certificate hostname, validity, student HTTP 200, presenter HTTP 200,
  and `{"count":0,"scores":[]}` from `/api/scores`. The user then performs the
  physical phone permission and IMU-axis check.

### Task 4: Final Repository Verification and Push

**Files:**
- No new production files beyond Tasks 1–2.

**Interfaces:**
- Consumes: all previous task commits.
- Produces: verified local and remote branch state.

- [ ] **Step 1: Run full verification**

  ```bash
  node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
  PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-disturbance-final \
    /home/light/anaconda3/bin/python -m unittest -v \
    tools.test_mobile_lab_server tools.test_mobile_lab_browser
  /home/light/anaconda3/bin/python tools/check_repo_layout.py
  git diff --check
  ```

- [ ] **Step 2: Recheck safety and protected files**

  Confirm task commits contain no existing presentation HTML/PPTX, no
  `docs/cascade_vs_single_pid.pdf`, and no prohibited runtime communication
  API. Confirm the PDF remains untracked and unstaged.

- [ ] **Step 3: Push and verify synchronization**

  Push `feat/magcal-ellipsoid-fit`, fetch it again, and require:

  ```text
  git rev-list --left-right --count HEAD...origin/feat/magcal-ellipsoid-fit
  0 0
  ```
