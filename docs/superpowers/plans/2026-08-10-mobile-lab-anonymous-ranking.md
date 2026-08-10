# Mobile Lab Anonymous Ranking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give every no-input participant a stable anonymous display name and show only each exact name's best score.

**Architecture:** A pure client identity module creates and recovers a versioned browser-local alias, while `app.mjs` only binds that value to the form and result payload. The existing locked `ScoreStore` keeps UUID idempotency but projects one deterministic best record per canonical nickname.

**Tech Stack:** Static ES modules, Node built-in test runner, Python 3 standard library HTTP server and `unittest`, Chrome DevTools Protocol, existing Oracle release tooling.

## Global Constraints

- No login, account, cookie, external identity service, or personally identifying data storage.
- Preserve offline challenge completion and UUID submission idempotency.
- Exact nickname equality is case-sensitive after existing whitespace normalization.
- Highest score wins; equal scores preserve first acceptance order.
- Do not modify or stage `docs/cascade_vs_single_pid.pdf`, presentation HTML outside `mobile-lab`, or PPTX files.

---

### Task 1: Stable browser-local anonymous identity

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/identity.mjs`
- Create: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/identity.test.mjs`
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/src/app.mjs`
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/index.html`
- Modify: `tools/test_mobile_lab_browser.py`
- Modify: `tools/oracle_web/sites/mobile-lab.json`

**Interfaces:**
- Produces: `anonymousNickname(storage, cryptoSource): string`, formatted as `익명-XXXXXXXX`.
- Consumes: `Storage.getItem/setItem` and `Crypto.getRandomValues` only.

- [ ] **Step 1: Write failing Node tests**

Test literal alias generation from `Uint32Array([0x12ab34cd])`, stored-value reuse without a second random draw, invalid stored-value replacement, and storage exception fallback.

- [ ] **Step 2: Run the focused Node test and verify RED**

Run: `node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/identity.test.mjs`

Expected: FAIL because `src/identity.mjs` does not exist.

- [ ] **Step 3: Implement the pure identity module and app binding**

Create a module with a versioned storage key, exact alias validation, 32-bit Web Crypto generation, and exception-safe storage access. Initialize the input from the module and use the generated alias when the trimmed field is empty. Describe the default as browser-local anonymous identification in the start screen.

- [ ] **Step 4: Add failing then passing Chrome expectations**

Assert the start screen contains `익명-XXXXXXXX`, clearing the field still produces that alias in the local result, and navigation in the same browser restores the same alias. Add `identity.mjs` to the release manifest.

- [ ] **Step 5: Run Task 1 tests**

Run Node identity tests, all Node mobile tests, and `tools.test_mobile_lab_browser`.

---

### Task 2: Best score per exact nickname

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/server.py`
- Modify: `tools/test_mobile_lab_server.py`

**Interfaces:**
- Consumes: canonical payloads already accepted into `_by_id` under the store lock.
- Produces: `snapshot()` with `count` equal to unique canonical nicknames and at most ten best-per-name records.

- [ ] **Step 1: Write failing HTTP behavior tests**

Submit lower, higher, and equal scores under the same canonical nickname with distinct UUIDs. Assert one public record, the higher score, the matching mode/stability, first-wins tie order, and unique-name `count`.

- [ ] **Step 2: Run focused server tests and verify RED**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server.MobileLabServerTest.test_same_nickname_keeps_only_its_best_score`

Expected: FAIL because the snapshot currently emits every submission.

- [ ] **Step 3: Implement deterministic grouped projection**

Inside the snapshot lock, choose a record when a nickname is unseen or its score is strictly greater. Sort selected records by score descending and acceptance sequence ascending. Keep `_by_id`, capacity, POST status, and conflict behavior unchanged.

- [ ] **Step 4: Run full server and concurrency tests**

Run: `/home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server`

Expected: all tests pass, including 50-worker submission smoke coverage.

---

### Task 3: Full verification, release, and live Oracle proof

**Files:**
- Verify all files from Tasks 1 and 2.

**Interfaces:**
- Consumes: existing `build_release`, `deploy_release`, and `check_status` CLIs.
- Produces: a committed and pushed release whose static identity module and backend grouping code are both active.

- [ ] **Step 1: Run complete local verification**

Run Node, server, Chrome, Oracle release/deploy, layout, and `git diff --check` suites. Confirm no PPTX/PDF staging.

- [ ] **Step 2: Build and inspect the immutable release**

Build from the committed HEAD with `tools.oracle_web.build_release`; confirm `public/src/identity.mjs` is present and the user PDF is absent.

- [ ] **Step 3: Deploy and validate live behavior**

Deploy with `tools.oracle_web.deploy_release`, verify HTTPS module MIME, default alias rendering, touch-button transition, API same-name best-score behavior using non-personal synthetic names, and presenter count semantics.

- [ ] **Step 4: Push and prove synchronization**

Push the current branch and verify `HEAD...origin/<branch>` divergence is `0 0`.
