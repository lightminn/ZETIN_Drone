# Presentation Launcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Start the maintained HTML deck and an isolated Chrome window with one command, then stop the server automatically when that window closes.

**Architecture:** A Bash launcher owns both child processes and a temporary Chrome profile. An EXIT trap is the single cleanup path for normal close, browser failure, and signals; an integration test replaces only the external Chrome executable while exercising the real launcher and HTTP server.

**Tech Stack:** Bash, conda Python `http.server`, Google Chrome/Chromium, Python `unittest`

## Global Constraints

- Work in the current `feat/magcal-ellipsoid-fit` checkout and preserve every unrelated dirty file.
- Default to port 8000 and accept an optional port argument in the range 1~65535.
- Launch a dedicated Chrome app window with a temporary user profile.
- Always stop the HTTP server and delete only the launcher-created temporary directory.
- Use `PRESENTATION_CHROME_BIN` only as a test/override boundary; do not mock the HTTP server.

---

### Task 1: Add the tested presentation launcher

**Files:**
- Create: `tools/test_presentation_launcher.py`
- Create: `docs/presentations/ai-startup-camp-drone/present.sh`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`

**Interfaces:**
- Consumes: optional CLI port, `PRESENTATION_PYTHON_BIN`, and `PRESENTATION_CHROME_BIN`.
- Produces: `./present.sh [port]`, which returns the Chrome exit code after cleaning up the server and temporary profile.

- [ ] **Step 1: Write the failing integration tests**

Create tests that allocate an unused localhost port, run the real launcher with a temporary browser executable, fetch the real deck HTML, and assert the port is closed after browser success or failure.

- [ ] **Step 2: Run the focused test to verify RED**

Run:

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-presentation-launcher-red \
  /home/light/anaconda3/bin/python -m unittest tools.test_presentation_launcher -v
```

Expected: both tests fail because `present.sh` does not exist.

- [ ] **Step 3: Implement the minimal launcher**

Create an executable Bash script that validates dependencies, starts the server, waits for readiness, launches isolated Chrome with `--app`, waits for it, and cleans both child processes plus the temporary directory from one EXIT trap.

- [ ] **Step 4: Document the one-command workflow**

Make `./present.sh` the recommended preview command. Keep the manual `http.server` procedure as a fallback and explain that closing the dedicated Chrome window ends the server.

- [ ] **Step 5: Run focused and full verification**

Run:

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-presentation-launcher-green \
  /home/light/anaconda3/bin/python -m unittest tools.test_presentation_launcher -v
PYTHONPYCACHEPREFIX=/tmp/zetin-presentation-launcher-full \
MPLCONFIGDIR=/tmp/zetin-presentation-launcher-mpl \
  /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
```

Expected: focused tests pass and the full suite reports zero failures.

- [ ] **Step 6: Commit and push only the launcher scope**

```bash
git add docs/presentations/ai-startup-camp-drone/present.sh \
  docs/presentations/ai-startup-camp-drone/README.md \
  tools/test_presentation_launcher.py
git diff --cached --check
git commit -m "feat: add one-command presentation launcher"
git push
```
