# Presentation Video Autoplay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Automatically play a slide's muted video when that slide becomes active, without requiring presenter input.

**Architecture:** Extend the existing `deck-stage` slide activation path with one media lifecycle method. It plays video elements in the active slide and pauses/resets videos in every inactive slide, so hidden media never consumes resources or continues out of view.

**Tech Stack:** Vanilla JavaScript custom elements, HTML5 video, Python `unittest`, Chrome DevTools Protocol integration test.

## Global Constraints

- Only the active slide may play video.
- Re-entering a video slide starts its video at 0 seconds.
- A rejected `play()` Promise must not block navigation.
- Printing must not start or continue video playback.
- Existing `controls loop muted playsinline preload="metadata"` attributes remain intact.

---

### Task 1: Active-slide video lifecycle

**Files:**
- Create: `tools/test_presentation_video_autoplay.py`
- Modify: `docs/presentations/ai-startup-camp-drone/deck-stage.js:720-750,1496-1515`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md:34-55`

**Interfaces:**
- Consumes: `DeckStage._slides: HTMLElement[]` and `DeckStage._index: number`.
- Produces: `DeckStage._syncSlideMedia(activeIndex: number): void`.

- [x] **Step 1: Write the failing browser regression test**

Create a `unittest.TestCase` that starts the real deck HTTP server and an isolated headless Chrome profile. Navigate directly to `#29`, poll `accelerometer.mp4`, and require `paused === false` with `currentTime > 0`. Then call `document.querySelector('deck-stage').goTo(29)` and require the old video to be paused at 0 while `gyro.mp4` advances. Also parse `index.html` and assert every video has the `muted` attribute.

- [x] **Step 2: Run the focused test to verify RED**

Run:

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-video-autoplay-red \
  /home/light/anaconda3/bin/python -m unittest tools.test_presentation_video_autoplay -v
```

Expected: the browser test fails because the active video remains paused at 0 seconds.

- [x] **Step 3: Implement the minimal media lifecycle**

Add `_syncSlideMedia(activeIndex)` to `deck-stage.js`. For each video in `_slides`, call `play()` only when its slide index equals `activeIndex` and the deck is not printing. For all other videos call `pause()` and attempt to set `currentTime = 0`. Catch both seek exceptions and rejected `play()` Promises. Call the method from `_applyIndex`, pause all media in `beforeprint` and `disconnectedCallback`, and resume the current slide through the existing `afterprint -> _applyIndex` path.

- [x] **Step 4: Update presenter documentation**

Change the README controls section to state that video starts automatically on entry, repeats muted, and can still be paused or scrubbed with native controls.

- [x] **Step 5: Run focused GREEN and syntax checks**

Run:

```bash
node --check docs/presentations/ai-startup-camp-drone/deck-stage.js
PYTHONPYCACHEPREFIX=/tmp/zetin-video-autoplay-green \
  /home/light/anaconda3/bin/python -m unittest tools.test_presentation_video_autoplay -v
```

Expected: JavaScript syntax exits 0 and both autoplay assertions pass.

- [x] **Step 6: Run the complete repository test suite**

Run:

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-video-autoplay-full \
MPLCONFIGDIR=/tmp/zetin-video-autoplay-mpl \
  /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
```

Expected: all tests pass with no failures or errors.

- [x] **Step 7: Commit and push the scoped change**

```bash
git add tools/test_presentation_video_autoplay.py \
  docs/presentations/ai-startup-camp-drone/deck-stage.js \
  docs/presentations/ai-startup-camp-drone/README.md \
  docs/superpowers/plans/2026-08-10-presentation-video-autoplay.md
git commit -m "feat: autoplay presentation videos on slide entry"
git push
```
