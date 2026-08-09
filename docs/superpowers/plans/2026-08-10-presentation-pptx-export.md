# Presentation PPTX Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Export the 77-slide HTML deck as a visually faithful 16:9 PPTX with 10 embedded H.264 videos and 77 speaker-note pages.

**Architecture:** A Node generator launches a local HTTP server and isolated headless Chrome, captures every active slide at 2560×1440, measures video rectangles, and packages the captures with embedded media through PptxGenJS. A Python artifact test independently inspects the generated OPC ZIP, slide dimensions, note count, video count, and codecs.

**Tech Stack:** Node.js, Chrome DevTools Protocol, PptxGenJS 4.0.1, chrome-remote-interface 0.34.0, FFmpeg/FFprobe, Python `unittest`, LibreOffice.

## Global Constraints

- Output path: `docs/presentations/ai-startup-camp-drone/ZETIN_Drone_AI_Startup_Camp.pptx`.
- Slide size and count: 16:9 and exactly 77 slides.
- Media: exactly 10 embedded MP4 files, all H.264.
- Notes: exactly 77 speaker-note slides.
- Visuals: no thumbnail rail, navigation overlay, animation residue, or browser chrome.
- The generated PPTX is presentation-oriented and its slide backgrounds are not text-editable.
- Videos are click-to-play in desktop PowerPoint; playback in LibreOffice, browser previews, and Google Slides is not guaranteed.

---

### Task 1: Define the PPTX artifact contract

**Files:**
- Create: `tools/test_presentation_pptx_export.py`
- Test: `tools/test_presentation_pptx_export.py`

**Interfaces:**
- Consumes: the PPTX file at the fixed output path.
- Produces: independent assertions for slide count, dimensions, notes, embedded videos, codecs, and package integrity.

- [x] **Step 1: Write the failing artifact test**

Create a `unittest.TestCase` that opens the file with `zipfile.ZipFile`, checks
`testzip() is None`, counts `ppt/slides/slideN.xml`, counts
`ppt/notesSlides/notesSlideN.xml`, parses `ppt/presentation.xml` for
`p:sldSz cx="12192000" cy="6858000"`, and extracts all embedded MP4 files to a
temporary directory. Run `ffprobe` for each extracted media file and require
`codec_name=h264`.

- [x] **Step 2: Run the test to verify RED**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-pptx-red \
  /home/light/anaconda3/bin/python -m unittest tools.test_presentation_pptx_export -v
```

Expected: FAIL because `ZETIN_Drone_AI_Startup_Camp.pptx` does not exist.

---

### Task 2: Implement the reproducible exporter

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/export_pptx.cjs`
- Modify: `docs/presentations/ai-startup-camp-drone/README.md`

**Interfaces:**
- Command: `node export_pptx.cjs [--output /absolute/file.pptx]`.
- Internal result: 77 PNG captures plus metadata records shaped as
  `{index, notes, screenshotPath, video: null | {path, coverData, x, y, w, h}}`.

- [x] **Step 1: Add dependency and process lifecycle helpers**

Implement `findFreePort()`, `waitForHttp(url)`, `run(command, args)`, and one
`try/finally` owner for the temporary directory, HTTP server, Chrome process,
and CDP client. Install exact temporary dependencies with:

```text
npm install --prefix <temp>/node --no-audit --no-fund --silent \
  pptxgenjs@4.0.1 chrome-remote-interface@0.34.0
```

- [x] **Step 2: Capture slides and media metadata**

At 1280×720 CSS pixels and device scale factor 2, set `no-rail`, hide the
shadow-root overlay, disable CSS animations/transitions, and wait for
`document.fonts.ready`. For indices 0 through 76 call `deckStage.goTo(index)`.
For an active video, set `muted`, remove controls, measure
`getBoundingClientRect()`, capture the full slide, then use an FFmpeg 0.5-second
frame and Pillow to composite a codec-independent poster and centered play
marker into the full slide and a separate media cover image.

- [x] **Step 3: Normalize incompatible video**

Inspect each source with FFprobe. Reuse H.264 MP4 files. Convert any other codec
with:

```bash
ffmpeg -y -i INPUT -c:v libx264 -profile:v high -level 4.1 \
  -pix_fmt yuv420p -c:a aac -b:a 160k -movflags +faststart OUTPUT.mp4
```

- [x] **Step 4: Package the PPTX**

Define and select a 13.333333 × 7.5-inch layout. For every slide add its PNG at
`x=0, y=0, w=13.333333, h=7.5`, add the speaker note through `slide.addNotes`,
and add video through `slide.addMedia({type:"video", path, cover, x, y, w, h})`
where CSS pixels are divided by 96 to produce inches. Write with compression.

- [x] **Step 5: Document export and playback**

Add README commands for `node export_pptx.cjs`, describe image-background
editability, and state that embedded videos are clicked in desktop PowerPoint.

---

### Task 3: Generate and verify the final artifact

**Files:**
- Create: `docs/presentations/ai-startup-camp-drone/ZETIN_Drone_AI_Startup_Camp.pptx`
- Modify: `docs/superpowers/plans/2026-08-10-presentation-pptx-export.md`

**Interfaces:**
- Consumes: `export_pptx.cjs` and the current HTML/assets.
- Produces: the final version-controlled PPTX.

- [x] **Step 1: Generate the PPTX**

```bash
cd docs/presentations/ai-startup-camp-drone
node export_pptx.cjs
```

Expected: 77 captures, 10 embedded videos, and the fixed output filename.

- [x] **Step 2: Run focused GREEN checks**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-pptx-green \
  /home/light/anaconda3/bin/python -m unittest tools.test_presentation_pptx_export -v
unzip -t docs/presentations/ai-startup-camp-drone/ZETIN_Drone_AI_Startup_Camp.pptx
```

Expected: artifact tests pass and ZIP reports no errors.

- [x] **Step 3: Validate through LibreOffice and inspect representative pages**

Convert the PPTX to PDF in a temporary directory, require 77 pages with
`pdfinfo`, render pages 1, 29, 67, and 77 to PNG, and visually inspect them for
cropping, rail/overlay residue, font failures, and correct video posters.

- [x] **Step 4: Run the complete repository suite**

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-pptx-full \
MPLCONFIGDIR=/tmp/zetin-pptx-full-mpl \
  /home/light/anaconda3/bin/python -m unittest discover -s tools -p 'test_*.py' -v
```

- [x] **Step 5: Commit and push only scoped files**

```bash
git add docs/presentations/ai-startup-camp-drone/export_pptx.cjs \
  docs/presentations/ai-startup-camp-drone/README.md \
  docs/presentations/ai-startup-camp-drone/ZETIN_Drone_AI_Startup_Camp.pptx \
  docs/superpowers/plans/2026-08-10-presentation-pptx-export.md \
  docs/superpowers/specs/2026-08-10-presentation-pptx-export-design.md \
  tools/test_presentation_pptx_export.py
git commit -m "feat: export AI startup camp deck as PPTX"
git push
```
