#!/usr/bin/env node
"use strict";

const fs = require("node:fs");
const fsp = require("node:fs/promises");
const http = require("node:http");
const net = require("node:net");
const os = require("node:os");
const path = require("node:path");
const { spawn, spawnSync } = require("node:child_process");
const { once } = require("node:events");

const DECK_DIR = __dirname;
const DEFAULT_OUTPUT = path.join(DECK_DIR, "ZETIN_Drone_AI_Startup_Camp.pptx");
const PYTHON_BIN = "/home/light/anaconda3/bin/python";
const SLIDE_COUNT = 84;
const VIEWPORT_W = 1280;
const VIEWPORT_H = 720;
const DEVICE_SCALE = 2;
const PX_PER_INCH = 96;
const PPTX_W = 13.333333;
const PPTX_H = 7.5;
const PPTXGEN_VERSION = "4.0.1";
const CDP_VERSION = "0.34.0";
const POSTER_COMPOSITOR = `
from PIL import Image, ImageDraw
import sys

slide_path, frame_path, cover_path = sys.argv[1:4]
x, y, width, height = map(int, sys.argv[4:8])

slide = Image.open(slide_path).convert("RGBA")
frame = Image.open(frame_path).convert("RGBA")
cover = Image.new("RGBA", (width, height), (6, 21, 47, 255))

scale = min(width / frame.width, height / frame.height)
resized = (
    max(1, round(frame.width * scale)),
    max(1, round(frame.height * scale)),
)
frame = frame.resize(resized, Image.Resampling.LANCZOS)
cover.alpha_composite(frame, ((width - resized[0]) // 2, (height - resized[1]) // 2))

draw = ImageDraw.Draw(cover, "RGBA")
radius = max(40, min(72, min(width, height) // 7))
center_x, center_y = width // 2, height // 2
line_width = max(3, radius // 18)
draw.ellipse(
    (center_x - radius, center_y - radius, center_x + radius, center_y + radius),
    fill=(0, 0, 0, 165),
    outline=(255, 255, 255, 230),
    width=line_width,
)
draw.polygon(
    [
        (center_x - radius // 4, center_y - radius // 2),
        (center_x - radius // 4, center_y + radius // 2),
        (center_x + radius // 2, center_y),
    ],
    fill=(255, 255, 255, 255),
)

slide.alpha_composite(cover, (x, y))
slide.convert("RGB").save(slide_path, "PNG")
cover.convert("RGB").save(cover_path, "PNG")
`;

function parseArgs(argv) {
  let output = DEFAULT_OUTPUT;
  for (let index = 0; index < argv.length; index += 1) {
    const arg = argv[index];
    if (arg === "--output") {
      const value = argv[index + 1];
      if (!value) throw new Error("--output requires a path");
      output = path.resolve(value);
      index += 1;
    } else if (arg === "--help" || arg === "-h") {
      console.log("Usage: node export_pptx.cjs [--output /path/deck.pptx]");
      process.exit(0);
    } else {
      throw new Error(`unknown argument: ${arg}`);
    }
  }
  return { output };
}

function findExecutable(candidates) {
  for (const candidate of candidates) {
    const result = spawnSync("sh", ["-c", `command -v "$1"`, "sh", candidate], {
      encoding: "utf8",
    });
    if (result.status === 0 && result.stdout.trim()) return result.stdout.trim();
  }
  throw new Error(`required executable not found: ${candidates.join(", ")}`);
}

function run(command, args, options = {}) {
  const result = spawnSync(command, args, {
    cwd: options.cwd || DECK_DIR,
    encoding: "utf8",
    stdio: options.quiet ? ["ignore", "pipe", "pipe"] : "inherit",
  });
  if (result.status !== 0) {
    const detail = options.quiet ? `\n${result.stderr || result.stdout || ""}` : "";
    throw new Error(`${command} exited with ${result.status}${detail}`);
  }
  return options.quiet ? result.stdout.trim() : "";
}

function delay(milliseconds) {
  return new Promise((resolve) => setTimeout(resolve, milliseconds));
}

function findFreePort() {
  return new Promise((resolve, reject) => {
    const server = net.createServer();
    server.unref();
    server.once("error", reject);
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      const port = typeof address === "object" && address ? address.port : null;
      server.close((error) => {
        if (error) reject(error);
        else if (!port) reject(new Error("failed to allocate a port"));
        else resolve(port);
      });
    });
  });
}

function requestOk(url) {
  return new Promise((resolve) => {
    const request = http.get(url, (response) => {
      response.resume();
      resolve(response.statusCode >= 200 && response.statusCode < 500);
    });
    request.setTimeout(300, () => request.destroy());
    request.on("error", () => resolve(false));
  });
}

async function waitForHttp(url, timeoutMs = 10000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (await requestOk(url)) return;
    await delay(50);
  }
  throw new Error(`timed out waiting for ${url}`);
}

async function stopProcess(child) {
  if (!child || child.exitCode !== null || child.signalCode !== null) return;
  child.kill("SIGTERM");
  await Promise.race([once(child, "exit"), delay(3000)]);
  if (child.exitCode === null && child.signalCode === null) {
    child.kill("SIGKILL");
    await once(child, "exit");
  }
}

function assertSafeRuntimeDir(runtimeDir) {
  const resolved = path.resolve(runtimeDir);
  const tempRoot = path.resolve(os.tmpdir()) + path.sep;
  if (!resolved.startsWith(tempRoot) || !path.basename(resolved).startsWith("zetin-pptx-export-")) {
    throw new Error(`refusing to remove unexpected runtime directory: ${resolved}`);
  }
}

async function evaluate(Runtime, expression) {
  const response = await Runtime.evaluate({
    expression,
    returnByValue: true,
    awaitPromise: true,
  });
  if (response.exceptionDetails) {
    const description = response.exceptionDetails.exception?.description || "browser evaluation failed";
    throw new Error(description);
  }
  return response.result.value;
}

async function waitForDeck(Runtime) {
  const deadline = Date.now() + 15000;
  while (Date.now() < deadline) {
    const ready = await evaluate(
      Runtime,
      `(() => {
        const stage = document.querySelector('deck-stage');
        return document.readyState === 'complete' && stage && stage._slides && stage._slides.length === ${SLIDE_COUNT};
      })()`
    );
    if (ready) return;
    await delay(50);
  }
  throw new Error("deck-stage did not become ready");
}

async function prepareDeck(Runtime) {
  return evaluate(
    Runtime,
    `(async () => {
      const stage = document.querySelector('deck-stage');
      stage.setAttribute('no-rail', '');
      stage.shadowRoot.querySelector('.overlay').style.display = 'none';
      stage.shadowRoot.querySelector('.rail').style.display = 'none';
      stage.shadowRoot.querySelector('.rail-resize').style.display = 'none';
      let style = document.getElementById('pptx-export-style');
      if (!style) {
        style = document.createElement('style');
        style.id = 'pptx-export-style';
        style.textContent =
          '*,*::before,*::after{animation:none!important;transition:none!important;caret-color:transparent!important}' +
          'html,body{margin:0!important;overflow:hidden!important;width:100%!important;height:100%!important}' +
          '#pptx-play-marker{pointer-events:none!important}';
        document.head.appendChild(style);
      }
      if (document.activeElement && document.activeElement.blur) document.activeElement.blur();
      await document.fonts.ready;
      await Promise.all([...document.images].map((image) => {
        if (image.complete) return Promise.resolve();
        return new Promise((resolve) => {
          const done = () => resolve();
          image.addEventListener('load', done, {once:true});
          image.addEventListener('error', done, {once:true});
          setTimeout(done, 3000);
        });
      }));
      stage._fit();
      await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      return {
        slides: stage._slides.length,
        notes: stage._slides.filter((slide) => slide.hasAttribute('data-speaker-notes')).length,
      };
    })()`
  );
}

async function prepareSlide(Runtime, index) {
  return evaluate(
    Runtime,
    `(async () => {
      const index = ${index};
      const stage = document.querySelector('deck-stage');
      stage.goTo(index);
      stage._fit();
      await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      const slide = stage._slides[index];
      await Promise.all([...slide.querySelectorAll('img')].map((image) => {
        if (image.complete) return Promise.resolve();
        return new Promise((resolve) => {
          const done = () => resolve();
          image.addEventListener('load', done, {once:true});
          image.addEventListener('error', done, {once:true});
          setTimeout(done, 2000);
        });
      }));

      document.getElementById('pptx-play-marker')?.remove();
      const video = slide.querySelector('video');
      let videoInfo = null;
      if (video) {
        video.muted = true;
        video.controls = false;
        video.loop = true;
        video.playsInline = true;
        if (video.readyState < 1) {
          await Promise.race([
            new Promise((resolve) => video.addEventListener('loadedmetadata', resolve, {once:true})),
            new Promise((resolve) => setTimeout(resolve, 3000)),
          ]);
        }
        video.pause();
        if (Number.isFinite(video.duration) && video.duration > 0) {
          const target = Math.min(0.5, video.duration / 3);
          if (Math.abs(video.currentTime - target) > 0.03) {
            video.currentTime = target;
            await Promise.race([
              new Promise((resolve) => video.addEventListener('seeked', resolve, {once:true})),
              new Promise((resolve) => setTimeout(resolve, 2000)),
            ]);
          }
        }
        video.pause();
        const rect = video.getBoundingClientRect();
        if (rect.width <= 0 || rect.height <= 0) throw new Error('active video has no visible rectangle');

        const marker = document.createElement('div');
        marker.id = 'pptx-play-marker';
        marker.style.cssText =
          'position:fixed;z-index:2147483647;display:grid;place-items:center;' +
          'left:' + rect.x + 'px;top:' + rect.y + 'px;width:' + rect.width + 'px;height:' + rect.height + 'px';
        const circle = document.createElement('div');
        circle.style.cssText =
          'width:72px;height:72px;border-radius:50%;display:grid;place-items:center;' +
          'background:rgba(0,0,0,.62);box-shadow:0 2px 10px rgba(0,0,0,.35);border:2px solid rgba(255,255,255,.9)';
        const triangle = document.createElement('div');
        triangle.style.cssText =
          'width:0;height:0;margin-left:7px;border-top:14px solid transparent;' +
          'border-bottom:14px solid transparent;border-left:23px solid white';
        circle.appendChild(triangle);
        marker.appendChild(circle);
        document.body.appendChild(marker);

        const sourceUrl = new URL(video.currentSrc || video.src, location.href);
        videoInfo = {
          sourcePath: decodeURIComponent(sourceUrl.pathname.replace(/^\\/+/, '')),
          rect: {x: rect.x, y: rect.y, width: rect.width, height: rect.height},
        };
      }
      await new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve)));
      return {
        index,
        notes: stage._notes[index] || slide.getAttribute('data-speaker-notes') || ' ',
        video: videoInfo,
      };
    })()`
  );
}

function pngDimensions(base64Data) {
  const buffer = Buffer.from(base64Data, "base64");
  if (buffer.length < 24 || buffer.toString("ascii", 1, 4) !== "PNG") {
    throw new Error("capture is not a PNG");
  }
  return { width: buffer.readUInt32BE(16), height: buffer.readUInt32BE(20) };
}

function resolveVideoPath(relativePath) {
  const resolved = path.resolve(DECK_DIR, relativePath);
  const deckRoot = path.resolve(DECK_DIR) + path.sep;
  if (!resolved.startsWith(deckRoot)) throw new Error(`video escaped deck directory: ${relativePath}`);
  if (!fs.existsSync(resolved)) throw new Error(`video source missing: ${resolved}`);
  return resolved;
}

function videoCodec(videoPath, ffprobeBin) {
  return run(
    ffprobeBin,
    [
      "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=codec_name",
      "-of", "default=noprint_wrappers=1:nokey=1", videoPath,
    ],
    { quiet: true }
  );
}

function normalizeVideo(videoPath, runtimeDir, ffprobeBin, ffmpegBin) {
  if (videoCodec(videoPath, ffprobeBin) === "h264") return videoPath;
  const output = path.join(runtimeDir, `h264-${path.basename(videoPath, path.extname(videoPath))}.mp4`);
  run(ffmpegBin, [
    "-y", "-i", videoPath, "-c:v", "libx264", "-profile:v", "high", "-level", "4.1",
    "-pix_fmt", "yuv420p", "-c:a", "aac", "-b:a", "160k", "-movflags", "+faststart", output,
  ]);
  if (videoCodec(output, ffprobeBin) !== "h264") throw new Error(`transcode did not produce H.264: ${output}`);
  return output;
}

async function renderVideoPoster(record, captureDir, ffmpegBin) {
  const source = resolveVideoPath(record.video.sourcePath);
  const stem = `slide-${String(record.index + 1).padStart(3, "0")}`;
  const framePath = path.join(captureDir, `${stem}-frame.png`);
  const coverPath = path.join(captureDir, `${stem}-cover.png`);
  run(
    ffmpegBin,
    ["-y", "-loglevel", "error", "-ss", "0.5", "-i", source, "-frames:v", "1", framePath],
    { quiet: true }
  );

  const rect = record.video.rect;
  const x = Math.round(rect.x * DEVICE_SCALE);
  const y = Math.round(rect.y * DEVICE_SCALE);
  const width = Math.round(rect.width * DEVICE_SCALE);
  const height = Math.round(rect.height * DEVICE_SCALE);
  run(
    PYTHON_BIN,
    [
      "-c", POSTER_COMPOSITOR, record.screenshotPath, framePath, coverPath,
      String(x), String(y), String(width), String(height),
    ],
    { quiet: true }
  );
  record.video.coverData = (await fsp.readFile(coverPath)).toString("base64");
}

async function buildPptx(records, runtimeDir, output, PptxGenJS, ffprobeBin, ffmpegBin) {
  const pptx = new PptxGenJS();
  pptx.author = "서울시립대학교";
  pptx.company = "서울시립대학교";
  pptx.subject = "AI 창업캠프 드론 비행제어 발표자료";
  pptx.title = "자작 드론 비행 제어 시스템 직접 만들기";
  pptx.lang = "ko-KR";
  pptx.theme = { headFontFace: "Noto Sans CJK KR", bodyFontFace: "Noto Sans CJK KR", lang: "ko-KR" };
  pptx.defineLayout({ name: "ZETIN_WIDE", width: PPTX_W, height: PPTX_H });
  pptx.layout = "ZETIN_WIDE";

  const normalizedVideos = new Map();
  for (const record of records) {
    const slide = pptx.addSlide();
    slide.background = { color: "FFFFFF" };
    slide.addImage({ path: record.screenshotPath, x: 0, y: 0, w: PPTX_W, h: PPTX_H });
    slide.addNotes(record.notes || " ");
    if (record.video) {
      const source = resolveVideoPath(record.video.sourcePath);
      let embedded = normalizedVideos.get(source);
      if (!embedded) {
        embedded = normalizeVideo(source, runtimeDir, ffprobeBin, ffmpegBin);
        normalizedVideos.set(source, embedded);
      }
      const rect = record.video.rect;
      slide.addMedia({
        type: "video",
        path: embedded,
        cover: `data:image/png;base64,${record.video.coverData}`,
        x: rect.x / PX_PER_INCH,
        y: rect.y / PX_PER_INCH,
        w: rect.width / PX_PER_INCH,
        h: rect.height / PX_PER_INCH,
        objectName: `Embedded video slide ${record.index + 1}`,
      });
    }
  }
  const generated = path.join(runtimeDir, "generated.pptx");
  await pptx.writeFile({ fileName: generated, compression: true });
  await fsp.mkdir(path.dirname(output), { recursive: true });
  await fsp.copyFile(generated, output);
}

async function main() {
  const { output } = parseArgs(process.argv.slice(2));
  const chromeBin = findExecutable(["google-chrome-stable", "google-chrome", "chromium"]);
  const ffmpegBin = findExecutable(["ffmpeg"]);
  const ffprobeBin = findExecutable(["ffprobe"]);
  if (!fs.existsSync(PYTHON_BIN)) throw new Error(`Python interpreter not found: ${PYTHON_BIN}`);

  const runtimeDir = await fsp.mkdtemp(path.join(os.tmpdir(), "zetin-pptx-export-"));
  assertSafeRuntimeDir(runtimeDir);
  let server = null;
  let chrome = null;
  let client = null;
  try {
    const dependencyRoot = path.join(runtimeDir, "node");
    console.log("[1/5] Installing temporary PPTX dependencies...");
    run("npm", [
      "install", "--prefix", dependencyRoot, "--no-audit", "--no-fund", "--silent",
      `pptxgenjs@${PPTXGEN_VERSION}`, `chrome-remote-interface@${CDP_VERSION}`,
    ]);
    const PptxGenJS = require(path.join(dependencyRoot, "node_modules", "pptxgenjs"));
    const CDP = require(path.join(dependencyRoot, "node_modules", "chrome-remote-interface"));

    const httpPort = await findFreePort();
    let debugPort = await findFreePort();
    while (debugPort === httpPort) debugPort = await findFreePort();
    const profileDir = path.join(runtimeDir, "chrome-profile");
    const captureDir = path.join(runtimeDir, "captures");
    await fsp.mkdir(captureDir, { recursive: true });

    server = spawn(
      PYTHON_BIN,
      ["-m", "http.server", String(httpPort), "--bind", "127.0.0.1", "--directory", DECK_DIR],
      { stdio: "ignore" }
    );
    await waitForHttp(`http://127.0.0.1:${httpPort}/`);

    chrome = spawn(
      chromeBin,
      [
        "--headless=new", "--no-sandbox", "--disable-gpu", "--hide-scrollbars",
        "--disable-background-mode", `--remote-debugging-port=${debugPort}`,
        "--remote-allow-origins=*", `--window-size=${VIEWPORT_W},${VIEWPORT_H}`,
        `--user-data-dir=${profileDir}`, "about:blank",
      ],
      { stdio: "ignore" }
    );
    await waitForHttp(`http://127.0.0.1:${debugPort}/json/version`);

    client = await CDP({ host: "127.0.0.1", port: debugPort });
    const { Emulation, Page, Runtime } = client;
    await Promise.all([Page.enable(), Runtime.enable()]);
    await Emulation.setDeviceMetricsOverride({
      width: VIEWPORT_W,
      height: VIEWPORT_H,
      deviceScaleFactor: DEVICE_SCALE,
      mobile: false,
    });
    await Page.navigate({ url: `http://127.0.0.1:${httpPort}/#1` });
    await waitForDeck(Runtime);
    const deckState = await prepareDeck(Runtime);
    if (deckState.slides !== SLIDE_COUNT || deckState.notes !== SLIDE_COUNT) {
      throw new Error(`unexpected deck state: ${JSON.stringify(deckState)}`);
    }

    console.log(`[2/5] Capturing ${SLIDE_COUNT} slides at ${VIEWPORT_W * DEVICE_SCALE}x${VIEWPORT_H * DEVICE_SCALE}...`);
    const records = [];
    for (let index = 0; index < SLIDE_COUNT; index += 1) {
      const record = await prepareSlide(Runtime, index);
      const screenshot = await Page.captureScreenshot({
        format: "png", fromSurface: true, captureBeyondViewport: false,
      });
      const dimensions = pngDimensions(screenshot.data);
      if (dimensions.width !== VIEWPORT_W * DEVICE_SCALE || dimensions.height !== VIEWPORT_H * DEVICE_SCALE) {
        throw new Error(`slide ${index + 1} capture is ${dimensions.width}x${dimensions.height}`);
      }
      record.screenshotPath = path.join(captureDir, `slide-${String(index + 1).padStart(3, "0")}.png`);
      await fsp.writeFile(record.screenshotPath, Buffer.from(screenshot.data, "base64"));
      if (record.video) {
        await renderVideoPoster(record, captureDir, ffmpegBin);
        await evaluate(Runtime, `document.getElementById('pptx-play-marker')?.remove()`);
      }
      records.push(record);
      process.stdout.write(`\r    slide ${String(index + 1).padStart(2, "0")}/${SLIDE_COUNT}`);
    }
    process.stdout.write("\n");

    const videoCount = records.filter((record) => record.video).length;
    if (videoCount !== 11) throw new Error(`expected 11 video slides, found ${videoCount}`);
    console.log(`[3/5] Packaging ${videoCount} embedded videos and ${SLIDE_COUNT} speaker notes...`);
    await buildPptx(records, runtimeDir, output, PptxGenJS, ffprobeBin, ffmpegBin);

    const stats = await fsp.stat(output);
    console.log(`[4/5] PPTX written: ${output}`);
    console.log(`      size: ${(stats.size / (1024 * 1024)).toFixed(1)} MiB`);
    console.log("[5/5] Temporary runtime will be removed.");
  } finally {
    if (client) await client.close().catch(() => {});
    await stopProcess(chrome);
    await stopProcess(server);
    assertSafeRuntimeDir(runtimeDir);
    await fsp.rm(runtimeDir, { recursive: true, force: true });
  }
}

main().catch((error) => {
  console.error(`[PPTX export failed] ${error.stack || error.message}`);
  process.exitCode = 1;
});
