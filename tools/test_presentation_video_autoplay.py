#!/usr/bin/env python3
"""Browser regression tests for presentation video autoplay."""

from __future__ import annotations

import json
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request
from html.parser import HTMLParser
from pathlib import Path

try:
    import websocket
except ImportError:  # pragma: no cover - environment-dependent skip
    websocket = None


REPO_ROOT = Path(__file__).resolve().parents[1]
DECK_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone"
CHROME_BIN = shutil.which("google-chrome-stable") or shutil.which("google-chrome")

TEAM_VISUALIZATIONS = {
    30: "mixer-saturation.mp4",
    33: "cascade-loop-timing.mp4",
    38: "accelerometer-confidence.mp4",
    54: "gravity-yaw-observability.mp4",
}

PYTHON_STATIC_IMAGES = {
    4: "drone-classification.png",
    5: "qualification-weight.png",
    6: "aircraft-uam.png",
    7: "mission-specs.png",
    9: "quadcopter-force-motion.png",
    10: "helicopter-quadcopter-torque.png",
    11: "swarm-system.png",
    12: "attitude-correction.png",
    46: "sil-closed-loop.png",
    63: "failsafe-timeline.png",
    64: "landing-observability.png",
    71: "shared-state-race.png",
    81: "telemetry-motor-balance.png",
}

GENERATED_DIAGRAM_VIDEOS = {
    Path(filename).with_suffix(".mp4").name
    for filename in PYTHON_STATIC_IMAGES.values()
}

REPLACED_SVG_FILENAMES = {
    "drone-classification-visual.svg",
    "qualification-weight-visual.svg",
    "aircraft-uam-visual.svg",
    "mission-specs-visual.svg",
    "quadcopter-force-motion-simple.svg",
    "helicopter-quadcopter-torque.svg",
    "swarm-system-simple.svg",
    "attitude-correction-simple.svg",
    "sil-closed-loop-simple.svg",
    "failsafe-timeline-simple.svg",
    "landing-observability-simple.svg",
    "shared-state-race-simple.svg",
    "telemetry-motor-balance-simple.svg",
}


def _unused_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class _DeckParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.sections: list[dict[str, object]] = []
        self.current_section: dict[str, object] | None = None

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        attributes = dict(attrs)
        if tag == "section":
            self.current_section = {
                "attrs": attributes,
                "videos": [],
                "images": [],
            }
            self.sections.append(self.current_section)
        elif tag == "video" and self.current_section is not None:
            self.current_section["videos"].append(attributes)
        elif tag == "img" and self.current_section is not None:
            self.current_section["images"].append(attributes)

    def handle_endtag(self, tag: str) -> None:
        if tag == "section":
            self.current_section = None


class PresentationVideoMarkupTests(unittest.TestCase):
    def test_deck_omits_zetin_team_name(self) -> None:
        source = (DECK_DIR / "index.html").read_text(encoding="utf-8")

        self.assertNotIn("zetin", source.casefold())

    def test_future_pptx_metadata_omits_zetin_team_name(self) -> None:
        source = (DECK_DIR / "export_pptx.cjs").read_text(encoding="utf-8")
        metadata = "\n".join(
            line.strip()
            for line in source.splitlines()
            if line.strip().startswith(("pptx.author", "pptx.company", "pptx.title"))
        )

        self.assertNotIn("zetin", metadata.casefold())

    def test_python_static_image_markup_and_assets_match_the_slide_contract(self) -> None:
        parser = _DeckParser()
        parser.feed((DECK_DIR / "index.html").read_text(encoding="utf-8"))

        self.assertEqual(len(parser.sections), 84)
        all_videos = [
            video
            for section in parser.sections
            for video in section["videos"]
        ]
        self.assertEqual(len(all_videos), 14)
        invalid_autoplay_markup = [
            attrs.get("src", "<unknown>")
            for attrs in all_videos
            if not all(
                name in attrs
                for name in ("controls", "loop", "muted", "playsinline")
            )
            or attrs.get("preload") != "metadata"
        ]
        self.assertEqual(invalid_autoplay_markup, [])

        for slide_number, filename in PYTHON_STATIC_IMAGES.items():
            with self.subTest(slide=slide_number, filename=filename):
                images = [
                    image
                    for image in parser.sections[slide_number - 1]["images"]
                    if "data-python-static" in image
                ]
                self.assertEqual(len(images), 1)
                image = images[0]
                self.assertEqual(image.get("src"), f"assets/{filename}")
                self.assertEqual(image.get("data-python-static"), Path(filename).stem)
                self.assertTrue(image.get("aria-label"))
                compact_style = re.sub(r"\s+", "", str(image.get("style", "")))
                self.assertIn("width:100%", compact_style)
                self.assertIn("aspect-ratio:16/9", compact_style)

        referenced_images = {
            Path(str(image.get("src", ""))).name
            for section in parser.sections
            for image in section["images"]
        }
        self.assertTrue(REPLACED_SVG_FILENAMES.isdisjoint(referenced_images))
        self.assertIn("mobile-lab-qr.svg", referenced_images)

        assets_dir = DECK_DIR / "assets"
        for filename in PYTHON_STATIC_IMAGES.values():
            with self.subTest(static_asset=filename):
                self.assertTrue((assets_dir / filename).is_file())
        self.assertEqual(
            sorted(
                filename
                for filename in GENERATED_DIAGRAM_VIDEOS
                if (assets_dir / filename).exists()
            ),
            [],
        )
        self.assertEqual(
            sorted(
                filename
                for filename in REPLACED_SVG_FILENAMES
                if (assets_dir / filename).exists()
            ),
            [],
        )
        self.assertTrue((assets_dir / "mobile-lab-qr.svg").is_file())


@unittest.skipUnless(CHROME_BIN and websocket, "Chrome and websocket-client are required")
class PresentationVideoBrowserTests(unittest.TestCase):
    server: subprocess.Popen[bytes]
    chrome: subprocess.Popen[bytes]
    ws: websocket.WebSocket
    command_id: int

    @classmethod
    def setUpClass(cls) -> None:
        cls.http_port = _unused_port()
        cls.debug_port = _unused_port()
        cls.runtime = tempfile.TemporaryDirectory(prefix="zetin-video-autoplay-")
        cls.server = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "http.server",
                str(cls.http_port),
                "--bind",
                "127.0.0.1",
                "--directory",
                str(DECK_DIR),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        cls.chrome = subprocess.Popen(
            [
                CHROME_BIN,
                "--headless=new",
                "--no-sandbox",
                "--disable-gpu",
                f"--remote-debugging-port={cls.debug_port}",
                "--remote-allow-origins=*",
                f"--user-data-dir={Path(cls.runtime.name) / 'profile'}",
                "about:blank",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        deadline = time.monotonic() + 5.0
        target = None
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{cls.debug_port}/json", timeout=0.2
                ) as response:
                    targets = json.load(response)
                target = next(item for item in targets if item.get("type") == "page")
                break
            except (OSError, StopIteration, ValueError):
                time.sleep(0.05)
        if target is None:
            cls._stop_processes()
            cls.runtime.cleanup()
            raise RuntimeError("Chrome DevTools target did not become ready")

        cls.ws = websocket.create_connection(target["webSocketDebuggerUrl"], timeout=5)
        cls.command_id = 0
        cls._call("Page.enable")
        cls._call("Runtime.enable")

    @classmethod
    def tearDownClass(cls) -> None:
        if hasattr(cls, "ws"):
            cls.ws.close()
        cls._stop_processes()
        if hasattr(cls, "runtime"):
            for attempt in range(20):
                try:
                    cls.runtime.cleanup()
                    break
                except OSError:
                    if attempt == 19:
                        raise
                    time.sleep(0.05)

    @classmethod
    def _stop_processes(cls) -> None:
        for name in ("chrome", "server"):
            process = getattr(cls, name, None)
            if process is None or process.poll() is not None:
                continue
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)

    @classmethod
    def _call(cls, method: str, params: dict | None = None) -> dict:
        cls.command_id += 1
        cls.ws.send(
            json.dumps(
                {"id": cls.command_id, "method": method, "params": params or {}}
            )
        )
        while True:
            message = json.loads(cls.ws.recv())
            if message.get("id") != cls.command_id:
                continue
            if "error" in message:
                raise RuntimeError(message["error"])
            return message.get("result", {})

    @classmethod
    def _evaluate(cls, expression: str, *, await_promise: bool = False):
        response = cls._call(
            "Runtime.evaluate",
            {
                "expression": expression,
                "returnByValue": True,
                "awaitPromise": await_promise,
            },
        )
        if "exceptionDetails" in response:
            raise RuntimeError(response["exceptionDetails"])
        return response["result"].get("value")

    @classmethod
    def _video_state(cls, filename: str) -> dict | None:
        quoted = json.dumps(filename)
        return cls._evaluate(
            r"""
            (() => {
              const filename = %s;
              const video = [...document.querySelectorAll('video')]
                .find(item => item.src.endsWith(filename));
              if (!video) return null;
              return {
                paused: video.paused,
                currentTime: video.currentTime,
                readyState: video.readyState,
                muted: video.muted,
                controls: video.controls,
                loop: video.loop,
                playsInline: video.playsInline,
                videoWidth: video.videoWidth,
                videoHeight: video.videoHeight,
                error: video.error && {code: video.error.code, message: video.error.message},
              };
            })()
            """
            % quoted
        )

    @classmethod
    def _wait_for_playback(cls, filename: str, timeout: float = 4.0) -> dict:
        deadline = time.monotonic() + timeout
        state = None
        while time.monotonic() < deadline:
            state = cls._video_state(filename)
            if state and not state["paused"] and state["currentTime"] > 0.2:
                return state
            time.sleep(0.05)
        raise AssertionError(f"{filename} did not autoplay; last state={state}")

    @classmethod
    def _open_deck(cls) -> None:
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{cls.http_port}/", timeout=0.2
                ) as response:
                    if response.status == 200:
                        break
            except OSError:
                time.sleep(0.05)
        else:
            raise AssertionError("presentation HTTP server did not become ready")
        cls._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{cls.http_port}/?_snthumb=1#1"},
        )
        deadline = time.monotonic() + 10.0
        ready = False
        while time.monotonic() < deadline:
            ready = bool(
                cls._evaluate(
                    "document.readyState === 'complete' && "
                    "document.querySelector('deck-stage')?._slides?.length === 84"
                )
            )
            if ready:
                break
            time.sleep(0.05)
        if not ready:
            raise AssertionError("84-slide presentation did not become ready")
        cls._evaluate(
            "(() => { const stage = document.querySelector('deck-stage'); "
            "stage.setAttribute('no-rail', ''); stage._fit(); return true; })()"
        )
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if cls._evaluate("document.fonts.status === 'loaded'"):
                break
            time.sleep(0.05)
        else:
            raise AssertionError("presentation fonts did not finish loading")

    def test_python_static_images_load_at_full_width_in_mapped_slides(self) -> None:
        self._open_deck()

        for slide_number, filename in PYTHON_STATIC_IMAGES.items():
            with self.subTest(slide=slide_number, filename=filename):
                self._evaluate(
                    "document.querySelector('deck-stage').goTo(%d)"
                    % (slide_number - 1)
                )
                deadline = time.monotonic() + 6.0
                state = None
                while time.monotonic() < deadline:
                    state = self._evaluate(
                        """
                        (() => {
                          const stage = document.querySelector('deck-stage');
                          const slide = stage._slides[%d];
                          const images = [...slide.querySelectorAll(
                            'img[data-python-static]'
                          )];
                          const image = images[0];
                          if (!image) return {count: images.length};
                          const rect = image.getBoundingClientRect();
                          const slideScale = slide.getBoundingClientRect().width / 1280;
                          return {
                            count: images.length,
                            source: image.getAttribute('src'),
                            marker: image.dataset.pythonStatic,
                            ariaLabel: image.getAttribute('aria-label'),
                            loaded: Boolean(image.complete && image.naturalWidth),
                            naturalWidth: image.naturalWidth,
                            naturalHeight: image.naturalHeight,
                            logicalWidth: rect.width / slideScale,
                          };
                        })()
                        """
                        % (slide_number - 1)
                    )
                    if state.get("loaded"):
                        break
                    time.sleep(0.05)

                self.assertEqual(state["count"], 1, state)
                self.assertEqual(state["source"], f"assets/{filename}", state)
                self.assertEqual(state["marker"], Path(filename).stem, state)
                self.assertTrue(state["ariaLabel"], state)
                self.assertTrue(state["loaded"], state)
                self.assertEqual(state["naturalWidth"], 1280, state)
                self.assertEqual(state["naturalHeight"], 720, state)
                self.assertGreaterEqual(state["logicalWidth"], 1040, state)

        qr = self._evaluate(
            """
            (() => {
              const slide = document.querySelector('deck-stage')._slides[79];
              const image = slide.querySelector('img[src$="mobile-lab-qr.svg"]');
              return {
                count: slide.querySelectorAll(
                  'img[src$="mobile-lab-qr.svg"]'
                ).length,
                loaded: Boolean(image?.complete && image.naturalWidth),
                naturalWidth: image?.naturalWidth || 0,
                naturalHeight: image?.naturalHeight || 0,
              };
            })()
            """
        )
        self.assertEqual(qr["count"], 1, qr)
        self.assertTrue(qr["loaded"], qr)
        self.assertGreater(qr["naturalWidth"], 0, qr)
        self.assertGreater(qr["naturalHeight"], 0, qr)

    def test_existing_evidence_media_remain_loaded(self) -> None:
        self._open_deck()
        result = self._evaluate(
            """
            (() => {
              const stage = document.querySelector('deck-stage');
              const collageSources = [...stage._slides[7].querySelectorAll(
                '[data-evidence-collage] img'
              )].map(image => image.getAttribute('src'));
              const timingVideo = stage._slides[32].querySelector(
                'video[src$="cascade-loop-timing.mp4"]'
              );
              const magChart = stage._slides[56].querySelector(
                'img[src$="chart_mag.png"]'
              );
              return {
                collageSources,
                timingVideoLoaded: Boolean(
                  timingVideo && timingVideo.readyState >= HTMLMediaElement.HAVE_METADATA
                ),
                magChartLoaded: Boolean(magChart?.complete && magChart.naturalWidth),
              };
            })()
            """
        )

        self.assertEqual(
            sorted(Path(source).name for source in result["collageSources"]),
            sorted(
                [
                    "image5.png",
                    "image12.png",
                    "chart_attitude.png",
                    "mobile-lab-student.png",
                ]
            ),
            result,
        )
        self.assertTrue(result["timingVideoLoaded"], result)
        self.assertTrue(result["magChartLoaded"], result)

    def test_concept_slides_use_relative_physical_language(self) -> None:
        self._open_deck()
        offenders = self._evaluate(
            r"""
            (() => {
              const stage = document.querySelector('deck-stage');
              const conceptSlides = [11, 12, 13, 16, 19, 21, 33, 37, 38, 39,
                47, 50, 51, 52, 53, 55, 56, 60, 61, 63, 67, 68, 69, 70,
                71, 77, 80, 81, 82, 83];
              const banned = /1\s*kHz|250\s*Hz|50\s*Hz|20\s*Hz|500\s*ms|115200\s*bps|1250\s*µs|10\s*배|5\s*대\s*군집|다섯\s*대\s*군집|1\s*초에\s*1000\s*번/i;
              return conceptSlides.flatMap(number => {
                const text = stage._slides[number - 1].textContent;
                const match = text.match(banned);
                return match ? [{number, phrase: match[0]}] : [];
              });
            })()
            """
        )

        self.assertEqual(offenders, [])

    def test_p_only_slide_uses_a_torque_balance_not_a_force_angle_equation(self) -> None:
        self._open_deck()
        text = self._evaluate(
            "document.querySelector('deck-stage')._slides[48].textContent"
        )

        self.assertNotIn("바람이 미는 힘 = Kp × 남은 기울기", text)
        self.assertIn("지속 외란 토크", text)
        self.assertIn("P 제어 경로의 복원 토크", text)

    def test_slide_type_scale_is_ten_percent_larger(self) -> None:
        self._open_deck()
        sizes = self._evaluate(
            """
            (() => {
              const slides = document.querySelector('deck-stage')._slides;
              const deepFind = (root, selector) => {
                const direct = root.querySelector?.(selector);
                if (direct) return direct;
                for (const element of root.querySelectorAll?.('*') || []) {
                  if (element.shadowRoot) {
                    const nested = deepFind(element.shadowRoot, selector);
                    if (nested) return nested;
                  }
                }
                return null;
              };
              const title = deepFind(slides[0], '.uos-title-slide__title-text');
              const body = [...slides[1].querySelectorAll('div')].find(
                element => element.textContent.trim() === '01'
              );
              const chartLabel = [...slides[57].querySelectorAll('svg text')].find(
                element => element.textContent.trim() === '기준 근처'
              );
              return {
                title: title && parseFloat(getComputedStyle(title).fontSize),
                body: body && parseFloat(getComputedStyle(body).fontSize),
                chart: chartLabel && parseFloat(getComputedStyle(chartLabel).fontSize),
              };
            })()
            """
        )

        self.assertAlmostEqual(sizes["title"], 64.5337, places=3)
        self.assertAlmostEqual(sizes["body"], 22.0, places=3)
        self.assertAlmostEqual(sizes["chart"], 16.5, places=3)

    def test_slide_60_renders_watchdog_timeout_as_directional_timeline(self) -> None:
        self._open_deck()
        timeline = self._evaluate(
            """
            (() => {
              const slide = document.querySelector('deck-stage')._slides[59];
              const label = slide.querySelector('[data-watchdog-label]');
              const result = [...slide.querySelectorAll('div')].find(
                element => element.textContent.trim() === '워치독 발동'
              );
              const arrow = slide.querySelector('[data-watchdog-arrow]');
              const arrowhead = slide.querySelector('[data-watchdog-arrowhead]');
              if (!label || !result || !arrow || !arrowhead) return null;
              const labelRect = label.getBoundingClientRect();
              const arrowRect = arrow.getBoundingClientRect();
              const arrowheadRect = arrowhead.getBoundingClientRect();
              const slideScale = slide.getBoundingClientRect().width / 1280;
              return {
                labelAboveArrow: labelRect.bottom < arrowRect.top,
                arrowWidth: arrowRect.width / slideScale,
                arrowHeight: arrowRect.height / slideScale,
                arrowheadAtRight:
                  arrowheadRect.left >= arrowRect.right - 1 &&
                  arrowheadRect.width > arrowheadRect.height,
              };
            })()
            """
        )

        self.assertIsNotNone(timeline)
        self.assertTrue(timeline["labelAboveArrow"])
        self.assertGreater(timeline["arrowWidth"], 180)
        self.assertAlmostEqual(timeline["arrowHeight"], 4.0, places=3)
        self.assertTrue(timeline["arrowheadAtRight"])

    def test_all_slide_text_stays_inside_clipping_ancestors(self) -> None:
        self.maxDiff = None
        self._open_deck()
        clipped = self._evaluate(
            """
            (async () => {
              const stage = document.querySelector('deck-stage');
              const clipped = [];
              const walk = (root, output) => {
                for (const element of root.querySelectorAll('*')) {
                  output.push(element);
                  if (element.shadowRoot) walk(element.shadowRoot, output);
                }
              };
              for (let index = 0; index < stage._slides.length; index += 1) {
                stage.goTo(index);
                stage._fit();
                await new Promise(resolve => requestAnimationFrame(
                  () => requestAnimationFrame(resolve)
                ));
                const slide = stage._slides[index];
                const elements = [];
                walk(slide, elements);
                for (const element of elements) {
                  const hasText = [...element.childNodes].some(
                    node => node.nodeType === Node.TEXT_NODE && node.textContent.trim()
                  );
                  if (!hasText) continue;
                  const style = getComputedStyle(element);
                  const rect = element.getBoundingClientRect();
                  if (style.display === 'none' || style.visibility === 'hidden' ||
                      Number(style.opacity) === 0 || rect.width < 0.5 || rect.height < 0.5) {
                    continue;
                  }
                  let ancestor = element;
                  while (ancestor) {
                    ancestor = ancestor.parentElement ||
                      (ancestor.getRootNode() instanceof ShadowRoot
                        ? ancestor.getRootNode().host : null);
                    if (!ancestor || ancestor === slide) break;
                    const ancestorStyle = getComputedStyle(ancestor);
                    const clips = [ancestorStyle.overflow, ancestorStyle.overflowX,
                      ancestorStyle.overflowY].some(value =>
                        value === 'hidden' || value === 'clip'
                      );
                    if (!clips) continue;
                    const parentRect = ancestor.getBoundingClientRect();
                    if (rect.left < parentRect.left - 1 || rect.top < parentRect.top - 1 ||
                        rect.right > parentRect.right + 1 ||
                        rect.bottom > parentRect.bottom + 1) {
                      clipped.push({
                        slide: index + 1,
                        label: slide.dataset.label,
                        text: element.textContent.trim().slice(0, 80),
                        rect: [rect.left, rect.top, rect.right, rect.bottom],
                        clippingRect: [parentRect.left, parentRect.top,
                          parentRect.right, parentRect.bottom],
                      });
                      break;
                    }
                  }
                }
              }
              return clipped;
            })()
            """,
            await_promise=True,
        )

        self.assertEqual(clipped, [])

    def test_rendered_deck_omits_scheduled_presentation_duration(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#1"},
        )
        deadline = time.monotonic() + 4.0
        deck_state = {"hash": "", "sections": 0, "content": ""}
        while time.monotonic() < deadline:
            deck_state = self._evaluate(
                """
                (() => {
                  const sections = [...document.querySelectorAll('section')];
                  const parts = [];
                  for (const section of sections) {
                    parts.push(section.textContent || '');
                    for (const element of section.querySelectorAll('*')) {
                      for (const attribute of element.attributes) parts.push(attribute.value);
                    }
                  }
                  return {
                    hash: location.hash,
                    sections: sections.length,
                    content: parts.join(' ').replace(/\\s+/g, ' ').trim(),
                  };
                })()
                """
            )
            if deck_state["hash"] == "#1" and deck_state["sections"] == 84:
                break
            time.sleep(0.05)

        self.assertEqual(deck_state["sections"], 84)
        self.assertNotRegex(
            deck_state["content"],
            r"(?:\d+(?:\.\d+)?\s*시간\s*과정|"
            r"(?:발표|시연|체험|진행).{0,8}\d+(?:\.\d+)?\s*(?:시간|분|초)|"
            r"\d+(?:\.\d+)?\s*(?:시간|분|초).{0,8}(?:발표|시연|체험|진행|과정))",
        )

    def test_slide_58_renders_yaw_drift_as_two_time_series(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#58"},
        )
        deadline = time.monotonic() + 4.0
        comparison = None
        while time.monotonic() < deadline:
            comparison = self._evaluate(
                """
                (() => {
                  const findDeep = (root, selector) => {
                    if (!root) return null;
                    const direct = root.querySelector?.(selector);
                    if (direct) return direct;
                    for (const element of root.querySelectorAll?.('*') || []) {
                      const nested = findDeep(element.shadowRoot, selector);
                      if (nested) return nested;
                    }
                    return null;
                  };
                  const chart = findDeep(document.body, 'svg[role="img"][aria-label^="30초 SIL"]');
                  return {
                    hash: location.hash,
                    readyState: document.readyState,
                    sections: document.querySelectorAll('section').length,
                    label: chart?.getAttribute('aria-label') || null,
                    seriesCount: chart?.querySelectorAll('polyline[data-series]').length || 0,
                    slideText: chart?.closest('section')?.textContent?.replace(/\s+/g, ' ').trim() || '',
                  };
                })()
                """
            )
            if (
                comparison["hash"] == "#58"
                and comparison["readyState"] == "complete"
                and comparison["sections"] == 84
                and comparison["label"]
            ):
                break
            time.sleep(0.05)

        self.assertEqual(comparison["hash"], "#58", comparison)
        self.assertEqual(comparison["readyState"], "complete", comparison)
        self.assertEqual(comparison["sections"], 84, comparison)
        self.assertEqual(
            comparison["label"],
            "30초 SIL: 자이로만 사용하면 Yaw 오차가 18.3도까지 누적되고, "
            "지자기 융합은 2.4도 근처에 머묾",
            comparison,
        )
        self.assertEqual(comparison["seriesCount"], 2, comparison)
        self.assertIn("SIL 시뮬레이션", comparison["slideText"], comparison)
        self.assertIn("전류 간섭 벤치", comparison["slideText"], comparison)

    def test_korean_whitespace_tokens_do_not_wrap_across_visual_lines(self) -> None:
        """Keep each Korean whitespace-delimited word together when Chrome lays out a slide."""
        self._open_deck()
        broken_tokens = self._evaluate(
            r"""
            (() => {
              const stage = document.querySelector('deck-stage');
              const broken = [];
              const originalIndex = stage.index;
              try {
                for (let index = 0; index < stage._slides.length; index += 1) {
                  stage.goTo(index);
                  const slide = stage._slides[index];
                  const walker = document.createTreeWalker(slide, NodeFilter.SHOW_TEXT);
                  let textNode;
                  while ((textNode = walker.nextNode())) {
                    const parent = textNode.parentElement;
                    if (!parent || parent.closest('script, style, svg, video')) continue;
                    for (const match of textNode.textContent.matchAll(/\S+/gu)) {
                      const token = match[0];
                      if (!/[가-힣]/u.test(token)) continue;
                      const range = document.createRange();
                      range.setStart(textNode, match.index);
                      range.setEnd(textNode, match.index + token.length);
                      const rows = [...range.getClientRects()]
                        .filter((rect) => rect.width > 0.5 && rect.height > 0.5)
                        .map((rect) => Math.round(rect.top));
                      if (new Set(rows).size > 1) {
                        broken.push({
                          slide: index + 1,
                          label: slide.dataset.label,
                          token,
                          context: textNode.textContent.trim().slice(0, 120),
                        });
                      }
                    }
                  }
                }
              } finally {
                stage.goTo(originalIndex);
              }
              return broken;
            })()
            """
        )

        self.assertEqual(broken_tokens, [], json.dumps(broken_tokens, ensure_ascii=False))

    def test_active_video_autoplays_and_previous_video_resets(self) -> None:
        self._open_deck()
        self._evaluate("document.querySelector('deck-stage').goTo(34)")
        self._wait_for_playback("accelerometer.mp4")

        self._evaluate("document.querySelector('deck-stage').goTo(35)")
        self._wait_for_playback("gyro.mp4")
        previous = self._video_state("accelerometer.mp4")

        self.assertIsNotNone(previous)
        self.assertTrue(previous["paused"])
        self.assertLess(previous["currentTime"], 0.05)

    def test_active_video_restores_runtime_playback_properties(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#35"},
        )
        deadline = time.monotonic() + 4.0
        state = None
        while time.monotonic() < deadline:
            state = self._video_state("accelerometer.mp4")
            if state and state["readyState"] >= 1:
                break
            time.sleep(0.05)

        self.assertIsNotNone(state)
        self.assertTrue(state["muted"])
        self.assertTrue(state["controls"])
        self.assertTrue(state["loop"])
        self.assertTrue(state["playsInline"])

    def test_team_visualizations_decode_and_remain_prominent(self) -> None:
        self._open_deck()

        for slide_number, filename in TEAM_VISUALIZATIONS.items():
            with self.subTest(slide=slide_number, filename=filename):
                self._evaluate(
                    "document.querySelector('deck-stage').goTo(%d)"
                    % (slide_number - 1)
                )
                state = self._wait_for_playback(filename)
                self.assertEqual(state["videoWidth"], 1280, state)
                self.assertEqual(state["videoHeight"], 720, state)

                rendered_width = self._evaluate(
                    """
                    (() => {
                      const stage = document.querySelector('deck-stage');
                      const slide = stage._slides[%d];
                      const video = [...slide.querySelectorAll('video')]
                        .find(item => item.src.endsWith(%s));
                      if (!video) return null;
                      const scale = slide.getBoundingClientRect().width / 1280;
                      return video.getBoundingClientRect().width / scale;
                    })()
                    """
                    % (slide_number - 1, json.dumps(filename))
                )
                self.assertIsNotNone(rendered_width)
                self.assertGreaterEqual(rendered_width, 620)


if __name__ == "__main__":
    unittest.main()
