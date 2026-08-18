#!/usr/bin/env python3
"""Browser regression tests for presentation video autoplay."""

from __future__ import annotations

import json
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
    38: "accelerometer-confidence.mp4",
    52: "cascade-loop-response.mp4",
    54: "gravity-yaw-observability.mp4",
}


def _unused_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class _VideoParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.videos: list[dict[str, str | None]] = []

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        if tag == "video":
            self.videos.append(dict(attrs))


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

    def test_every_video_is_muted_for_input_free_autoplay(self) -> None:
        parser = _VideoParser()
        parser.feed((DECK_DIR / "index.html").read_text(encoding="utf-8"))

        self.assertEqual(len(parser.videos), 14)
        missing = [attrs.get("src", "<unknown>") for attrs in parser.videos if "muted" not in attrs]
        self.assertEqual(missing, [])


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
            cls.runtime.cleanup()

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
            """
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

    def test_significance_section_renders_professor_feedback_visuals(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#4"},
        )
        deadline = time.monotonic() + 4.0
        visuals = None
        while time.monotonic() < deadline:
            visuals = self._evaluate(
                """
                (() => {
                  if (document.readyState !== 'complete') return null;
                  const classification = document.querySelector('[data-drone-classification]');
                  const qualification = document.querySelector('[data-qualification-scale]');
                  const comparison = document.querySelector('[data-aircraft-comparison]');
                  const missions = document.querySelector('[data-mission-specs]');
                  const forceSum = document.querySelector('svg[data-force-sum]');
                  const mixing = document.querySelector('svg[data-control-mixing]');
                  const swarm = document.querySelector('[data-swarm-expansion]');
                  return {
                    classification: Boolean(classification),
                    qualificationBands: qualification?.querySelectorAll('[data-band]').length || 0,
                    aircraftRows: comparison?.querySelectorAll('[data-aircraft]').length || 0,
                    missionRows: missions?.querySelectorAll('[data-mission]').length || 0,
                    forceLabel: forceSum?.getAttribute('aria-label') || null,
                    mixingLabel: mixing?.getAttribute('aria-label') || null,
                    swarmLayers: swarm?.querySelectorAll('[data-swarm-layer]').length || 0,
                    futureGoal: swarm?.dataset.status || null,
                  };
                })()
                """
            )
            if visuals and visuals["classification"]:
                break
            time.sleep(0.05)

        self.assertIsNotNone(visuals)
        self.assertTrue(visuals["classification"], visuals)
        self.assertEqual(visuals["qualificationBands"], 5, visuals)
        self.assertEqual(visuals["aircraftRows"], 4, visuals)
        self.assertEqual(visuals["missionRows"], 4, visuals)
        self.assertIn("네 로터 추력의 벡터 합", visuals["forceLabel"])
        self.assertIn("힘과 토크", visuals["mixingLabel"])
        self.assertEqual(visuals["swarmLayers"], 6, visuals)
        self.assertEqual(visuals["futureGoal"], "future-goal", visuals)

    def test_slide_9_force_illustration_has_a_clear_vector_hierarchy(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#9"},
        )
        deadline = time.monotonic() + 4.0
        hierarchy = None
        while time.monotonic() < deadline:
            hierarchy = self._evaluate(
                """
                (() => {
                  if (document.readyState !== 'complete') return null;
                  const figure = document.querySelector('svg[data-force-sum]');
                  if (!figure) return null;
                  return {
                    layout: figure.closest('[data-force-layout]')?.dataset.forceLayout || null,
                    logicalWidth: figure.getBoundingClientRect().width /
                      (figure.closest('section').getBoundingClientRect().width / 1280),
                    rotorThrusts: figure.querySelectorAll(
                      '[data-vector="rotor-thrust"]'
                    ).length,
                    aggregateThrusts: figure.querySelectorAll(
                      '[data-vector="aggregate-thrust"]'
                    ).length,
                    weights: figure.querySelectorAll('[data-vector="weight"]').length,
                    drags: figure.querySelectorAll('[data-vector="drag"]').length,
                    forceLabels: figure.querySelectorAll('[data-force-label]').length,
                    droneBodies: figure.querySelectorAll('[data-drone-body]').length,
                  };
                })()
                """
            )
            if hierarchy:
                break
            time.sleep(0.05)

        self.assertIsNotNone(hierarchy)
        self.assertEqual(hierarchy["layout"], "direct-labels", hierarchy)
        self.assertGreaterEqual(hierarchy["logicalWidth"], 1040, hierarchy)
        self.assertEqual(hierarchy["rotorThrusts"], 4, hierarchy)
        self.assertEqual(hierarchy["aggregateThrusts"], 1, hierarchy)
        self.assertEqual(hierarchy["weights"], 1, hierarchy)
        self.assertEqual(hierarchy["drags"], 1, hierarchy)
        self.assertGreaterEqual(hierarchy["forceLabels"], 4, hierarchy)
        self.assertEqual(hierarchy["droneBodies"], 1, hierarchy)

    def test_slide_10_yaw_diagram_is_top_down_and_pair_driven(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#10"},
        )
        deadline = time.monotonic() + 4.0
        yaw = None
        while time.monotonic() < deadline:
            yaw = self._evaluate(
                """
                (() => {
                  if (document.readyState !== 'complete') return null;
                  const comparison = document.querySelector('[data-torque-comparison]');
                  const helicopter = comparison?.querySelector('svg[data-helicopter-torque]');
                  const figure = comparison?.querySelector('svg[data-control-mixing]');
                  if (!comparison || !helicopter || !figure) return null;
                  const rotors = [...figure.querySelectorAll('[data-yaw-rotor]')];
                  const motors = Object.fromEntries(rotors.map(rotor => [
                    rotor.dataset.motor,
                    {
                      spin: rotor.dataset.spin,
                      command: rotor.dataset.pairCommand,
                    },
                  ]));
                  const helicopterRect = helicopter.getBoundingClientRect();
                  const droneRect = figure.getBoundingClientRect();
                  return {
                    comparison: comparison.dataset.layout || null,
                    helicopterMainTorque: helicopter.querySelectorAll(
                      '[data-heli-torque="main"]'
                    ).length,
                    helicopterReactionTorque: helicopter.querySelectorAll(
                      '[data-heli-torque="reaction"]'
                    ).length,
                    helicopterTailCounterTorque: helicopter.querySelectorAll(
                      '[data-heli-torque="tail-counter"]'
                    ).length,
                    diagramsSideBySide:
                      helicopterRect.right < droneRect.left &&
                      Math.abs(helicopterRect.top - droneRect.top) < 4,
                    view: figure.dataset.view || null,
                    rotors: rotors.length,
                    motors,
                    circularRotors: rotors.filter(rotor => {
                      const rect = rotor.getBoundingClientRect();
                      return Math.abs(rect.width - rect.height) < 2;
                    }).length,
                    cw: figure.querySelectorAll('[data-spin="cw"]').length,
                    ccw: figure.querySelectorAll('[data-spin="ccw"]').length,
                    increase: figure.querySelectorAll(
                      '[data-pair-command="increase"]'
                    ).length,
                    decrease: figure.querySelectorAll(
                      '[data-pair-command="decrease"]'
                    ).length,
                    yawResult: figure.querySelector('[data-yaw-result]')?.dataset.direction || null,
                    body: figure.querySelectorAll('[data-yaw-body]').length,
                  };
                })()
                """
            )
            if (yaw):
                break
            time.sleep(0.05)

        self.assertIsNotNone(yaw)
        self.assertEqual(yaw["comparison"], "side-by-side", yaw)
        self.assertEqual(yaw["helicopterMainTorque"], 1, yaw)
        self.assertEqual(yaw["helicopterReactionTorque"], 1, yaw)
        self.assertEqual(yaw["helicopterTailCounterTorque"], 1, yaw)
        self.assertTrue(yaw["diagramsSideBySide"], yaw)
        self.assertEqual(yaw["view"], "top", yaw)
        self.assertEqual(yaw["rotors"], 4, yaw)
        self.assertEqual(
            yaw["motors"],
            {
                "M1": {"spin": "cw", "command": "decrease"},
                "M2": {"spin": "cw", "command": "decrease"},
                "M3": {"spin": "ccw", "command": "increase"},
                "M4": {"spin": "ccw", "command": "increase"},
            },
            yaw,
        )
        self.assertEqual(yaw["circularRotors"], 4, yaw)
        self.assertEqual(yaw["cw"], 2, yaw)
        self.assertEqual(yaw["ccw"], 2, yaw)
        self.assertEqual(yaw["increase"], 2, yaw)
        self.assertEqual(yaw["decrease"], 2, yaw)
        self.assertEqual(yaw["yawResult"], "cw", yaw)
        self.assertEqual(yaw["body"], 1, yaw)

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

    def test_active_video_autoplays_and_previous_video_resets(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#35"},
        )
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
