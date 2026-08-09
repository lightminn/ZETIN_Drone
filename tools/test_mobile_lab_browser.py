#!/usr/bin/env python3
"""Real-Chrome regressions for the dependency-free mobile drone lab."""

from __future__ import annotations

import base64
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request
from pathlib import Path

try:
    import websocket
except ImportError:  # pragma: no cover - environment-dependent skip
    websocket = None


REPO_ROOT = Path(__file__).resolve().parents[1]
DECK_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone"
LAB_DIR = DECK_DIR / "mobile-lab"
CHROME_BIN = shutil.which("google-chrome-stable") or shutil.which("google-chrome")
SCREENSHOT_DIR = Path(
    os.environ.get("MOBILE_LAB_SCREENSHOT_DIR", "/tmp/zetin-mobile-lab-browser-screenshots")
)


def _unused_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


SENSOR_AND_CLOCK_SHIM = r"""
(() => {
  const params = new URLSearchParams(location.search);
  const scenario = params.get('sensors');
  window.__permissionProbe = {orientation: 0, motion: 0, activations: []};

  const record = (kind, result) => {
    window.__permissionProbe[kind] += 1;
    window.__permissionProbe.activations.push(Boolean(navigator.userActivation?.isActive));
    return Promise.resolve(result);
  };

  if (scenario === 'insecure') {
    Object.defineProperty(window, 'isSecureContext', {value: false, configurable: true});
  }
  if (scenario === 'none' || scenario === 'insecure') {
    Object.defineProperty(window, 'DeviceOrientationEvent', {value: undefined, configurable: true});
    Object.defineProperty(window, 'DeviceMotionEvent', {value: undefined, configurable: true});
  } else if (scenario === 'granted' || scenario === 'denied') {
    class SyntheticOrientationEvent {}
    class SyntheticMotionEvent {}
    SyntheticOrientationEvent.requestPermission = () => record(
      'orientation', scenario === 'denied' ? 'denied' : 'granted'
    );
    SyntheticMotionEvent.requestPermission = () => record('motion', 'granted');
    Object.defineProperty(window, 'DeviceOrientationEvent', {
      value: SyntheticOrientationEvent,
      configurable: true,
    });
    Object.defineProperty(window, 'DeviceMotionEvent', {
      value: SyntheticMotionEvent,
      configurable: true,
    });
  }

  window.__emitOrientation = (beta, gamma) => {
    const event = new Event('deviceorientation');
    Object.defineProperties(event, {
      beta: {value: beta},
      gamma: {value: gamma},
    });
    window.dispatchEvent(event);
  };
  window.__emitMotion = (x, y, z) => {
    const event = new Event('devicemotion');
    Object.defineProperty(event, 'accelerationIncludingGravity', {
      value: {x, y, z},
    });
    window.dispatchEvent(event);
  };

  if (params.get('clock') === 'manual') {
    let now = 0;
    let nextId = 1;
    let callbacks = [];
    window.requestAnimationFrame = (callback) => {
      const id = nextId++;
      callbacks.push({id, callback});
      return id;
    };
    window.cancelAnimationFrame = (id) => {
      callbacks = callbacks.filter(item => item.id !== id);
    };
    window.__runFrames = (count, milliseconds = 1000 / 60) => {
      for (let index = 0; index < count; index += 1) {
        now += milliseconds;
        const pending = callbacks;
        callbacks = [];
        pending.forEach(item => item.callback(now));
      }
      return {now, queued: callbacks.length};
    };
  }
})();
"""


@unittest.skipUnless(CHROME_BIN and websocket, "Chrome and websocket-client are required")
class MobileLabBrowserTests(unittest.TestCase):
    server: subprocess.Popen[bytes]
    chrome: subprocess.Popen[bytes]
    ws: websocket.WebSocket
    command_id: int

    @classmethod
    def setUpClass(cls) -> None:
        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)
        cls.http_port = _unused_port()
        cls.debug_port = _unused_port()
        cls.runtime = tempfile.TemporaryDirectory(prefix="zetin-mobile-lab-browser-")
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
                "--disable-background-networking",
                f"--remote-debugging-port={cls.debug_port}",
                "--remote-allow-origins=*",
                f"--user-data-dir={Path(cls.runtime.name) / 'profile'}",
                "about:blank",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        deadline = time.monotonic() + 8.0
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
        cls._call("Network.enable")
        cls._call("Emulation.setTouchEmulationEnabled", {"enabled": True, "maxTouchPoints": 1})
        cls._call(
            "Page.addScriptToEvaluateOnNewDocument", {"source": SENSOR_AND_CLOCK_SHIM}
        )

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
        cls.ws.send(json.dumps({"id": cls.command_id, "method": method, "params": params or {}}))
        while True:
            message = json.loads(cls.ws.recv())
            if message.get("id") != cls.command_id:
                continue
            if "error" in message:
                raise RuntimeError(message["error"])
            return message.get("result", {})

    @classmethod
    def evaluate(cls, expression: str):
        response = cls._call(
            "Runtime.evaluate",
            {"expression": expression, "returnByValue": True, "awaitPromise": True},
        )
        if "exceptionDetails" in response:
            raise RuntimeError(response["exceptionDetails"])
        return response["result"].get("value")

    @classmethod
    def _wait_for(cls, expression: str, timeout: float = 4.0):
        deadline = time.monotonic() + timeout
        value = None
        while time.monotonic() < deadline:
            value = cls.evaluate(expression)
            if value:
                return value
            time.sleep(0.03)
        raise AssertionError(f"condition did not become true: {expression}; last={value!r}")

    @classmethod
    def _navigate(cls, page: str, *, width: int = 390, height: int = 844) -> None:
        cls._call(
            "Emulation.setDeviceMetricsOverride",
            {
                "width": width,
                "height": height,
                "deviceScaleFactor": 1,
                "mobile": True,
            },
        )
        cls._call("Network.setBlockedURLs", {"urls": []})
        cls._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{cls.http_port}/mobile-lab/{page}"},
        )
        cls._wait_for("document.readyState === 'complete'")
        cls._wait_for("Boolean(document.querySelector('main'))")

    @classmethod
    def _rect(cls, selector: str) -> dict:
        return cls.evaluate(
            """
            (() => {
              const element = document.querySelector(%s);
              if (!element) return null;
              const rect = element.getBoundingClientRect();
              return {left: rect.left, top: rect.top, width: rect.width, height: rect.height,
                      right: rect.right, bottom: rect.bottom};
            })()
            """
            % json.dumps(selector)
        )

    @classmethod
    def _click(cls, selector: str) -> None:
        rect = cls._rect(selector)
        if rect is None:
            raise AssertionError(f"missing click target {selector}")
        x = rect["left"] + rect["width"] / 2
        y = rect["top"] + rect["height"] / 2
        cls._call(
            "Input.dispatchMouseEvent",
            {"type": "mousePressed", "x": x, "y": y, "button": "left", "clickCount": 1},
        )
        cls._call(
            "Input.dispatchMouseEvent",
            {"type": "mouseReleased", "x": x, "y": y, "button": "left", "clickCount": 1},
        )

    @classmethod
    def _touch(cls, event_type: str, x: float | None = None, y: float | None = None) -> None:
        points = [] if event_type == "touchEnd" else [{"x": x, "y": y, "radiusX": 2, "radiusY": 2}]
        cls._call("Input.dispatchTouchEvent", {"type": event_type, "touchPoints": points})

    @classmethod
    def text(cls, selector: str) -> str:
        return cls.evaluate(
            f"document.querySelector({json.dumps(selector)})?.textContent?.trim() || ''"
        )

    @classmethod
    def _screenshot(cls, filename: str) -> Path:
        encoded = cls._call("Page.captureScreenshot", {"format": "png", "fromSurface": True})[
            "data"
        ]
        path = SCREENSHOT_DIR / filename
        path.write_bytes(base64.b64decode(encoded))
        return path

    def test_sensor_absence_and_insecure_context_offer_explained_touch_fallback(self) -> None:
        self._navigate("index.html?sensors=none")
        self._wait_for("document.fonts.status === 'loaded'")
        self.assertEqual(
            ["loaded", "loaded", "loaded"],
            self.evaluate(
                "[...document.fonts].filter(face => face.family.includes('Noto Sans CJK KR'))"
                ".map(face => face.status).sort()"
            ),
        )
        self.assertEqual("start", self.evaluate("document.querySelector('main').dataset.screen"))
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'permission'")
        self.assertIn("센서", self.text("[data-sensor-reason]"))
        self.assertGreaterEqual(self._rect('[data-action="touch-fallback"]')["height"], 48)
        self._click('[data-action="touch-fallback"]')
        self.assertEqual("touch", self.evaluate("document.querySelector('main').dataset.mode"))

        self._navigate("index.html?sensors=insecure")
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'permission'")
        self.assertIn("HTTPS", self.text("[data-sensor-reason]"))

    def test_granted_sensor_calibrates_and_updates_synthetic_telemetry(self) -> None:
        self._navigate("index.html?sensors=granted")
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'permission'")
        self.evaluate("__emitOrientation(10, -5); __emitMotion(0.5, -1, 9.8)")
        self._wait_for("document.querySelector('main').dataset.screen === 'calibration'")
        probe = self.evaluate("window.__permissionProbe")
        self.assertEqual(1, probe["orientation"])
        self.assertEqual(1, probe["motion"])
        self.assertTrue(all(probe["activations"]))

        self._click('[data-action="calibrate"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'imu'")
        self.assertEqual("motion", self.evaluate("document.querySelector('main').dataset.mode"))
        self.assertEqual("0.0°", self.text("[data-roll-value]"))
        self.assertEqual("0.0°", self.text("[data-pitch-value]"))

        self.evaluate("__emitOrientation(20, 15); __emitMotion(1.5, -2, 9.4)")
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '+3.6°'")
        self.assertEqual("+1.8°", self.text("[data-pitch-value]"))
        self.assertEqual("0.7", self.text("[data-ax-value]"))
        self.assertEqual("-1.2", self.text("[data-ay-value]"))
        self.assertEqual("9.7", self.text("[data-az-value]"))

    def test_denied_sensor_permission_is_calm_and_keeps_touch_route(self) -> None:
        self._navigate("index.html?sensors=denied")
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'permission'")
        self._wait_for("document.querySelector('[data-sensor-reason]').textContent.includes('거부')")
        probe = self.evaluate("window.__permissionProbe")
        self.assertEqual({"orientation": 1, "motion": 1}, {
            "orientation": probe["orientation"], "motion": probe["motion"]
        })
        self.assertTrue(all(probe["activations"]))
        self.assertIsNotNone(self._rect('[data-action="touch-fallback"]'))

    def test_pointer_challenge_offline_result_and_restart(self) -> None:
        self._navigate("index.html?sensors=none&clock=manual")
        self._click('[data-action="touch"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'imu'")
        zone = self._rect("[data-joystick]")
        center_x = zone["left"] + zone["width"] / 2
        center_y = zone["top"] + zone["height"] / 2
        self._touch("touchStart", center_x, center_y)
        self._touch("touchMove", zone["right"] - 8, zone["top"] + 8)
        self._wait_for("parseFloat(document.querySelector('[data-roll-value]').textContent) > 10")
        self.assertGreater(float(self.text("[data-pitch-value]").replace("°", "")), 10)

        self._click('[data-action="start-challenge"]')
        self.evaluate("__runFrames(90)")
        self._touch("touchEnd")
        self.evaluate("__runFrames(1205)")
        self._wait_for("document.querySelector('main').dataset.screen === 'result'")
        score_before = self.text("[data-result-score]")
        self.assertRegex(score_before, r"^\d{1,4}$")
        self.assertTrue(self.evaluate("Boolean(document.querySelector('[data-result]').dataset.submissionId)"))

        self._call("Network.setBlockedURLs", {"urls": ["*/api/scores"]})
        self._click('[data-action="submit-score"]')
        self._wait_for("document.querySelector('[data-submit-status]').textContent.includes('로컬 결과')")
        self.assertEqual(score_before, self.text("[data-result-score]"))

        self._click('[data-action="restart"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'challenge'")
        self.assertEqual("20.0", self.text("[data-challenge-time]"))
        self.assertEqual("0000", self.text("[data-live-score]"))
        self.assertEqual("0.0°", self.text("[data-roll-value]"))
        self.assertEqual("0.0°", self.text("[data-pitch-value]"))
        self.assertFalse(
            self.evaluate("Boolean(document.querySelector('[data-result]').dataset.submissionId)")
        )

    def test_presenter_uses_local_qr_and_survives_missing_score_api(self) -> None:
        self._navigate("presenter.html")
        self._wait_for("Boolean(document.querySelector('[data-qr] svg'))")
        student_url = self.evaluate("document.querySelector('[data-student-url]').value")
        self.assertTrue(student_url.endswith("/index.html"), student_url)
        self.assertNotIn("presenter.html", student_url)
        self._wait_for("document.querySelector('[data-board-status]').textContent.includes('선택 기능')")
        self.assertEqual("0", self.text("[data-score-count]"))
        resource_urls = self.evaluate("performance.getEntriesByType('resource').map(entry => entry.name)")
        self.assertTrue(all(url.startswith(f"http://127.0.0.1:{self.http_port}/") for url in resource_urls))

    def test_mobile_viewports_have_no_horizontal_overflow_or_clipped_actions(self) -> None:
        for width, height in ((360, 800), (390, 844)):
            with self.subTest(viewport=f"{width}x{height}"):
                self._navigate("index.html?sensors=none", width=width, height=height)
                self._click('[data-action="touch"]')
                self._wait_for("document.querySelector('main').dataset.screen === 'imu'")
                geometry = self.evaluate(
                    """
                    (() => {
                      const visible = element => {
                        const style = getComputedStyle(element);
                        return !element.hidden && style.display !== 'none' && style.visibility !== 'hidden'
                          && element.getClientRects().length > 0;
                      };
                      const buttons = [...document.querySelectorAll('button.primary')]
                        .filter(visible)
                        .map(button => {
                          const rect = button.getBoundingClientRect();
                          return {left: rect.left, right: rect.right, width: rect.width,
                                  height: rect.height};
                        });
                      const dock = document.querySelector('section:not([hidden]) .action-dock')
                        .getBoundingClientRect();
                      return {
                        scrollWidth: document.documentElement.scrollWidth,
                        innerWidth,
                        innerHeight,
                        buttons,
                        dock: {left: dock.left, right: dock.right, bottom: dock.bottom},
                        safety: document.querySelector('.safety-boundary').textContent.trim(),
                      };
                    })()
                    """
                )
                self.assertLessEqual(geometry["scrollWidth"], geometry["innerWidth"])
                self.assertTrue(geometry["buttons"])
                for button in geometry["buttons"]:
                    self.assertGreaterEqual(button["left"], 0)
                    self.assertLessEqual(button["right"], width)
                    self.assertGreaterEqual(button["height"], 48)
                if len(geometry["buttons"]) == 1:
                    self.assertGreaterEqual(geometry["buttons"][0]["width"], width - 28)
                self.assertGreaterEqual(geometry["dock"]["left"], 0)
                self.assertLessEqual(geometry["dock"]["right"], width)
                self.assertLessEqual(geometry["dock"]["bottom"], height)
                self.assertIn("실제 기체와 연결되지 않습니다", geometry["safety"])
                self._screenshot(f"student-{width}x{height}.png")

        self._navigate("presenter.html", width=390, height=844)
        self._wait_for("Boolean(document.querySelector('[data-qr] svg'))")
        self._screenshot("presenter-390x844.png")


if __name__ == "__main__":
    unittest.main()
