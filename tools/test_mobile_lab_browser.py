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
import threading
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
sys.path.insert(0, str(LAB_DIR))

from server import build_server  # noqa: E402


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
  const permissionResolvers = [];
  const sensorListeners = {
    deviceorientation: new Set(),
    devicemotion: new Set(),
  };
  const nativeAddEventListener = window.addEventListener.bind(window);
  const nativeRemoveEventListener = window.removeEventListener.bind(window);
  window.addEventListener = (type, listener, options) => {
    sensorListeners[type]?.add(listener);
    return nativeAddEventListener(type, listener, options);
  };
  window.removeEventListener = (type, listener, options) => {
    sensorListeners[type]?.delete(listener);
    return nativeRemoveEventListener(type, listener, options);
  };
  window.__sensorListenerCounts = () => ({
    orientation: sensorListeners.deviceorientation.size,
    motion: sensorListeners.devicemotion.size,
  });

  const record = (kind, result) => {
    window.__permissionProbe[kind] += 1;
    window.__permissionProbe.activations.push(Boolean(navigator.userActivation?.isActive));
    if (scenario === 'delayed-grant') {
      return new Promise(resolve => permissionResolvers.push(() => resolve(result)));
    }
    return Promise.resolve(result);
  };
  window.__resolvePermissions = () => {
    const pending = permissionResolvers.splice(0);
    pending.forEach(resolve => resolve());
    return pending.length;
  };

  if (scenario === 'insecure') {
    Object.defineProperty(window, 'isSecureContext', {value: false, configurable: true});
  }
  if (scenario === 'none' || scenario === 'insecure') {
    Object.defineProperty(window, 'DeviceOrientationEvent', {value: undefined, configurable: true});
    Object.defineProperty(window, 'DeviceMotionEvent', {value: undefined, configurable: true});
  } else if (scenario === 'granted' || scenario === 'denied' || scenario === 'delayed-grant') {
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

  if (params.get('scores') === 'hang') {
    const nativeFetch = window.fetch.bind(window);
    window.__scoreFetchProbe = {calls: 0, concurrent: 0, maxConcurrent: 0, aborts: 0};
    window.fetch = (input, options = {}) => {
      if (String(input) !== '/api/scores') return nativeFetch(input, options);
      const probe = window.__scoreFetchProbe;
      probe.calls += 1;
      probe.concurrent += 1;
      probe.maxConcurrent = Math.max(probe.maxConcurrent, probe.concurrent);
      return new Promise((resolve, reject) => {
        let settled = false;
        const abort = () => {
          if (settled) return;
          settled = true;
          probe.concurrent -= 1;
          probe.aborts += 1;
          reject(new DOMException('synthetic hung score request aborted', 'AbortError'));
        };
        if (options.signal?.aborted) abort();
        else options.signal?.addEventListener('abort', abort, {once: true});
      });
    };
  }

  if (params.get('clock') === 'manual') {
    const challengeSeeds = [0, 0xffffffff];
    const nativeGetRandomValues = crypto.getRandomValues.bind(crypto);
    window.__challengeSeedDrawCount = 0;
    Crypto.prototype.getRandomValues = function(values) {
      if (values instanceof Uint32Array && values.length === 1) {
        window.__challengeSeedDrawCount += 1;
        if (challengeSeeds.length > 0) {
          values[0] = challengeSeeds.shift();
          return values;
        }
      }
      return nativeGetRandomValues(values);
    };
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
    chrome: subprocess.Popen[bytes]
    ws: websocket.WebSocket
    command_id: int

    @classmethod
    def setUpClass(cls) -> None:
        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)
        cls.debug_port = _unused_port()
        cls.runtime = tempfile.TemporaryDirectory(prefix="zetin-mobile-lab-browser-")
        cls.server = build_server("127.0.0.1", 0, LAB_DIR)
        cls.server_thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.server_thread.start()
        cls.http_port = int(cls.server.server_address[1])
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
            cls._stop_chrome()
            cls._stop_server()
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
        cls._stop_chrome()
        cls._stop_server()
        if hasattr(cls, "runtime"):
            cls.runtime.cleanup()

    @classmethod
    def _stop_chrome(cls) -> None:
        process = getattr(cls, "chrome", None)
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)

    @classmethod
    def _stop_server(cls) -> None:
        server = getattr(cls, "server", None)
        if server is None:
            return
        server.shutdown()
        cls.server_thread.join(timeout=5)
        server.server_close()

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
    def _navigate(
        cls,
        page: str,
        *,
        width: int = 390,
        height: int = 844,
        blocked_urls: list[str] | None = None,
    ) -> None:
        cls._call(
            "Emulation.setDeviceMetricsOverride",
            {
                "width": width,
                "height": height,
                "deviceScaleFactor": 1,
                "mobile": True,
            },
        )
        cls._call("Network.setBlockedURLs", {"urls": blocked_urls or []})
        cls._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{cls.http_port}/{page}"},
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
    def _key(cls, event_type: str, key: str, code: str, virtual_key_code: int) -> None:
        cls._call(
            "Input.dispatchKeyEvent",
            {
                "type": event_type,
                "key": key,
                "code": code,
                "windowsVirtualKeyCode": virtual_key_code,
                "nativeVirtualKeyCode": virtual_key_code,
            },
        )

    @classmethod
    def _press_key(cls, key: str, code: str, virtual_key_code: int) -> None:
        cls._key("rawKeyDown", key, code, virtual_key_code)
        cls._key("keyUp", key, code, virtual_key_code)

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

    @classmethod
    def _post_score(cls, index: int, nickname: str, score: int) -> None:
        payload = {
            "submission_id": f"fedcba98-7654-4321-8abc-{index:012d}",
            "nickname": nickname,
            "score": score,
            "stability": score / 10,
            "duration_ms": 20_000,
            "mode": "touch",
        }
        request = urllib.request.Request(
            f"http://127.0.0.1:{cls.http_port}/api/scores",
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=5) as response:
            if response.status != 201:
                raise AssertionError(f"score submission failed: {response.status}")

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

    def test_granted_sensor_without_a_sample_falls_back_after_four_seconds(self) -> None:
        self._navigate("index.html?sensors=granted")
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.sensorState === 'waiting'")
        self._wait_for(
            "document.querySelector('main').dataset.sensorState === 'fallback'",
            timeout=5.5,
        )
        self.assertEqual("permission", self.evaluate("document.querySelector('main').dataset.screen"))
        self.assertIn("도착하지 않았습니다", self.text("[data-sensor-reason]"))
        self.assertIsNotNone(self._rect('[data-action="touch-fallback"]'))

    def test_delayed_sensor_grant_cannot_undo_touch_or_home_cancellation(self) -> None:
        self._navigate("index.html?sensors=delayed-grant")
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.sensorState === 'requesting'")
        self._click('[data-action="touch-fallback"]')
        self.assertEqual(2, self.evaluate("__resolvePermissions()"))
        time.sleep(0.1)
        self.assertEqual(
            {"screen": "imu", "mode": "touch"},
            self.evaluate(
                "({screen: document.querySelector('main').dataset.screen, "
                "mode: document.querySelector('main').dataset.mode})"
            ),
        )
        self.assertEqual(
            {"orientation": 0, "motion": 0}, self.evaluate("__sensorListenerCounts()")
        )

        self._navigate("index.html?sensors=delayed-grant")
        self._click('[data-action="sensor"]')
        self._wait_for("document.querySelector('main').dataset.sensorState === 'requesting'")
        self._click('[data-action="back-start"]')
        self.assertEqual(2, self.evaluate("__resolvePermissions()"))
        time.sleep(0.1)
        self.assertEqual(
            {"screen": "start", "mode": "none"},
            self.evaluate(
                "({screen: document.querySelector('main').dataset.screen, "
                "mode: document.querySelector('main').dataset.mode})"
            ),
        )
        self.assertEqual(
            {"orientation": 0, "motion": 0}, self.evaluate("__sensorListenerCounts()")
        )

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
        self._touch("touchEnd")
        self.evaluate("__runFrames(120)")
        first_attempt_attitude = (
            self.text("[data-roll-value]"),
            self.text("[data-pitch-value]"),
        )
        self.evaluate("__runFrames(1085)")
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
        self.assertEqual(2, self.evaluate("window.__challengeSeedDrawCount"))

        self.evaluate("__runFrames(120)")
        second_attempt_attitude = (
            self.text("[data-roll-value]"),
            self.text("[data-pitch-value]"),
        )
        self.assertNotEqual(first_attempt_attitude, second_attempt_attitude)

    def test_keyboard_joystick_focus_axes_keyup_and_blur_neutralization(self) -> None:
        self._navigate("index.html?sensors=none")
        self._click('[data-action="touch"]')
        self._wait_for("document.querySelector('main').dataset.screen === 'imu'")
        self._click("[data-horizon]")
        self._press_key("Tab", "Tab", 9)

        self.assertTrue(
            self.evaluate("document.activeElement === document.querySelector('[data-joystick]')")
        )
        self.assertEqual(0, self.evaluate("document.querySelector('[data-joystick]').tabIndex"))
        self.assertIn("Roll", self.evaluate("document.querySelector('[data-joystick]').ariaLabel"))
        self.assertIn("방향키", self.text("[data-joystick-instructions]"))
        self.assertTrue(
            self.evaluate(
                "document.getElementById(document.querySelector('[data-joystick]')"
                ".getAttribute('aria-describedby')) === "
                "document.querySelector('[data-joystick-instructions]')"
            )
        )
        self.assertFalse(
            self.evaluate(
                "Boolean(document.querySelector('[data-joystick-instructions]')"
                ".closest('[aria-hidden=\"true\"]'))"
            )
        )
        self.assertTrue(
            self.evaluate("document.querySelector('[data-joystick]').matches(':focus-visible')")
        )
        focus_outline = self.evaluate(
            "(() => { const style = getComputedStyle(document.querySelector('[data-joystick]')); "
            "return {style: style.outlineStyle, width: parseFloat(style.outlineWidth)}; })()"
        )
        self.assertEqual("solid", focus_outline["style"])
        self.assertGreaterEqual(focus_outline["width"], 3)
        self.assertEqual(
            "polite",
            self.evaluate("document.querySelector('.attitude-values').getAttribute('aria-live')"),
        )

        self._key("rawKeyDown", "ArrowRight", "ArrowRight", 39)
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '+20.0°'")
        self.assertEqual("0.0°", self.text("[data-pitch-value]"))
        self._key("keyUp", "ArrowRight", "ArrowRight", 39)
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '0.0°'")

        self._key("rawKeyDown", "ArrowRight", "ArrowRight", 39)
        self._key("rawKeyDown", "ArrowUp", "ArrowUp", 38)
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '+14.1°'")
        self.assertEqual("+14.1°", self.text("[data-pitch-value]"))
        self._key("keyUp", "ArrowRight", "ArrowRight", 39)
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '0.0°'")
        self.assertEqual("+20.0°", self.text("[data-pitch-value]"))
        self._key("keyUp", "ArrowUp", "ArrowUp", 38)

        self._key("rawKeyDown", "ArrowDown", "ArrowDown", 40)
        self._wait_for("document.querySelector('[data-pitch-value]').textContent.trim() === '-20.0°'")
        self._key("keyUp", "ArrowDown", "ArrowDown", 40)
        self._wait_for("document.querySelector('[data-pitch-value]').textContent.trim() === '0.0°'")

        self._key("rawKeyDown", "ArrowLeft", "ArrowLeft", 37)
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '-20.0°'")
        self._press_key("Tab", "Tab", 9)
        self._wait_for("document.querySelector('[data-roll-value]').textContent.trim() === '0.0°'")
        self.assertEqual("0.0°", self.text("[data-pitch-value]"))
        self.assertFalse(
            self.evaluate("document.activeElement === document.querySelector('[data-joystick]')")
        )

    def test_presenter_uses_local_qr_and_survives_missing_score_api(self) -> None:
        self._navigate("presenter.html", blocked_urls=["*/api/scores"])
        self._wait_for("Boolean(document.querySelector('[data-qr] svg'))")
        student_url = self.evaluate("document.querySelector('[data-student-url]').value")
        self.assertTrue(student_url.endswith("/index.html"), student_url)
        self.assertNotIn("presenter.html", student_url)
        self._wait_for("document.querySelector('[data-board-status]').textContent.includes('선택 기능')")
        self.assertEqual("0", self.text("[data-score-count]"))
        resource_urls = self.evaluate("performance.getEntriesByType('resource').map(entry => entry.name)")
        self.assertTrue(all(url.startswith(f"http://127.0.0.1:{self.http_port}/") for url in resource_urls))

    def test_presenter_renders_total_count_and_ordered_scores_from_product_server(self) -> None:
        self._post_score(1, "첫째", 1000)
        self._post_score(2, "둘째", 1000)
        self._post_score(3, "셋째", 999)
        self._navigate("presenter.html")
        self._wait_for("document.querySelector('[data-score-count]').textContent.trim() === '3'")
        disclaimer = self.text("[data-score-disclaimer]")
        self.assertIn("참가자 브라우저가 제출", disclaimer)
        self.assertIn("교육용 비공식", disclaimer)
        self.assertIn("실제 비행 성능", disclaimer)
        self.assertIn("검증된 측정값이 아닙니다", disclaimer)
        self.assertEqual(
            ["첫째", "둘째", "셋째"],
            self.evaluate(
                "[...document.querySelectorAll('[data-score-list] .score-name')]"
                ".map(element => element.textContent.trim())"
            ),
        )
        self.assertEqual(
            ["1000", "1000", "0999"],
            self.evaluate(
                "[...document.querySelectorAll('[data-score-list] .score-value')]"
                ".map(element => element.textContent.trim())"
            ),
        )

    def test_presenter_times_out_hung_polling_without_overlapping_requests(self) -> None:
        self._navigate("presenter.html?scores=hang")
        self._wait_for("window.__scoreFetchProbe?.aborts >= 1", timeout=7.0)
        probe = self.evaluate("window.__scoreFetchProbe")
        self.assertGreaterEqual(probe["calls"], 1)
        self.assertGreaterEqual(probe["aborts"], 1)
        self.assertEqual(1, probe["maxConcurrent"])
        self.assertIn("선택 기능", self.text("[data-board-status]"))

    def test_optional_nickname_warns_against_personal_info_before_public_display(self) -> None:
        for width, height in ((360, 800), (390, 844)):
            with self.subTest(viewport=f"{width}x{height}"):
                self._navigate("index.html?sensors=none", width=width, height=height)
                privacy = self.evaluate(
                    """
                    (() => {
                      const input = document.querySelector('[data-nickname]');
                      const warning = document.querySelector('#nickname-privacy');
                      if (!input || !warning) return null;
                      const style = getComputedStyle(warning);
                      const rect = warning.getBoundingClientRect();
                      const dock = document.querySelector('.action-dock').getBoundingClientRect();
                      return {
                        autocomplete: input.getAttribute('autocomplete'),
                        required: input.required,
                        fieldText: input.closest('label').textContent.trim(),
                        warningText: warning.textContent.trim(),
                        display: style.display,
                        visibility: style.visibility,
                        opacity: style.opacity,
                        rect: {left: rect.left, right: rect.right, width: rect.width,
                               height: rect.height, bottom: rect.bottom},
                        dockTop: dock.top,
                        scrollWidth: document.documentElement.scrollWidth,
                        innerWidth,
                      };
                    })()
                    """
                )
                self.assertIsNotNone(privacy)
                self.assertEqual("off", privacy["autocomplete"])
                self.assertFalse(privacy["required"])
                self.assertIn("선택", privacy["fieldText"])
                for phrase in ("실명", "연락처", "개인정보", "표시 이름과 점수", "발표자 화면", "공개"):
                    self.assertIn(phrase, privacy["warningText"])
                self.assertNotEqual("none", privacy["display"])
                self.assertNotEqual("hidden", privacy["visibility"])
                self.assertNotEqual("0", privacy["opacity"])
                self.assertGreater(privacy["rect"]["width"], 0)
                self.assertGreater(privacy["rect"]["height"], 0)
                self.assertGreaterEqual(privacy["rect"]["left"], 0)
                self.assertLessEqual(privacy["rect"]["right"], width)
                self.assertLessEqual(privacy["rect"]["bottom"], privacy["dockTop"])
                self.assertLessEqual(privacy["scrollWidth"], privacy["innerWidth"])
                self._screenshot(f"student-privacy-{width}x{height}.png")

    def test_mobile_viewports_have_no_horizontal_overflow_or_clipped_actions(self) -> None:
        def open_student_state(state: str, width: int, height: int) -> None:
            parameters = "sensors=none&clock=manual" if state == "result" else "sensors=none"
            if state in {"permission", "calibration"}:
                parameters = "sensors=granted"
            self._navigate(f"index.html?{parameters}", width=width, height=height)
            if state == "start":
                return
            if state == "permission":
                self._click('[data-action="sensor"]')
            elif state == "calibration":
                self._click('[data-action="sensor"]')
                self._wait_for("document.querySelector('main').dataset.screen === 'permission'")
                self.evaluate("__emitOrientation(0, 0)")
            else:
                self._click('[data-action="touch"]')
                self._wait_for("document.querySelector('main').dataset.screen === 'imu'")
                if state in {"challenge", "result"}:
                    self._click('[data-action="start-challenge"]')
                if state == "result":
                    self.evaluate("__runFrames(1205)")
            self._wait_for(
                f"document.querySelector('main').dataset.screen === {json.dumps(state)}"
            )

        for width, height in ((360, 800), (390, 844)):
            for state in ("start", "permission", "calibration", "imu", "challenge", "result"):
                with self.subTest(viewport=f"{width}x{height}", student_state=state):
                    open_student_state(state, width, height)
                    geometry = self.evaluate(
                        """
                        (() => {
                          const panel = document.querySelector('section[data-screen-panel]:not([hidden])');
                          const buttons = [...panel.querySelectorAll('button')].map(button => {
                            const rect = button.getBoundingClientRect();
                            return {left: rect.left, top: rect.top, right: rect.right,
                                    bottom: rect.bottom, width: rect.width, height: rect.height};
                          });
                          const dock = panel.querySelector('.action-dock').getBoundingClientRect();
                          const panelRect = panel.getBoundingClientRect();
                          return {
                            scrollWidth: document.documentElement.scrollWidth,
                            innerWidth,
                            innerHeight,
                            buttons,
                            dock: {left: dock.left, right: dock.right, bottom: dock.bottom},
                            panel: {left: panelRect.left, right: panelRect.right},
                            safety: document.querySelector('.safety-boundary').textContent.trim(),
                          };
                        })()
                        """
                    )
                    self.assertLessEqual(geometry["scrollWidth"], geometry["innerWidth"])
                    self.assertGreaterEqual(geometry["panel"]["left"], 0)
                    self.assertLessEqual(geometry["panel"]["right"], width)
                    for button in geometry["buttons"]:
                        self.assertGreaterEqual(button["left"], 0)
                        self.assertLessEqual(button["right"], width)
                        self.assertGreaterEqual(button["top"], 0)
                        self.assertLessEqual(button["bottom"], height)
                        self.assertGreaterEqual(button["height"], 48)
                    if len(geometry["buttons"]) == 1:
                        self.assertGreaterEqual(geometry["buttons"][0]["width"], width - 28)
                    self.assertGreaterEqual(geometry["dock"]["left"], 0)
                    self.assertLessEqual(geometry["dock"]["right"], width)
                    self.assertLessEqual(geometry["dock"]["bottom"], height)
                    self.assertIn("실제 기체와 연결되지 않습니다", geometry["safety"])
            self._screenshot(f"student-result-{width}x{height}.png")

        for width, height in ((360, 800), (390, 844)):
            with self.subTest(viewport=f"{width}x{height}", presenter=True):
                self._navigate(
                    "presenter.html",
                    width=width,
                    height=height,
                    blocked_urls=["*/api/scores"],
                )
                self._wait_for("Boolean(document.querySelector('[data-qr] svg'))")
                geometry = self.evaluate(
                    """
                    (() => {
                      const selectors = [
                        '.presenter-shell', '.presenter-grid', '.presenter-qr-panel',
                        '.leaderboard-panel', '[data-qr]',
                      ];
                      return {
                        scrollWidth: document.documentElement.scrollWidth,
                        innerWidth,
                        regions: selectors.map(selector => {
                          const rect = document.querySelector(selector).getBoundingClientRect();
                          return {selector, left: rect.left, right: rect.right};
                        }),
                        safety: document.querySelector('.safety-boundary').textContent.trim(),
                      };
                    })()
                    """
                )
                self.assertLessEqual(geometry["scrollWidth"], geometry["innerWidth"])
                for region in geometry["regions"]:
                    self.assertGreaterEqual(region["left"], 0, region["selector"])
                    self.assertLessEqual(region["right"], width, region["selector"])
                self.assertIn("실제 기체와 연결되지 않습니다", geometry["safety"])
                for selector in ('[data-action="update-qr"]', '[data-action="copy-url"]'):
                    rect = self.evaluate(
                        """
                        (() => {
                          const button = document.querySelector(%s);
                          button.scrollIntoView({block: 'center'});
                          const rect = button.getBoundingClientRect();
                          return {left: rect.left, top: rect.top, right: rect.right,
                                  bottom: rect.bottom, height: rect.height};
                        })()
                        """
                        % json.dumps(selector)
                    )
                    self.assertGreaterEqual(rect["left"], 0)
                    self.assertLessEqual(rect["right"], width)
                    self.assertGreaterEqual(rect["top"], 0)
                    self.assertLessEqual(rect["bottom"], height)
                    self.assertGreaterEqual(rect["height"], 48)
                self._screenshot(f"presenter-{width}x{height}.png")


if __name__ == "__main__":
    unittest.main()
