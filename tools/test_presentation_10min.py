#!/usr/bin/env python3
"""Contract tests for the technical 10-minute drone summary deck."""

from __future__ import annotations

import json
import re
import posixpath
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request
import zipfile
from fractions import Fraction
from html.parser import HTMLParser
from pathlib import Path
from xml.etree import ElementTree

from pptx import Presentation

try:
    import websocket
except ImportError:  # pragma: no cover - environment-dependent skip
    websocket = None


REPO_ROOT = Path(__file__).resolve().parents[1]
DECK_DIR = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone-10min"
HTML_PATH = DECK_DIR / "index.html"
PPTX_PATH = DECK_DIR / "드론_10분_요약본.pptx"
EXPECTED_SLIDES = 14
EXPECTED_VIDEOS = 1
SLIDE_NAME = re.compile(r"ppt/slides/slide\d+\.xml$")
NOTE_NAME = re.compile(r"ppt/notesSlides/notesSlide\d+\.xml$")
PRESENTATION_NS = {
    "p": "http://schemas.openxmlformats.org/presentationml/2006/main"
}
RELATIONSHIP_NS = {
    "r": "http://schemas.openxmlformats.org/package/2006/relationships"
}
VIDEO_REL = "http://schemas.openxmlformats.org/officeDocument/2006/relationships/video"
MEDIA_REL = "http://schemas.microsoft.com/office/2007/relationships/media"
CHROME_BIN = (
    shutil.which("google-chrome-stable")
    or shutil.which("google-chrome")
    or shutil.which("chromium")
)


def _unused_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


class _DeckParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.sections: list[dict[str, str | None]] = []
        self.videos: list[dict[str, str | None]] = []
        self.images: list[dict[str, str | None]] = []

    def handle_starttag(
        self, tag: str, attrs: list[tuple[str, str | None]]
    ) -> None:
        attributes = dict(attrs)
        if tag == "section":
            self.sections.append(attributes)
        elif tag == "video":
            self.videos.append(attributes)
        elif tag == "img":
            self.images.append(attributes)


class Presentation10MinuteHtmlTests(unittest.TestCase):
    def setUp(self) -> None:
        self.source = HTML_PATH.read_text(encoding="utf-8")
        self.parser = _DeckParser()
        self.parser.feed(self.source)

    def test_deck_has_fourteen_numbered_slides_with_speaker_notes(self) -> None:
        self.assertEqual(len(self.parser.sections), EXPECTED_SLIDES)
        self.assertEqual(
            [item.get("data-screen-label") for item in self.parser.sections],
            [f"{index:02d}" for index in range(1, EXPECTED_SLIDES + 1)],
        )
        self.assertTrue(
            all(item.get("data-speaker-notes", "").strip() for item in self.parser.sections)
        )

    def test_deck_avoids_team_dates_hashes_and_scheduled_duration_copy(self) -> None:
        folded = self.source.casefold()
        for forbidden in ("zetin", "커밋", "commit", "3시간", "10분 과정", "예상 시간"):
            self.assertNotIn(forbidden, folded)
        self.assertIsNone(re.search(r"\b20\d{2}[./-]\d{1,2}", self.source))
        self.assertIsNone(re.search(r"\b[0-9a-f]{7,40}\b", folded))
        self.assertNotIn("같은 시험", self.source)

    def test_sil_slide_uses_real_source_excerpt(self) -> None:
        self.assertIn(
            'arduino_fake::<span style="color:#9bdcff">pre_tick_hook</span> = [&amp;](uint32_t tick)',
            self.source,
        )
        self.assertIn("state, disturbanceAt(config, tick - 1U, state)", self.source)
        self.assertNotIn("integratePlant(motorOut)", self.source)

    def test_deck_embeds_one_muted_looping_real_flight_video(self) -> None:
        self.assertEqual(len(self.parser.videos), EXPECTED_VIDEOS)
        video = self.parser.videos[0]
        self.assertEqual(video.get("src"), "assets/hover_demo.mp4")
        for required in ("muted", "loop", "playsinline"):
            self.assertIn(required, video)
        self.assertTrue((DECK_DIR / "assets" / "hover_demo.mp4").is_file())

    def test_source_video_is_chrome_compatible_h264_30fps(self) -> None:
        video_path = DECK_DIR / "assets" / "hover_demo.mp4"
        result = subprocess.run(
            [
                "ffprobe", "-v", "error", "-select_streams", "v:0",
                "-show_entries", "stream=codec_name,width,height,avg_frame_rate,r_frame_rate",
                "-of", "default=noprint_wrappers=1", str(video_path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        stream = dict(
            line.split("=", 1)
            for line in result.stdout.splitlines()
            if "=" in line
        )
        self.assertEqual(stream.get("codec_name"), "h264")
        self.assertEqual((int(stream["width"]), int(stream["height"])), (1280, 720))
        self.assertEqual(Fraction(stream["avg_frame_rate"]), Fraction(30, 1))
        self.assertEqual(Fraction(stream["r_frame_rate"]), Fraction(30, 1))

    def test_all_local_image_assets_exist(self) -> None:
        missing = []
        for image in self.parser.images:
            source = image.get("src")
            if source and not source.startswith(("http://", "https://", "data:")):
                path = DECK_DIR / source
                if not path.is_file():
                    missing.append(source)
        self.assertEqual(missing, [])


class Presentation10MinutePptxTests(unittest.TestCase):
    def test_pptx_contains_fourteen_notes_and_one_h264_video(self) -> None:
        self.assertTrue(PPTX_PATH.is_file(), f"missing generated PPTX: {PPTX_PATH}")
        presentation = Presentation(str(PPTX_PATH))
        self.assertEqual(len(presentation.slides), EXPECTED_SLIDES)

        with zipfile.ZipFile(PPTX_PATH) as archive:
            self.assertIsNone(archive.testzip())
            names = archive.namelist()
            self.assertEqual(sum(bool(SLIDE_NAME.fullmatch(name)) for name in names), EXPECTED_SLIDES)
            self.assertEqual(sum(bool(NOTE_NAME.fullmatch(name)) for name in names), EXPECTED_SLIDES)

            root = ElementTree.fromstring(archive.read("ppt/presentation.xml"))
            slide_size = root.find("p:sldSz", PRESENTATION_NS)
            self.assertIsNotNone(slide_size)
            self.assertEqual(slide_size.get("cx"), "12192000")
            self.assertEqual(slide_size.get("cy"), "6858000")

            video_names = [
                name for name in names
                if name.startswith("ppt/media/") and name.lower().endswith(".mp4")
            ]
            self.assertEqual(len(video_names), EXPECTED_VIDEOS)

            video_relationships = []
            media_relationships = []
            for relationship_name in names:
                if not relationship_name.startswith("ppt/slides/_rels/"):
                    continue
                if not relationship_name.endswith(".rels"):
                    continue
                relationship_root = ElementTree.fromstring(
                    archive.read(relationship_name)
                )
                slide_part = relationship_name.replace("/_rels/", "/")
                slide_part = slide_part.removesuffix(".rels")
                slide_directory = posixpath.dirname(slide_part)
                for relationship in relationship_root.findall("r:Relationship", RELATIONSHIP_NS):
                    target = posixpath.normpath(
                        posixpath.join(slide_directory, relationship.get("Target", ""))
                    )
                    item = (relationship_name, target)
                    if relationship.get("Type") == VIDEO_REL:
                        video_relationships.append(item)
                    elif relationship.get("Type") == MEDIA_REL:
                        media_relationships.append(item)

            self.assertEqual(len(video_relationships), EXPECTED_VIDEOS)
            self.assertEqual(len(media_relationships), EXPECTED_VIDEOS)
            self.assertEqual(video_relationships[0][1], media_relationships[0][1])
            embedded_name = video_relationships[0][1]
            self.assertIn(embedded_name, video_names)
            self.assertEqual(
                archive.read(embedded_name),
                (DECK_DIR / "assets" / "hover_demo.mp4").read_bytes(),
            )
            with tempfile.TemporaryDirectory(prefix="drone-summary-pptx-") as temp_dir:
                video_path = Path(temp_dir) / "embedded.mp4"
                video_path.write_bytes(archive.read(embedded_name))
                result = subprocess.run(
                    [
                        "ffprobe", "-v", "error", "-select_streams", "v:0",
                        "-show_entries", "stream=codec_name,avg_frame_rate,r_frame_rate",
                        "-of", "default=noprint_wrappers=1", str(video_path),
                    ],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                stream = dict(
                    line.split("=", 1)
                    for line in result.stdout.splitlines()
                    if "=" in line
                )
                self.assertEqual(stream.get("codec_name"), "h264")
                self.assertEqual(Fraction(stream["avg_frame_rate"]), Fraction(30, 1))
                self.assertEqual(Fraction(stream["r_frame_rate"]), Fraction(30, 1))


@unittest.skipUnless(CHROME_BIN and websocket, "Chrome and websocket-client are required")
class Presentation10MinuteBrowserWrapTests(unittest.TestCase):
    """Visual regression coverage for whitespace-delimited text tokens."""

    server: subprocess.Popen[bytes]
    chrome: subprocess.Popen[bytes]
    ws: websocket.WebSocket
    command_id: int

    @classmethod
    def setUpClass(cls) -> None:
        cls.http_port = _unused_port()
        cls.debug_port = _unused_port()
        cls.runtime = tempfile.TemporaryDirectory(prefix="drone-summary-wrap-")
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
                "--hide-scrollbars",
                "--window-size=1280,720",
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
        cls._call(
            "Emulation.setDeviceMetricsOverride",
            {"width": 1280, "height": 720, "deviceScaleFactor": 1, "mobile": False},
        )
        cls._call("Page.navigate", {"url": f"http://127.0.0.1:{cls.http_port}/#1"})

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if cls._evaluate(
                "document.readyState === 'complete' && "
                "document.querySelector('deck-stage')?._slides?.length === 14 && "
                "document.fonts.status === 'loaded'"
            ):
                cls._evaluate(
                    "(() => { const stage = document.querySelector('deck-stage'); "
                    "stage.setAttribute('no-rail', ''); stage._fit(); return true; })()"
                )
                return
            time.sleep(0.05)
        raise RuntimeError("14-slide presentation did not become ready")

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
    def _evaluate(cls, expression: str):
        response = cls._call(
            "Runtime.evaluate", {"expression": expression, "returnByValue": True}
        )
        if "exceptionDetails" in response:
            raise RuntimeError(response["exceptionDetails"])
        return response["result"].get("value")

    def test_whitespace_delimited_tokens_do_not_split_across_visual_lines(self) -> None:
        splits = self._evaluate(
            """
            (() => {
              const stage = document.querySelector('deck-stage');
              const findings = [];
              const lineFor = (range) => [...range.getClientRects()]
                .filter((rect) => rect.width > 0 && rect.height > 0)
                .map((rect) => Math.round(rect.top * 10) / 10);
              for (const [index, slide] of stage._slides.entries()) {
                stage.goTo(index);
                const walker = document.createTreeWalker(slide, NodeFilter.SHOW_TEXT);
                for (let node = walker.nextNode(); node; node = walker.nextNode()) {
                  const parent = node.parentElement;
                  if (!parent || !/[가-힣]/.test(node.data)) continue;
                  const style = getComputedStyle(parent);
                  if (style.display === 'none' || style.visibility === 'hidden') continue;
                  for (const match of node.data.matchAll(/\\S+/g)) {
                    const token = match[0];
                    if (!/[가-힣]/.test(token)) continue;
                    const range = document.createRange();
                    range.setStart(node, match.index);
                    range.setEnd(node, match.index + token.length);
                    const visualLines = [...new Set(lineFor(range))];
                    if (visualLines.length > 1) {
                      findings.push({
                        slide: index + 1,
                        token,
                        lines: visualLines,
                        context: parent.textContent.trim().replace(/\\s+/g, ' ').slice(0, 100),
                      });
                    }
                  }
                }
              }
              return findings;
            })()
            """
        )
        self.assertEqual(splits, [], f"mid-token visual line breaks: {splits}")


if __name__ == "__main__":
    unittest.main()
