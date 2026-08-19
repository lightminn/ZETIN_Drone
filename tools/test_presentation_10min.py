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
SCRIPT_PATH = DECK_DIR / "SCRIPT.md"
PPTX_PATH = DECK_DIR / "드론_10분_요약본.pptx"
PRODUCTION_ESTIMATE_PATH = (
    REPO_ROOT
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "production_estimate.json"
)
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
    def test_ten_minute_script_matches_the_current_deck_and_time_budget(self) -> None:
        script = SCRIPT_PATH.read_text(encoding="utf-8")
        normalized = re.sub(r"\s+", " ", script)
        self.assertEqual(
            [int(number) for number in re.findall(r"^## (\d+)쪽", script, re.MULTILINE)],
            list(range(1, 15)),
        )
        self.assertGreaterEqual(len(script), 4500)
        self.assertLessEqual(len(script), 5500)
        for phrase in (
            "직접 설계한 범위와 조달한 범위",
            "설계 → 출력 → 시험·파손 → 측정 → 개선",
            "3901-L0X",
            "BMS 상태 공유와 충전 도크",
            "sim-to-real 경로 계획",
            "로봇개나 지상 로봇",
            "유선 드론",
            "다기체 군집",
            "통신 이상",
            "현재 성과가 아니라 후속 계획",
            "RC 안전 전환 로직은 host에서 확인",
        ):
            self.assertIn(phrase, normalized)
        self.assertNotIn("네 ESC의 BEC 출력을 모두 병렬", normalized)
        for segmented_label in ("핵심 발화", "확장 발화", "선택 발화"):
            self.assertNotIn(segmented_label, script)

    def setUp(self) -> None:
        self.source = HTML_PATH.read_text(encoding="utf-8")
        self.parser = _DeckParser()
        self.parser.feed(self.source)

    def slide_source(self, screen_label: str) -> str:
        match = re.search(
            rf'<section\b(?=[^>]*data-screen-label="{re.escape(screen_label)}")[^>]*>'
            r".*?</section>",
            self.source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match, f"missing slide {screen_label}")
        return match.group(0)

    def test_deck_has_fourteen_numbered_slides_with_speaker_notes(self) -> None:
        self.assertEqual(len(self.parser.sections), EXPECTED_SLIDES)
        self.assertEqual(
            [item.get("data-screen-label") for item in self.parser.sections],
            [f"{index:02d}" for index in range(1, EXPECTED_SLIDES + 1)],
        )
        self.assertTrue(
            all(item.get("data-speaker-notes", "").strip() for item in self.parser.sections)
        )

    def test_deck_identity_is_a_results_presentation(self) -> None:
        self.assertIn("<title>자작 드론 비행 제어 개발 결과</title>", self.source)
        slide = self.slide_source("01")
        self.assertIn('title="자작 드론\n비행 제어 개발 결과"', slide)
        self.assertIn('subtitle="기체·제어 보드·펌웨어·검증"', slide)

    def test_technical_slide_titles_are_short_and_natural(self) -> None:
        expected_titles = {
            "03": "설계한 하드웨어와 조달 부품",
            "05": "쿼드콥터의 힘과 토크",
            "06": "자이로와 가속도계 융합",
            "08": "실제 펌웨어 기반 SIL",
            "09": "캐스케이드 자세 제어",
            "10": "Yaw 기준과 지자기 보정",
            "11": "조종 신호 두절 시 안전 전환",
            "12": "테더 자세 제어 시험",
        }
        for screen_label, expected_title in expected_titles.items():
            with self.subTest(slide=screen_label):
                slide = self.slide_source(screen_label)
                self.assertIn(f'title="{expected_title}"', slide)

        slide_08 = self.slide_source("08")
        visible_markup = slide_08.split(">", 1)[1]
        self.assertNotIn("1ms", visible_markup)

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

    def test_slide_02_distinguishes_operational_products_from_learning_system(self) -> None:
        slide = self.slide_source("02")
        for required in (
            'title="상용 제품과 자작 학습"',
            "현장 운용",
            "학습·검증",
            "오류 주입",
            "디버깅",
            "우열",
        ):
            self.assertIn(required, slide)
        self.assertIn("data-teamless-crop", slide)
        self.assertNotIn("왜 비행제어 컴퓨터까지 직접 만들었나", slide)

    def test_slide_03_shows_designed_hardware_integrated_parts_and_power_path(self) -> None:
        slide = self.slide_source("03")
        rendered = slide.split(">", 1)[1]
        for required in (
            'src="assets/assembled-bench.jpeg"',
            "직접 설계",
            "모듈형 프레임 · 비행제어 PCB",
            "조달·통합",
            "모터 4개 · ESC 4개",
            "ESP32-S3 · 듀얼 IMU · BMM350",
            "실제 전원 경로",
            "12V → ESC BEC → 5V → ESP32-S3 → 3.3V → 센서",
        ):
            self.assertIn(required, rendered)
        for absent in ("보호용 MOSFET", "5V 레귤레이터"):
            self.assertNotIn(absent, slide)

    def test_slide_04_presents_rapid_prototyping_and_modular_repair(self) -> None:
        slide = self.slide_source("04")
        for required in (
            'title="래피드 프로토타이핑"',
            'src="assets/cad-top.png"',
            'src="assets/frame-iterations.jpeg"',
            'src="assets/modular-arm.png"',
            "설계 → 출력 → 시험·파손 → 측정 → 개선",
            "모듈형 암",
            "손상된 부분만 다시 출력해 교체",
            "Maker Space 박근원 선생님의 장비·제작 지원에 감사드립니다.",
            "프린터 1대",
            "프린터 1대 기준",
            "직접 작업 6–10시간",
            "모터 4개",
            "8.0만 원",
            "ESC 4개",
            "4.0만 원",
            "22.2–23.3만 원",
            "직접 작업 60–100시간",
            "222–233만 원",
            "3901-L0X 옵션",
            "+4.8–4.9만 원/대",
            "기본 합계에서 제외",
        ):
            self.assertIn(required, slide)

        visible_markup = slide.split(">", 1)[1]
        for absent in (
            "인시",
            "계획 추정",
            "참고 품목 합계",
            "완성기 총액",
            "확정 BOM",
            "사용자 확인 프로젝트 추정",
        ):
            self.assertNotIn(absent, visible_markup)

    def test_slide_04_uses_user_confirmed_cost_estimate(self) -> None:
        slide = self.slide_source("04")
        rendered = slide.split(">", 1)[1]
        for expected in (
            "모터 4개",
            "8.0만 원",
            "ESC 4개",
            "4.0만 원",
            "기본 합계",
            "1대 22.2–23.3만 원",
            "10대 222–233만 원",
            "3901-L0X 옵션",
            "+4.8–4.9만 원/대",
            "기본 합계에서 제외",
        ):
            self.assertIn(expected, rendered)
        for stale in (
            "7.8–8.2만 원",
            "17.5만 원",
            "35.5–37.1만 원",
            "355–371만 원",
        ):
            self.assertNotIn(stale, slide)

        notes = re.search(r'data-speaker-notes="([^"]+)"', slide)
        self.assertIsNotNone(notes)
        self.assertIn("현재 부품비 산정", notes.group(1))
        self.assertIn("221,900~233,100원", notes.group(1))
        self.assertIn("2,219,000~2,331,000원", notes.group(1))
        self.assertIn("48,000~49,000원", notes.group(1))
        self.assertIn("기본 합계에서 제외", notes.group(1))

    def test_slide_04_time_values_follow_production_estimate_json(self) -> None:
        estimate = json.loads(PRODUCTION_ESTIMATE_PATH.read_text(encoding="utf-8"))
        slide = self.slide_source("04")
        rendered = slide.split(">", 1)[1]

        one_unit_time = estimate["time"]["one_unit"]
        ten_unit_time = estimate["time"]["ten_units"]
        self.assertIn(
            f"프린터 점유 {one_unit_time['printer_hours'][0]}–"
            f"{one_unit_time['printer_hours'][1]}시간",
            rendered,
        )
        self.assertIn(
            f"직접 작업 {one_unit_time['hands_on_hours'][0]}–"
            f"{one_unit_time['hands_on_hours'][1]}시간",
            rendered,
        )
        self.assertIn(
            f"프린터 점유 {ten_unit_time['printer_hours_one_printer'][0]}–"
            f"{ten_unit_time['printer_hours_one_printer'][1]}시간",
            rendered,
        )
        self.assertIn(
            f"직접 작업 {ten_unit_time['hands_on_hours'][0]}–"
            f"{ten_unit_time['hands_on_hours'][1]}시간",
            rendered,
        )

    def test_slide_07_separates_actual_axis_debugging_from_sil_mixer_mutation(self) -> None:
        slide = self.slide_source("07")
        for required in (
            'title="실제 디버깅과 SIL 검출력"',
            "실제 디버깅",
            "센서축 ↔ 기체축 부호",
            "두 IMU 원시값 비교",
            "host SIL 검출력",
            "Roll R → −R",
            "정상 조건 수렴",
            "별도 오류 주입",
            "R → −R",
            "같은 사건을 재현한 것이 아니라",
        ):
            self.assertIn(required, slide)
        visible_markup = slide.split(">", 1)[1]
        self.assertNotIn("SIL_INJECT_", visible_markup)
        self.assertNotIn("자이로 부호를 반전", slide)
        self.assertNotIn("같은 결함 재현", slide)

    def test_slide_09_explains_relative_loop_rates_without_fixed_numbers(self) -> None:
        slide = self.slide_source("09")
        for required in ("바깥 자세 루프", "안쪽 각속도 루프", "더 빠르게", "현행 구현값"):
            self.assertIn(required, slide)
        for forbidden in ("250Hz", "1kHz", "250 Hz", "1 kHz"):
            self.assertNotIn(forbidden, slide)

    def test_slide_11_uses_condition_based_safety_and_evidence_boundaries(self) -> None:
        slide = self.slide_source("11")
        for required in (
            "신호가 유효",
            "신호 두절",
            "호버 추정 없음",
            "저스로틀",
            "호버 추정 유효",
            "저스로틀 초과",
            'src="assets/landing-probe-evidence.png"',
            "공중 2회 0.061·0.097g",
            "접지 10회 0.059~1.147g",
            "기록만 하고",
            "착지 판정에서는 제외",
            "landed=false",
            "host와 SIL",
            "보드에서",
            "착지 판정",
        ):
            self.assertIn(required, slide)
        self.assertNotIn("500ms", slide)
        self.assertNotIn("WDT panic 재부팅 후", slide)
        self.assertNotIn("지상인가", slide)
        self.assertNotIn("공중인가", slide)

    def test_slide_12_uses_full_tether_motor_means_and_scoped_interpretation(self) -> None:
        slide = self.slide_source("12")
        for required in (
            'src="assets/telemetry-motor-balance.png"',
            "2,135행",
            "M3는 1360.0마이크로초",
            "M1은 1334.2마이크로초",
            "25.8마이크로초",
            "M3 쪽에 연결된 테더 줄",
            "확정 원인으로 단정하지 않는다",
        ):
            self.assertIn(required, slide)
        rendered = slide.split(">", 1)[1]
        self.assertIn("M3 평균이 M1보다 25.8µs 높았다", rendered)
        self.assertNotIn('src="assets/chart-attitude.png"', rendered)

    def test_slide_13_separates_short_mid_and_long_term_goals(self) -> None:
        slide = self.slide_source("13")
        for required in (
            'title="지상 로봇과 함께 쓰는 다음 단계"',
            "단기",
            "중기",
            "장기",
            "단일 기체 기준선",
            "3901-L0X 거리·광류 제어 연결",
            "배터리 전압·전류·온도 계측",
            "제작·충전 인프라",
            "BMS 상태 공유·충전 도크 검증",
            "sim-to-real 경로 검증",
            "지상 로봇 협업",
            "로봇개 상부에 드론을 싣고 이동",
            "작업 지점에서 유선 드론 전개",
            "군집·충돌 회피·통신 이상 안전",
            "경량화·저가화",
        ):
            self.assertIn(required, slide)
        rendered = slide.split(">", 1)[1]
        self.assertEqual(rendered.count("후속 계획"), 1)
        self.assertNotIn("현재 검증 단계", rendered)

    def test_slide_14_is_a_clean_thanks_and_questions_close(self) -> None:
        slide = self.slide_source("14")
        for required in (
            'title="감사합니다"',
            "Q &amp; A",
            "질문을 받겠습니다.",
        ):
            self.assertIn(required, slide)
        for absent in ("만든 것과 남긴 것", "하드웨어", "소프트웨어", "자유비행"):
            self.assertNotIn(absent, slide)

    def test_audience_terms_use_plain_korean(self) -> None:
        self.assertNotIn("지상국", self.source)
        self.assertNotIn("센서를 오염", self.source)


class Presentation10MinutePptxTests(unittest.TestCase):
    """Structural and content checks for the current PPTX delivery artifact."""

    def test_current_pptx_contains_latest_notes_and_one_h264_video(self) -> None:
        self.assertTrue(PPTX_PATH.is_file(), f"missing generated PPTX: {PPTX_PATH}")
        presentation = Presentation(str(PPTX_PATH))
        self.assertEqual(len(presentation.slides), EXPECTED_SLIDES)

        with zipfile.ZipFile(PPTX_PATH) as archive:
            self.assertIsNone(archive.testzip())
            names = archive.namelist()
            self.assertEqual(sum(bool(SLIDE_NAME.fullmatch(name)) for name in names), EXPECTED_SLIDES)
            self.assertEqual(sum(bool(NOTE_NAME.fullmatch(name)) for name in names), EXPECTED_SLIDES)

            notes_text = " ".join(
                element.text or ""
                for name in names
                if NOTE_NAME.fullmatch(name)
                for element in ElementTree.fromstring(archive.read(name)).iter()
                if element.tag.endswith("}t")
            )
            for current_phrase in (
                "12V 입력에서 ESC의 BEC를 거쳐 5V",
                "모듈형 프레임과 비행제어 PCB",
                "모터 4개 80,000원, ESC 4개 40,000원",
                "접지 10회는 0.059~1.147g",
                "M3는 1360.0마이크로초, M1은 1334.2마이크로초",
                "로봇개 또는 지상 로봇이 드론을 싣고 작업 지점까지 이동",
                "발표를 마치고 질문을 받는다.",
            ):
                self.assertIn(current_phrase, notes_text)

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

    def test_reworked_result_slides_fit_the_1280_by_720_content_frame(self) -> None:
        result = self._evaluate(
            """
            (() => {
              const stage = document.querySelector('deck-stage');
              const targets = [2, 3, 4, 7, 8, 9, 11, 12, 13, 14];
              const findings = {};
              for (const number of targets) {
                stage.goTo(number - 1);
                stage._fit();
                const slide = stage._slides[number - 1];
                const slideRect = slide.getBoundingClientRect();
                const marker = slide.querySelector('[data-result-refresh]');
                const elements = [...slide.querySelectorAll('[data-result-element]')];
                const overflow = elements.flatMap(element => {
                  const rect = element.getBoundingClientRect();
                  return rect.left < slideRect.left - 1 || rect.top < slideRect.top - 1
                    || rect.right > slideRect.right + 1 || rect.bottom > slideRect.bottom + 1
                    ? [{text: element.textContent.trim().slice(0, 70)}]
                    : [];
                });
                const smallText = elements.flatMap(element => {
                  const style = getComputedStyle(element);
                  const hasOwnText = [...element.childNodes].some(node =>
                    node.nodeType === Node.TEXT_NODE && node.textContent.trim()
                  );
                  const size = Number.parseFloat(style.fontSize);
                  return hasOwnText && size < 18.7
                    ? [{text: element.textContent.trim().slice(0, 70), size}]
                    : [];
                });
                findings[number] = {
                  marker: marker?.dataset.resultRefresh || null,
                  markedElements: elements.length,
                  overflow,
                  smallText,
                };
              }
              return findings;
            })()
            """
        )
        for number in (2, 3, 4, 7, 8, 9, 11, 12, 13, 14):
            with self.subTest(slide=number):
                item = result[str(number)]
                self.assertEqual(item["marker"], str(number), result)
                self.assertGreater(item["markedElements"], 0, result)
                self.assertEqual(item["overflow"], [], result)
                self.assertEqual(item["smallText"], [], result)

    def test_real_hover_video_autoplays_and_resets_after_leaving_slide(self) -> None:
        self._evaluate("document.querySelector('deck-stage').goTo(11)")
        deadline = time.monotonic() + 4.0
        active = None
        while time.monotonic() < deadline:
            active = self._evaluate(
                """
                (() => {
                  const video = document.querySelector(
                    'video[src$="hover_demo.mp4"]'
                  );
                  if (!video) return null;
                  return {
                    paused: video.paused,
                    currentTime: video.currentTime,
                    muted: video.muted,
                    controls: video.controls,
                    loop: video.loop,
                    playsInline: video.playsInline,
                    readyState: video.readyState,
                  };
                })()
                """
            )
            if active and not active["paused"] and active["currentTime"] > 0.2:
                break
            time.sleep(0.05)

        self.assertIsNotNone(active)
        self.assertFalse(active["paused"], active)
        self.assertGreater(active["currentTime"], 0.2, active)
        self.assertTrue(active["muted"], active)
        self.assertTrue(active["controls"], active)
        self.assertTrue(active["loop"], active)
        self.assertTrue(active["playsInline"], active)

        self._evaluate("document.querySelector('deck-stage').goTo(12)")
        deadline = time.monotonic() + 2.0
        previous = None
        while time.monotonic() < deadline:
            previous = self._evaluate(
                """
                (() => {
                  const video = document.querySelector(
                    'video[src$="hover_demo.mp4"]'
                  );
                  return video && {
                    paused: video.paused,
                    currentTime: video.currentTime,
                  };
                })()
                """
            )
            if previous and previous["paused"] and previous["currentTime"] < 0.05:
                break
            time.sleep(0.05)

        self.assertIsNotNone(previous)
        self.assertTrue(previous["paused"], previous)
        self.assertLess(previous["currentTime"], 0.05, previous)


if __name__ == "__main__":
    unittest.main()
