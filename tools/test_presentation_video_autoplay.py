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
    31: "mixer-saturation.mp4",
    33: "cascade-loop-timing.mp4",
    38: "accelerometer-confidence.mp4",
    55: "gravity-yaw-observability.mp4",
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
    64: "failsafe-timeline.png",
    70: "landing-probe-evidence.png",
    72: "shared-state-race.png",
    80: "telemetry-motor-balance.png",
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
                "elements": [],
                "text": [],
            }
            self.sections.append(self.current_section)
        if self.current_section is not None:
            self.current_section["elements"].append((tag, attributes))

        if tag == "video" and self.current_section is not None:
            self.current_section["videos"].append(attributes)
        elif tag == "img" and self.current_section is not None:
            self.current_section["images"].append(attributes)

    def handle_endtag(self, tag: str) -> None:
        if tag == "section":
            self.current_section = None

    def handle_data(self, data: str) -> None:
        if self.current_section is not None and data.strip():
            self.current_section["text"].append(data.strip())


def _slide_text(section: dict[str, object]) -> str:
    return " ".join(str(item) for item in section["text"])


def _slide_title(section: dict[str, object]) -> str | None:
    for tag, attrs in section["elements"]:
        if tag == "x-import" and attrs.get("title"):
            return attrs["title"]
    return None


class PresentationVideoMarkupTests(unittest.TestCase):
    def test_three_session_run_of_show_uses_two_real_break_slides(self) -> None:
        parser = _DeckParser()
        parser.feed((DECK_DIR / "index.html").read_text(encoding="utf-8"))

        self.assertEqual(len(parser.sections), 84)
        self.assertEqual(
            [section["attrs"].get("data-screen-label") for section in parser.sections],
            [f"{number:02d}" for number in range(1, 85)],
        )
        self.assertTrue(
            all(section["attrs"].get("data-speaker-notes") for section in parser.sections)
        )
        self.assertEqual(
            [
                section["attrs"].get("data-session-break")
                for section in parser.sections
                if section["attrs"].get("data-session-break")
            ],
            ["1", "2"],
        )

        agenda = parser.sections[1]
        agenda_visible = " ".join(
            filter(None, (_slide_title(agenda), _slide_text(agenda)))
        )
        for phrase in (
            "첫째 교시",
            "사업성·의의 비행 원리·CAD",
            "둘째 교시",
            "PCB · 배선 · 센서 융합 · 제어",
            "셋째 교시",
            "검증 · 안전 · 시연 · 모바일 체험",
            "질문과 답변",
        ):
            self.assertIn(phrase, agenda_visible)
        self.assertNotRegex(agenda_visible, r"(?:2\s*시간|40\s*분|10\s*분)")

        agenda_notes = str(agenda["attrs"].get("data-speaker-notes", ""))
        for phrase in (
            "전체 진행은 2시간 30분",
            "첫째 교시는 40분",
            "10분 휴식 후 둘째 교시",
            "다시 10분 휴식 후 셋째 교시",
            "발표 종료 뒤 10분",
        ):
            self.assertIn(phrase, agenda_notes)

        first_break = parser.sections[26]
        self.assertEqual(first_break["attrs"].get("data-session-break"), "1")
        first_break_visible = " ".join(
            filter(None, (_slide_title(first_break), _slide_text(first_break)))
        )
        for phrase in (
            "잠시 쉽니다",
            "둘째 교시",
            "PCB·배선에서 제어 소프트웨어까지",
        ):
            self.assertIn(phrase, first_break_visible)
        self.assertIn(
            "첫째 교시가 끝났다. 10분 휴식 후",
            str(first_break["attrs"].get("data-speaker-notes", "")),
        )

        second_break = parser.sections[53]
        self.assertEqual(second_break["attrs"].get("data-session-break"), "2")
        second_break_visible = " ".join(
            filter(None, (_slide_title(second_break), _slide_text(second_break)))
        )
        for phrase in (
            "잠시 쉽니다",
            "셋째 교시",
            "검증·안전·시연·모바일 체험",
        ):
            self.assertIn(phrase, second_break_visible)
        self.assertIn(
            "둘째 교시가 끝났다. 10분 휴식 후",
            str(second_break["attrs"].get("data-speaker-notes", "")),
        )

        self.assertEqual(_slide_title(parser.sections[27]), "비행제어 보드 회로")
        self.assertEqual(_slide_title(parser.sections[32]), "제어 소프트웨어 구성")
        self.assertEqual(_slide_title(parser.sections[54]), "Yaw의 기준 문제")
        self.assertEqual(_slide_title(parser.sections[74]), "시연 안전 규칙")
        self.assertEqual(_slide_title(parser.sections[2]), "사업성·의의\n비행제어기를 직접 만든 이유")
        self.assertEqual(_slide_title(parser.sections[19]), "기체 설계\nCAD에서 실물 프레임까지")
        self.assertEqual(parser.sections[2]["attrs"].get("data-label"), "사업성·의의")
        self.assertEqual(parser.sections[19]["attrs"].get("data-label"), "기체 설계")

        closing = parser.sections[83]
        closing_visible = " ".join(
            filter(None, (_slide_title(closing), _slide_text(closing)))
        )
        self.assertIn("감사합니다", closing_visible)
        self.assertIn("Q & A", closing_visible)
        self.assertNotIn("기체 관람", closing_visible)
        self.assertEqual(closing["videos"], [])
        self.assertIn(
            "마지막 10분은 질문과 답변 시간이다.",
            str(closing["attrs"].get("data-speaker-notes", "")),
        )

    def test_session_one_script_covers_all_slides_and_business_half(self) -> None:
        script = (DECK_DIR / "SESSION1_SCRIPT.md").read_text(encoding="utf-8")
        self.assertEqual(
            [int(number) for number in re.findall(r"^## (\d+)쪽", script, re.MULTILINE)],
            list(range(1, 27)),
        )
        for phrase in (
            "진행 목표는 39분",
            "사업성·의의에 약 20분",
            "비행 원리·CAD에 약 19분",
            "학교, 캠프, 메이커스페이스",
            "진행자 동반형 수업",
            "강사용 운영 자료",
            "기관의 시간·공간·예산 배정 의사",
            "마지막 1분",
        ):
            self.assertIn(phrase, script)
        self.assertGreaterEqual(len(script), 19000)
        self.assertLessEqual(len(script), 20500)
        for segmented_label in ("핵심 발화", "확장 발화", "선택 발화"):
            self.assertNotIn(segmented_label, script)
        for unsupported_claim in ("시장 규모는", "예상 매출은", "투자 수익률"):
            self.assertNotIn(unsupported_claim, script)

    def test_session_two_script_covers_pcb_sensors_sil_and_pid(self) -> None:
        script = (DECK_DIR / "SESSION2_SCRIPT.md").read_text(encoding="utf-8")
        normalized = re.sub(r"\s+", " ", script)
        self.assertEqual(
            [int(number) for number in re.findall(r"^## (\d+)쪽", script, re.MULTILINE)],
            list(range(28, 54)),
        )
        for phrase in (
            "PCB·배선에 약 10분",
            "센서 융합·자세 제어에 약 29분",
            "12볼트",
            "ESC의 BEC",
            "M1은 GPIO 4의 전방 왼쪽 모터이고 CW",
            "같은 물리 방향을 같은 부호로 표현하도록 먼저 축을 정렬",
            "Roll 믹서의 보정항",
            "host SIL",
            "실제 기체에서 확정한 최종 게인이 아니",
            "마지막 1분",
        ):
            self.assertIn(phrase, normalized)
        self.assertGreaterEqual(len(script), 19500)
        self.assertLessEqual(len(script), 22000)
        for segmented_label in ("핵심 발화", "확장 발화", "선택 발화"):
            self.assertNotIn(segmented_label, script)

    def test_session_three_script_covers_safety_evidence_experience_and_qna(self) -> None:
        script = (DECK_DIR / "SESSION3_SCRIPT.md").read_text(encoding="utf-8")
        normalized = re.sub(r"\s+", " ", script)
        self.assertEqual(
            [int(number) for number in re.findall(r"^## (\d+)쪽", script, re.MULTILINE)],
            list(range(55, 85)),
        )
        for phrase in (
            "방향 기준·안전·검증에 약 20분",
            "시연·로그·모바일 체험·계획에 약 20분",
            "질문과 답변에 약 10분",
            "여기까지는 수신·파싱·기록",
            "아직 모터 출력에 영향을 주지 않",
            "공중 프로브 2회",
            "실제 접지 상태에서 얻은 프로브 10회",
            "M3 평균은 1360.0",
            "테더 하중의 영향으로 추정",
            "실제 기체와 연결되는 화면이 아니라",
        ):
            self.assertIn(phrase, normalized)
        self.assertGreaterEqual(len(script), 14500)
        self.assertLessEqual(len(script), 18000)
        for segmented_label in ("핵심 발화", "확장 발화", "선택 발화"):
            self.assertNotIn(segmented_label, script)

    @unittest.skipUnless(shutil.which("ffprobe"), "ffprobe is required")
    def test_hover_demo_source_is_chrome_compatible_h264(self) -> None:
        result = subprocess.run(
            [
                shutil.which("ffprobe") or "ffprobe",
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=codec_name,width,height,pix_fmt,r_frame_rate,avg_frame_rate",
                "-of",
                "json",
                str(DECK_DIR / "assets" / "hover_demo.mp4"),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        stream = json.loads(result.stdout)["streams"][0]
        self.assertEqual(stream["codec_name"], "h264")
        self.assertEqual((stream["width"], stream["height"]), (1280, 720))
        self.assertEqual(stream["pix_fmt"], "yuv420p")
        self.assertEqual(stream["r_frame_rate"], "30/1")
        self.assertEqual(stream["avg_frame_rate"], "30/1")

    def test_corrected_evidence_slides_keep_measurement_and_inference_boundaries(self) -> None:
        parser = _DeckParser()
        source = (DECK_DIR / "index.html").read_text(encoding="utf-8")
        parser.feed(source)

        self.assertEqual(len(parser.sections), 84)
        self.assertNotIn(
            "현재 실행 상태 · 차기 PCB는 주문 진행 중이다. 납땜 작업과 알리 부품 납기는 일정 위험으로 관리하고 있다.",
            source,
        )
        slide30 = parser.sections[29]
        self.assertNotIn("차기 PCB는 주문 진행 중", _slide_text(slide30))
        self.assertNotIn(
            "차기 PCB는 주문 진행 중",
            str(slide30["attrs"].get("data-speaker-notes", "")),
        )

        slide31 = parser.sections[30]
        slide31_boundary = _slide_text(slide31) + " " + str(
            slide31["attrs"].get("data-speaker-notes", "")
        )
        for phrase in (
            "먼저 collective",
            "자세 차동",
            "차동 폭",
            "같은 비율",
        ):
            self.assertIn(phrase, slide31_boundary)

        slide42 = parser.sections[41]
        slide42_boundary = _slide_text(slide42) + " " + str(
            slide42["attrs"].get("data-speaker-notes", "")
        )
        self.assertIn("Roll·Pitch", slide42_boundary)
        self.assertIn("Yaw", slide42_boundary)
        self.assertIn("별도 벤치 재검증", slide42_boundary)
        self.assertNotIn("현재 검증된 여섯 줄", slide42_boundary)

        slide58 = parser.sections[57]
        slide58_text = _slide_text(slide58)
        slide58_notes = str(slide58["attrs"].get("data-speaker-notes", ""))
        for phrase in (
            "점 = 고정 벤치 실측 샘플",
            "실선 = 회귀 추세선",
            "과거 모터 전류 보상 벤치 기준",
            "지자기 융합 ON/OFF 비교가 아니다",
        ):
            self.assertIn(phrase, slide58_text)
        self.assertIn("현행 타원체 보정과 구분", slide58_notes)

        slide60 = parser.sections[59]
        slide60_text = _slide_text(slide60)
        slide60_notes = str(slide60["attrs"].get("data-speaker-notes", ""))
        self.assertEqual(_slide_title(slide60), "제어와 안전 전환")
        for phrase in (
            "센서 측정",
            "자세 추정",
            "모터 제어",
            "안전 전환",
            "흐름이 끊겼을 때",
        ):
            self.assertIn(phrase, slide60_text + " " + slide60_notes)
        self.assertNotIn("Mag_Enabled", slide60_text)
        self.assertNotIn("mag 1", slide60_text)

        slide61 = parser.sections[60]
        slide61_notes = str(slide61["attrs"].get("data-speaker-notes", ""))
        for phrase in ("host shim은 no-op", "timeout·panic·재부팅", "실제 보드"):
            self.assertIn(phrase, slide61_notes)

        slide65 = parser.sections[64]
        self.assertEqual(
            [video.get("src") for video in slide65["videos"]],
            ["assets/landing-ambiguity.mp4"],
        )
        self.assertEqual(slide65["images"], [])
        slide65_video = slide65["videos"][0]
        for required in ("controls", "loop", "muted", "playsinline"):
            self.assertIn(required, slide65_video)
        self.assertEqual(slide65_video.get("preload"), "metadata")
        slide65_text_and_notes = _slide_text(slide65) + " " + str(
            slide65["attrs"].get("data-speaker-notes", "")
        )
        for phrase in ("지면 정지", "등속 하강", "같은 1g", "IMU"):
            self.assertIn(phrase, slide65_text_and_notes)

        slide70 = parser.sections[69]
        self.assertEqual(slide70["videos"], [])
        landing_images = [
            image
            for image in slide70["images"]
            if image.get("src") == "assets/landing-probe-evidence.png"
        ]
        self.assertEqual(len(landing_images), 1)
        slide70_text_and_notes = _slide_text(slide70) + " " + str(
            slide70["attrs"].get("data-speaker-notes", "")
        )
        for phrase in (
            "공중 프로브 2회",
            "접지 프로브 10회",
            "값이 겹쳤다",
            "지면이면 무반응",
            "기록 전용",
            "착지 판정에서 제외",
        ):
            self.assertIn(phrase, slide70_text_and_notes)

        slide72 = parser.sections[71]
        slide72_boundary = _slide_text(slide72) + " " + str(
            slide72["attrs"].get("data-speaker-notes", "")
        )
        self.assertIn("가능한 경쟁", slide72_boundary)
        self.assertIn("관측된 비행사고는 아니다", slide72_boundary)

        slide74 = parser.sections[73]
        slide74_boundary = _slide_text(slide74) + " " + str(
            slide74["attrs"].get("data-speaker-notes", "")
        )
        for phrase in ("지자기 보정 비활성·부호 반전", "Roll 믹서 부호 반전"):
            self.assertIn(phrase, slide74_boundary)
        self.assertNotIn("게이트 제거와 순서 변경", slide74_boundary)

        slide76 = parser.sections[75]
        slide76_text = _slide_text(slide76)
        for phrase in ("주목적", "줄 하중", "별도 테더 로그"):
            self.assertIn(phrase, slide76_text)
        self.assertNotIn("이 순간 실제로 기록된 숫자", slide76_text)
        self.assertNotIn("역할만 한다", slide76_text)

        slide80 = parser.sections[79]
        slide80_text = _slide_text(slide80)
        slide80_notes = str(slide80["attrs"].get("data-speaker-notes", ""))
        slide80_alt = " ".join(str(image.get("alt", "")) for image in slide80["images"])
        slide80_boundary = " ".join((slide80_text, slide80_notes, slide80_alt))
        for phrase in (
            "M3>M1",
            "M3 근처",
            "줄 무게까지",
            "해석",
            "한 구간",
            "추력·프레임·공력 차이",
        ):
            self.assertIn(phrase, slide80_boundary)
        self.assertNotIn("추정", slide80_text)
        self.assertNotIn("확정 아님", slide80_text)

        slide81 = parser.sections[80]
        self.assertEqual(_slide_title(slide81), "FC의 균형잡기 체험")
        slide81_text = _slide_text(slide81)
        self.assertIn("비행제어기가 빠르게 반복하는 균형잡기", slide81_text)
        self.assertNotIn("설치·로그인 없이", slide81_text)

        sources = (DECK_DIR / "SOURCES.md").read_text(encoding="utf-8")
        self.assertNotIn("PCB 주문·납땜·부품 납기 상태", sources)
        self.assertIn("사용자 확인 프로젝트 추정", sources)
        self.assertIn("지자기 융합 ON/OFF 비교가 아님", sources)
        self.assertIn("테더 줄이 M3 근처에 연결", sources)

    def test_professor_feedback_slides_preserve_the_required_evidence_boundaries(self) -> None:
        parser = _DeckParser()
        parser.feed((DECK_DIR / "index.html").read_text(encoding="utf-8"))

        self.assertEqual(len(parser.sections), 84)

        slide14 = parser.sections[13]
        slide14_text = _slide_text(slide14)
        self.assertEqual(_slide_title(slide14), "상용 제품과 자작 학습")
        for phrase in (
            "빠른 운용",
            "현장 적용",
            "학습·검증",
            "오류 재현",
            "디버깅",
            "우열이 아니라 목적의 차이",
        ):
            self.assertIn(phrase, slide14_text)
        self.assertNotIn("듀얼 IMU 구성", slide14_text)
        self.assertTrue(
            any(
                attrs.get("src") == "assets/image18.jpeg"
                for attrs in slide14["images"]
            )
        )

        slide22 = parser.sections[21]
        slide22_text = _slide_text(slide22)
        slide22_notes = str(slide22["attrs"].get("data-speaker-notes", ""))
        self.assertEqual(_slide_title(slide22), "1대와 10대 제작 규모")
        for hidden_caveat in (
            "예비 산정",
            "실측 기록 아님",
            "프로젝트 추정",
            "판매처 가격 증빙",
            "완성 기체 총원가가 아니다",
        ):
            self.assertNotIn(hidden_caveat, slide22_text)
        self.assertTrue(
            any(
                attrs.get("src") == "assets/production-estimate.png"
                for attrs in slide22["images"]
            )
        )
        for phrase in (
            "24~48시간",
            "6~10인시",
            "약 2~3일",
            "240~480시간",
            "60~100인시",
            "약 10~20일",
            "모터 4개 80,000원",
            "ESC 4개 40,000원",
            "221,900~233,100원",
            "2,219,000~2,331,000원",
            "사용자 확인 프로젝트 추정",
            "판매처 가격 증빙",
            "부품 재고",
            "프린터 한 대",
            "조립된 FC PCB",
            "첫 출력 성공",
            "검증된 BOM이 아니다",
            "FC PCB",
            "전원",
            "배선",
            "체결부품",
            "배송·관부가세",
            "인건비",
            "재출력",
            "비행 튜닝",
        ):
            self.assertIn(phrase, slide22_notes)

        slide23 = parser.sections[22]
        slide23_text = _slide_text(slide23)
        slide23_notes = str(slide23["attrs"].get("data-speaker-notes", ""))
        self.assertEqual(_slide_title(slide23), "모듈형 프레임 설계")
        for phrase in (
            "손상된 부품만",
            "다시 출력",
            "교체",
            "테스트 중 수리",
        ):
            self.assertIn(phrase, slide23_text)
        self.assertEqual(len(slide23["images"]), 1)
        self.assertNotIn("트러스", slide23_text + slide23_notes)
        for unsupported_detail in ("세 번", "하루 안에"):
            self.assertNotIn(unsupported_detail, slide23_text + slide23_notes)

        slide25 = parser.sections[24]
        slide25_text = _slide_text(slide25)
        self.assertEqual(_slide_title(slide25), "3D 프린팅 래피드 프로토타이핑")
        for phrase in (
            "CAD",
            "출력",
            "조립",
            "수정",
            "나사 간격",
            "적층 방향",
            "배선 통로",
            "Maker Space 박근원 선생님의 장비·제작 지원에 감사드립니다.",
        ):
            self.assertIn(phrase, slide25_text)
        self.assertNotIn("하루 안에", slide25_text)
        self.assertTrue(
            any(
                attrs.get("src") == "assets/image14.jpeg"
                for attrs in slide25["images"]
            )
        )

        slide43 = parser.sections[42]
        slide43_text = _slide_text(slide43)
        slide43_notes = str(slide43["attrs"].get("data-speaker-notes", ""))
        self.assertEqual(_slide_title(slide43), "문제 추적과 수정")
        for phrase in (
            "증상",
            "기록·비교",
            "원인 추적",
            "오류 재현",
            "수정 검증",
            "Roll 믹서",
            "실패를 재현",
            "디버깅",
        ):
            self.assertIn(phrase, slide43_text)
        self.assertIn("host SIL", slide43_notes)
        self.assertIn("실기·비행 성능의 증거가 아니다", slide43_notes)

        slide83 = parser.sections[82]
        slide83_text = _slide_text(slide83)
        slide83_notes = str(slide83["attrs"].get("data-speaker-notes", ""))
        self.assertEqual(_slide_title(slide83), "단기·중기·장기 목표")
        for phrase in (
            "단기",
            "반복 가능한 단일 기체 자세 제어",
            "3901-L0X",
            "착지 판단 검증",
            "중기",
            "반복 제작·교정",
            "sim-to-real 경로 계획",
            "다기체 운용 기반",
            "장기",
            "군집 제어",
            "충돌 회피",
            "집단 안전",
            "통과 기준을 충족한 뒤",
        ):
            self.assertIn(phrase, slide83_text)
        self.assertIn("아직 구현·검증하지 않은 계획", slide83_notes)

        slide21_text = _slide_text(parser.sections[20])
        self.assertIn("지상 제어 프로그램을 직접 설계·구현했다", slide21_text)
        self.assertNotIn("지상국을 직접 설계·구현했다", slide21_text)

        slide24_text = _slide_text(parser.sections[23])
        self.assertIn("배터리와 배선, ESC", slide24_text)
        self.assertNotIn("육각 패턴", slide24_text)
        self.assertIn("방위 센서 측정값에 간섭을 일으키기 때문이다", slide24_text)
        self.assertNotIn("방위 센서를 오염시키기 때문이다", slide24_text)
        self.assertTrue(
            any(
                "data-component-bay-label" in attrs
                for _, attrs in parser.sections[23]["elements"]
            )
        )

        slide60 = parser.sections[59]
        self.assertIn("데이터 상태 확인", _slide_text(slide60))
        self.assertTrue(
            any(
                "data-safety-branch" in attrs
                for _, attrs in slide60["elements"]
            )
        )

    def test_requested_slide_revisions_match_the_audience_contract(self) -> None:
        parser = _DeckParser()
        source = (DECK_DIR / "index.html").read_text(encoding="utf-8")
        parser.feed(source)

        slide8 = parser.sections[7]
        self.assertEqual(len(slide8["images"]), 4)
        self.assertEqual(
            sum(
                1
                for tag, attrs in slide8["elements"]
                if tag == "figcaption" and "data-evidence-label" in attrs
            ),
            4,
        )
        cad = next(image for image in slide8["images"] if "data-cad-evidence" in image)
        self.assertEqual(cad.get("src"), "assets/image13.png")
        self.assertIn("object-fit:contain", re.sub(r"\s+", "", str(cad.get("style", ""))))

        slide28 = parser.sections[27]
        slide28_text = _slide_text(slide28)
        for phrase in (
            "12V 입력",
            "ESC의 BEC",
            "5V를 ESP32에 공급",
            "ESP32의 3.3V 출력",
        ):
            self.assertIn(phrase, slide28_text)
        self.assertNotIn("보호용 MOSFET", slide28_text)
        self.assertNotIn("12V → 5V / 3.3V", source)

        slide29 = parser.sections[28]
        self.assertTrue(
            any(image.get("src") == "assets/pcb-render-current.png" for image in slide29["images"])
        )
        self.assertIn("실물", _slide_text(slide29))
        self.assertNotIn("비행제어 보드 v1.4.3", _slide_text(slide29))
        self.assertTrue((DECK_DIR / "assets" / "pcb-render-current.png").is_file())

        slide30_text = _slide_text(parser.sections[29])
        self.assertIn("외부 센서 보드와 점퍼선", slide30_text)
        self.assertIn("신호 경로 단축", slide30_text)
        self.assertNotIn("센서를 교체하면 배선을 다시 만들어야 했다", slide30_text)

        slide32 = parser.sections[31]
        self.assertEqual(slide32["images"], [])
        for phrase in (
            "프로펠러 제거",
            "극성·핀·모터 순서",
            "전류 제한 전원",
            "한 번에 한 모터",
            "USB 명령",
        ):
            self.assertIn(phrase, _slide_text(slide32))

        slide66_text = _slide_text(parser.sections[65])
        self.assertIn("바닥까지 거리 h", slide66_text)

        self.assertNotIn("“스톱” 후", _slide_text(parser.sections[74]))

        self.assertEqual(_slide_title(parser.sections[79]), "텔레메트리 로그 해석")
        self.assertEqual(_slide_title(parser.sections[80]), "FC의 균형잡기 체험")

        closing = parser.sections[83]
        closing_visible = " ".join(filter(None, (_slide_title(closing), _slide_text(closing))))
        self.assertEqual(_slide_title(closing), "감사합니다")
        self.assertIn("Q & A", closing_visible)
        for removed in (
            "기체 관람",
            "궁금한 점을 질문하고",
            "프레임과 조립 구조",
            "비행제어 보드와 배선",
            "센서와 안전 설계",
            "프로펠러를 분리하고",
            "테더 시험 기록",
        ):
            self.assertNotIn(removed, closing_visible)
        self.assertEqual(closing["videos"], [])

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
        self.assertEqual(len(all_videos), 13)
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
    def _dispatch_f_key(cls) -> None:
        for event_type in ("keyDown", "keyUp"):
            params = {
                "type": event_type,
                "key": "f",
                "code": "KeyF",
                "windowsVirtualKeyCode": 70,
                "nativeVirtualKeyCode": 70,
            }
            if event_type == "keyDown":
                params["text"] = "f"
            cls._call("Input.dispatchKeyEvent", params)

    @classmethod
    def _fullscreen_state(cls) -> dict:
        return cls._evaluate(
            """
            (() => {
              const stage = document.querySelector('deck-stage');
              const button = stage.shadowRoot.querySelector('.fullscreen');
              return {
                fullscreen: Boolean(document.fullscreenElement),
                presenting: Boolean(stage._presenting),
                label: button && button.getAttribute('aria-label'),
                overlayVisible: stage._overlay.hasAttribute('data-visible'),
              };
            })()
            """
        )

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
              const slide = document.querySelector('deck-stage')._slides[80];
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
              const magChart = stage._slides[57].querySelector(
                'img[src$="chart_mag.png"]'
              );
              const magSlide = stage._slides[57];
              const magScale = magSlide.getBoundingClientRect().width / 1280;
              return {
                collageSources,
                timingVideoLoaded: Boolean(
                  timingVideo && timingVideo.readyState >= HTMLMediaElement.HAVE_METADATA
                ),
                magChartLoaded: Boolean(magChart?.complete && magChart.naturalWidth),
                magChartLogicalWidth:
                  magChart ? magChart.getBoundingClientRect().width / magScale : 0,
              };
            })()
            """
        )

        self.assertEqual(
            sorted(Path(source).name for source in result["collageSources"]),
            sorted(
                [
                    "image13.png",
                    "image12.png",
                    "chart_attitude.png",
                    "mobile-lab-student.png",
                ]
            ),
            result,
        )
        self.assertTrue(result["timingVideoLoaded"], result)
        self.assertTrue(result["magChartLoaded"], result)
        self.assertGreaterEqual(result["magChartLogicalWidth"], 1000, result)

    def test_torque_comparison_fits_and_safety_transition_replaces_mag_command(self) -> None:
        self._open_deck()
        result = self._evaluate(
            r"""
            (async () => {
              const stage = document.querySelector('deck-stage');
              const inspect = async number => {
                stage.goTo(number - 1);
                stage._fit();
                await new Promise(resolve => requestAnimationFrame(
                  () => requestAnimationFrame(resolve)
                ));
                return stage._slides[number - 1];
              };

              const torqueSlide = await inspect(10);
              const torqueSlideRect = torqueSlide.getBoundingClientRect();
              const torqueFigure = torqueSlide.querySelector(
                'img[data-torque-comparison-art]'
              )?.parentElement;
              const torqueFigureRect = torqueFigure?.getBoundingClientRect();
              const torqueImage = torqueSlide.querySelector(
                'img[data-torque-comparison-art]'
              );
              const torqueRect = torqueImage?.getBoundingClientRect();

              const safetySlide = await inspect(60);
              return {
                torqueLoaded: Boolean(
                  torqueImage?.complete && torqueImage.naturalWidth
                ),
                torqueInside: Boolean(
                  torqueRect
                  && torqueRect.left >= torqueSlideRect.left - 1
                  && torqueRect.top >= torqueSlideRect.top - 1
                  && torqueRect.right <= torqueSlideRect.right + 1
                  && torqueRect.bottom <= torqueSlideRect.bottom + 1
                ),
                torqueBottomOverflow: torqueRect
                  ? torqueRect.bottom - torqueSlideRect.bottom : null,
                torqueSlideRect: torqueSlideRect
                  ? [torqueSlideRect.left, torqueSlideRect.top,
                     torqueSlideRect.width, torqueSlideRect.height] : null,
                torqueFigureRect: torqueFigureRect
                  ? [torqueFigureRect.left, torqueFigureRect.top,
                     torqueFigureRect.width, torqueFigureRect.height] : null,
                torqueImageRect: torqueRect
                  ? [torqueRect.left, torqueRect.top,
                     torqueRect.width, torqueRect.height] : null,
                torqueImageStyle: torqueImage
                  ? {
                      width: getComputedStyle(torqueImage).width,
                      height: getComputedStyle(torqueImage).height,
                      maxWidth: getComputedStyle(torqueImage).maxWidth,
                      maxHeight: getComputedStyle(torqueImage).maxHeight,
                    } : null,
                magCommandCount: safetySlide.querySelectorAll(
                  '[data-command="mag-1"]'
                ).length,
                safetyText: safetySlide.textContent,
              };
            })()
            """,
            await_promise=True,
        )

        self.assertTrue(result["torqueLoaded"], result)
        self.assertTrue(result["torqueInside"], result)
        self.assertEqual(result["magCommandCount"], 0, result)
        self.assertIn("안전 전환", result["safetyText"])

    def test_slide_28_schematic_is_contained_without_cropping(self) -> None:
        self._open_deck()
        result = self._evaluate(
            r"""
            (async () => {
              const stage = document.querySelector('deck-stage');
              const index = stage._slides.findIndex(
                slide => slide.dataset.label === '보드 회로'
              );
              stage.goTo(index);
              stage._fit();
              await new Promise(resolve => requestAnimationFrame(
                () => requestAnimationFrame(resolve)
              ));
              const slide = stage._slides[index];
              const image = slide.querySelector('img[src$="pcb_schematic.png"]');
              const frame = image?.parentElement;
              const imageRect = image?.getBoundingClientRect();
              const frameRect = frame?.getBoundingClientRect();
              const slideRect = slide?.getBoundingClientRect();
              return {
                slide: index + 1,
                loaded: Boolean(image?.complete && image.naturalWidth),
                objectFit: image && getComputedStyle(image).objectFit,
                insideFrame: Boolean(
                  imageRect && frameRect
                  && imageRect.left >= frameRect.left - 1
                  && imageRect.top >= frameRect.top - 1
                  && imageRect.right <= frameRect.right + 1
                  && imageRect.bottom <= frameRect.bottom + 1
                ),
                insideSlide: Boolean(
                  imageRect && slideRect
                  && imageRect.left >= slideRect.left - 1
                  && imageRect.top >= slideRect.top - 1
                  && imageRect.right <= slideRect.right + 1
                  && imageRect.bottom <= slideRect.bottom + 1
                ),
                imageRect: imageRect && [
                  imageRect.left, imageRect.top, imageRect.right, imageRect.bottom
                ],
                frameRect: frameRect && [
                  frameRect.left, frameRect.top, frameRect.right, frameRect.bottom
                ],
              };
            })()
            """,
            await_promise=True,
        )

        self.assertEqual(result["slide"], 28, result)
        self.assertTrue(result["loaded"], result)
        self.assertEqual(result["objectFit"], "contain", result)
        self.assertTrue(result["insideFrame"], result)
        self.assertTrue(result["insideSlide"], result)

    def test_tof_label_clears_vector_and_legacy_board_silkscreen_is_cropped(self) -> None:
        self._open_deck()
        result = self._evaluate(
            r"""
            (async () => {
              const stage = document.querySelector('deck-stage');
              const inspect = async number => {
                stage.goTo(number - 1);
                stage._fit();
                await new Promise(resolve => requestAnimationFrame(
                  () => requestAnimationFrame(resolve)
                ));
                return stage._slides[number - 1];
              };

              const tofSlide = await inspect(66);
              const vector = tofSlide.querySelector('[data-tof-distance-vector]');
              const label = tofSlide.querySelector('[data-tof-distance-label]');
              const vectorRect = vector.getBoundingClientRect();
              const labelRect = label.getBoundingClientRect();

              const pcbSlide = await inspect(29);
              const layout = pcbSlide.querySelector('img[src$="pcb_layout.png"]');
              const photo = pcbSlide.querySelector('img[src$="image18.jpeg"]');
              const layoutRect = layout.getBoundingClientRect();
              const layoutFrameRect = layout.parentElement.getBoundingClientRect();
              const photoRect = photo.getBoundingClientRect();
              const frameRect = photo.parentElement.getBoundingClientRect();

              const cycleSlide = await inspect(13);
              const benchMask = cycleSlide.querySelector('[data-legacy-silk-mask="bench"]');
              const purposeSlide = await inspect(14);
              const purposeBoard = purposeSlide.querySelector('[data-legacy-silk-crop="board-purpose"]');
              const layerSlide = await inspect(15);
              const layerBoard = layerSlide.querySelector('[data-legacy-silk-crop="board-layer"]');
              return {
                tofGap: labelRect.left - vectorRect.right,
                tofScale: tofSlide.getBoundingClientRect().width / 1280,
                layoutCropFraction:
                  (layoutRect.right - layoutFrameRect.right) / layoutFrameRect.width,
                photoCropFraction: (photoRect.right - frameRect.right) / frameRect.width,
                benchMaskVisible: Boolean(
                  benchMask && benchMask.getBoundingClientRect().width > 40
                ),
                purposeScale: new DOMMatrix(
                  getComputedStyle(purposeBoard).transform
                ).a,
                layerScale: new DOMMatrix(
                  getComputedStyle(layerBoard).transform
                ).a,
              };
            })()
            """,
            await_promise=True,
        )
        self.assertGreaterEqual(result["tofGap"] / result["tofScale"], 8.0, result)
        self.assertGreaterEqual(result["layoutCropFraction"], 0.14, result)
        self.assertGreaterEqual(result["photoCropFraction"], 0.25, result)
        self.assertTrue(result["benchMaskVisible"], result)
        self.assertGreaterEqual(result["purposeScale"], 1.44, result)
        self.assertGreaterEqual(result["layerScale"], 1.17, result)

    def test_professor_feedback_slides_render_visual_first_without_overflow(self) -> None:
        self._open_deck()
        result = self._evaluate(
            r"""
            (async () => {
              const stage = document.querySelector('deck-stage');
              const targets = [14, 22, 25, 43, 83];
              const slides = {};
              for (const number of targets) {
                stage.goTo(number - 1);
                stage._fit();
                await new Promise(resolve => requestAnimationFrame(
                  () => requestAnimationFrame(resolve)
                ));
                const slide = stage._slides[number - 1];
                const slideRect = slide.getBoundingClientRect();
                const scale = slideRect.width / 1280;
                const marker = slide.querySelector('[data-professor-feedback]');
                const markedElements = [...slide.querySelectorAll(
                  '[data-purpose-column], [data-production-estimate], '
                  + '[data-rapid-step], [data-debug-step], [data-goal-stage]'
                )];
                const overflow = markedElements.flatMap(element => {
                  const rect = element.getBoundingClientRect();
                  return rect.left < slideRect.left - 1 || rect.top < slideRect.top - 1
                    || rect.right > slideRect.right + 1
                    || rect.bottom > slideRect.bottom + 1
                    ? [{text: element.textContent.trim().slice(0, 60)}]
                    : [];
                });
                const image = slide.querySelector(
                  'img[src$="image18.jpeg"], img[src$="production-estimate.png"], '
                  + 'img[src$="image14.jpeg"]'
                );
                const imageRect = image?.getBoundingClientRect();
                slides[number] = {
                  marker: marker?.dataset.professorFeedback || null,
                  text: slide.textContent.replace(/\s+/g, ' ').trim(),
                  overflow,
                  purposeColumns: slide.querySelectorAll('[data-purpose-column]').length,
                  rapidSteps: slide.querySelectorAll('[data-rapid-step]').length,
                  debugSteps: slide.querySelectorAll('[data-debug-step]').length,
                  goalStages: slide.querySelectorAll('[data-goal-stage]').length,
                  imageSource: image?.getAttribute('src') || null,
                  imageLoaded: Boolean(image?.complete && image.naturalWidth),
                  imageLogicalWidth: imageRect ? imageRect.width / scale : 0,
                  imageLogicalHeight: imageRect ? imageRect.height / scale : 0,
                  productionSize: image?.src.endsWith('production-estimate.png')
                    ? [image.naturalWidth, image.naturalHeight] : null,
                };
              }
              return slides;
            })()
            """,
            await_promise=True,
        )

        for number in (14, 22, 25, 43, 83):
            with self.subTest(slide=number):
                self.assertEqual(result[str(number)]["marker"], str(number), result)
                self.assertEqual(result[str(number)]["overflow"], [], result)

        self.assertEqual(result["14"]["purposeColumns"], 2, result)
        self.assertTrue(result["14"]["imageLoaded"], result)
        self.assertGreaterEqual(result["14"]["imageLogicalWidth"], 360, result)

        self.assertEqual(
            result["22"]["imageSource"], "assets/production-estimate.png", result
        )
        self.assertTrue(result["22"]["imageLoaded"], result)
        self.assertEqual(result["22"]["productionSize"], [1280, 720], result)
        self.assertGreaterEqual(result["22"]["imageLogicalWidth"], 1040, result)

        self.assertEqual(result["25"]["rapidSteps"], 4, result)
        self.assertTrue(result["25"]["imageLoaded"], result)
        self.assertGreaterEqual(result["25"]["imageLogicalWidth"], 600, result)
        self.assertGreaterEqual(result["25"]["imageLogicalHeight"], 360, result)

        self.assertEqual(result["43"]["debugSteps"], 5, result)
        self.assertEqual(result["83"]["goalStages"], 3, result)

    def test_concept_slides_use_relative_physical_language(self) -> None:
        self._open_deck()
        offenders = self._evaluate(
            r"""
            (() => {
              const stage = document.querySelector('deck-stage');
              const conceptSlides = [11, 12, 13, 16, 19, 21, 33, 37, 38, 39,
                47, 50, 51, 52, 53, 56, 57, 61, 62, 64, 68, 69, 70, 71,
                72, 77, 80, 81, 82, 83];
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
                element => element.textContent.trim() ===
                  '교육 아이템의 가치와 드론 원리부터 프레임 제작까지'
              );
              const chartLabel = [...slides[58].querySelectorAll('svg text')].find(
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

    def test_slide_61_renders_watchdog_timeout_as_directional_timeline(self) -> None:
        self._open_deck()
        timeline = self._evaluate(
            """
            (() => {
              const slide = document.querySelector('deck-stage')._slides[60];
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

    def test_slide_59_renders_yaw_drift_as_two_time_series(self) -> None:
        self._call(
            "Page.navigate",
            {"url": f"http://127.0.0.1:{self.http_port}/#59"},
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
                comparison["hash"] == "#59"
                and comparison["readyState"] == "complete"
                and comparison["sections"] == 84
                and comparison["label"]
            ):
                break
            time.sleep(0.05)

        self.assertEqual(comparison["hash"], "#59", comparison)
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

        self._evaluate("document.querySelector('deck-stage').goTo(64)")
        self._wait_for_playback("landing-ambiguity.mp4")
        self._evaluate("document.querySelector('deck-stage').goTo(69)")
        landing_previous = self._video_state("landing-ambiguity.mp4")

        self.assertIsNotNone(landing_previous)
        self.assertTrue(landing_previous["paused"])
        self.assertLess(landing_previous["currentTime"], 0.05)

    def test_fullscreen_control_and_f_key_toggle_presentation_mode(self) -> None:
        """Catch a missing or broken presenter fullscreen entry and exit path."""
        self._open_deck()
        initial = self._evaluate(
            """
            (() => {
              const stage = document.querySelector('deck-stage');
              const button = stage.shadowRoot.querySelector('.fullscreen');
              return {
                button: Boolean(button),
                label: button && button.getAttribute('aria-label'),
                fullscreen: Boolean(document.fullscreenElement),
                presenting: Boolean(stage._presenting),
              };
            })()
            """
        )
        self.assertEqual(
            initial,
            {
                "button": True,
                "label": "전체화면 보기",
                "fullscreen": False,
                "presenting": False,
            },
        )

        self._dispatch_f_key()
        deadline = time.monotonic() + 3.0
        entered = None
        while time.monotonic() < deadline:
            entered = self._fullscreen_state()
            if entered["fullscreen"] and entered["presenting"]:
                break
            time.sleep(0.05)
        self.assertEqual(
            entered,
            {
                "fullscreen": True,
                "presenting": True,
                "label": "전체화면 종료",
                "overlayVisible": False,
            },
        )

        self._dispatch_f_key()
        deadline = time.monotonic() + 3.0
        exited = None
        while time.monotonic() < deadline:
            exited = self._fullscreen_state()
            if not exited["fullscreen"] and not exited["presenting"]:
                break
            time.sleep(0.05)
        self.assertEqual(
            exited,
            {
                "fullscreen": False,
                "presenting": False,
                "label": "전체화면 보기",
                "overlayVisible": False,
            },
        )

    def test_hover_demo_instance_decodes_autoplays_and_resets(self) -> None:
        self._open_deck()
        slide_indices = self._evaluate(
            r"""
            (() => {
              const stage = document.querySelector('deck-stage');
              return stage._slides.flatMap((slide, index) =>
                slide.querySelector('video[src$="hover_demo.mp4"]') ? [index] : []
              );
            })()
            """
        )
        self.assertEqual(slide_indices, [75], slide_indices)

        for index in slide_indices:
            with self.subTest(slide=index + 1):
                self._evaluate(
                    f"document.querySelector('deck-stage').goTo({index})"
                )
                deadline = time.monotonic() + 5.0
                active = None
                while time.monotonic() < deadline:
                    active = self._evaluate(
                        r"""
                        (() => {
                          const stage = document.querySelector('deck-stage');
                          const index = %d;
                          const video = stage._slides[index].querySelector(
                            'video[src$="hover_demo.mp4"]'
                          );
                          return video && {
                            paused: video.paused,
                            currentTime: video.currentTime,
                            readyState: video.readyState,
                            videoWidth: video.videoWidth,
                            videoHeight: video.videoHeight,
                            error: video.error && {
                              code: video.error.code,
                              message: video.error.message,
                            },
                          };
                        })()
                        """
                        % index
                    )
                    if (
                        active
                        and not active["paused"]
                        and active["currentTime"] > 0.2
                        and active["videoWidth"] == 1280
                        and active["videoHeight"] == 720
                    ):
                        break
                    time.sleep(0.05)

                self.assertIsNotNone(active)
                self.assertEqual(active["error"], None, active)
                self.assertGreaterEqual(active["readyState"], 2, active)
                self.assertEqual(active["videoWidth"], 1280, active)
                self.assertEqual(active["videoHeight"], 720, active)
                self.assertFalse(active["paused"], active)
                self.assertGreater(active["currentTime"], 0.2, active)

                destination = index - 1 if index > 0 else index + 1
                self._evaluate(
                    f"document.querySelector('deck-stage').goTo({destination})"
                )
                deadline = time.monotonic() + 2.0
                reset = None
                while time.monotonic() < deadline:
                    reset = self._evaluate(
                        r"""
                        (() => {
                          const stage = document.querySelector('deck-stage');
                          const video = stage._slides[%d].querySelector(
                            'video[src$="hover_demo.mp4"]'
                          );
                          return video && {
                            paused: video.paused,
                            currentTime: video.currentTime,
                          };
                        })()
                        """
                        % index
                    )
                    if reset and reset["paused"] and reset["currentTime"] < 0.05:
                        break
                    time.sleep(0.05)
                self.assertTrue(reset["paused"], reset)
                self.assertLess(reset["currentTime"], 0.05, reset)

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
