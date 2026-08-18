#!/usr/bin/env python3
"""Contract tests for the technical 10-minute drone summary deck."""

from __future__ import annotations

import re
import posixpath
import subprocess
import tempfile
import unittest
import zipfile
from fractions import Fraction
from html.parser import HTMLParser
from pathlib import Path
from xml.etree import ElementTree

from pptx import Presentation


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


if __name__ == "__main__":
    unittest.main()
