#!/usr/bin/env python3
"""Contract tests for the generated AI startup camp presentation."""

from __future__ import annotations

import os
import posixpath
import re
import shutil
import subprocess
import tempfile
import unittest
import zipfile
from pathlib import Path
from xml.etree import ElementTree

from pptx import Presentation


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PPTX_PATH = (
    REPO_ROOT
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "ZETIN_Drone_AI_Startup_Camp.pptx"
)
PPTX_PATH = Path(
    os.environ.get("ZETIN_PRESENTATION_PPTX", str(DEFAULT_PPTX_PATH))
)
DEFAULT_PDF_PATH = DEFAULT_PPTX_PATH.with_suffix(".pdf")
PDF_PATH = Path(
    os.environ.get("ZETIN_PRESENTATION_PDF", str(DEFAULT_PDF_PATH))
)
EXPECTED_SLIDES = 84
EXPECTED_VIDEO_SLIDES = 27
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


@unittest.skipUnless(shutil.which("ffprobe"), "ffprobe is required")
class PresentationPptxExportTests(unittest.TestCase):
    def test_pptx_contains_complete_deck_notes_and_h264_videos(self) -> None:
        self.assertTrue(PPTX_PATH.is_file(), f"missing generated PPTX: {PPTX_PATH}")

        presentation = Presentation(str(PPTX_PATH))
        self.assertEqual(len(presentation.slides), EXPECTED_SLIDES)

        with zipfile.ZipFile(PPTX_PATH) as archive:
            self.assertIsNone(archive.testzip())
            names = archive.namelist()
            self.assertEqual(
                sum(bool(SLIDE_NAME.fullmatch(name)) for name in names),
                EXPECTED_SLIDES,
            )
            self.assertEqual(
                sum(bool(NOTE_NAME.fullmatch(name)) for name in names),
                EXPECTED_SLIDES,
            )

            root = ElementTree.fromstring(archive.read("ppt/presentation.xml"))
            slide_size = root.find("p:sldSz", PRESENTATION_NS)
            self.assertIsNotNone(slide_size)
            self.assertEqual(slide_size.get("cx"), "12192000")
            self.assertEqual(slide_size.get("cy"), "6858000")

            video_names = sorted(
                name
                for name in names
                if name.startswith("ppt/media/") and name.lower().endswith(".mp4")
            )
            self.assertEqual(len(video_names), EXPECTED_VIDEO_SLIDES)

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
                for relationship in relationship_root.findall(
                    "r:Relationship", RELATIONSHIP_NS
                ):
                    target = posixpath.normpath(
                        posixpath.join(
                            slide_directory, relationship.get("Target", "")
                        )
                    )
                    item = (relationship_name, target)
                    if relationship.get("Type") == VIDEO_REL:
                        video_relationships.append(item)
                    elif relationship.get("Type") == MEDIA_REL:
                        media_relationships.append(item)

            self.assertEqual(len(video_relationships), EXPECTED_VIDEO_SLIDES)
            self.assertEqual(len(media_relationships), EXPECTED_VIDEO_SLIDES)
            video_targets = {target for _, target in video_relationships}
            media_targets = {target for _, target in media_relationships}
            self.assertEqual(len(video_targets), EXPECTED_VIDEO_SLIDES)
            self.assertEqual(video_targets, media_targets)
            self.assertEqual(video_targets, set(video_names))

            with tempfile.TemporaryDirectory(prefix="zetin-pptx-media-") as temp_dir:
                codecs = []
                for index, video_name in enumerate(video_names, start=1):
                    video_path = Path(temp_dir) / f"video-{index:02d}.mp4"
                    video_path.write_bytes(archive.read(video_name))
                    result = subprocess.run(
                        [
                            "ffprobe",
                            "-v",
                            "error",
                            "-select_streams",
                            "v:0",
                            "-show_entries",
                            "stream=codec_name",
                            "-of",
                            "default=noprint_wrappers=1:nokey=1",
                            str(video_path),
                        ],
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                    codecs.append(result.stdout.strip())
                self.assertEqual(codecs, ["h264"] * EXPECTED_VIDEO_SLIDES)


@unittest.skipUnless(shutil.which("pdfinfo"), "pdfinfo is required")
class PresentationPdfExportTests(unittest.TestCase):
    def test_pdf_contains_complete_wide_deck(self) -> None:
        self.assertTrue(PDF_PATH.is_file(), f"missing generated PDF: {PDF_PATH}")

        result = subprocess.run(
            ["pdfinfo", str(PDF_PATH)],
            check=True,
            capture_output=True,
            text=True,
        )
        metadata = {
            key.strip(): value.strip()
            for line in result.stdout.splitlines()
            if ":" in line
            for key, value in [line.split(":", 1)]
        }

        self.assertEqual(metadata.get("Pages"), str(EXPECTED_SLIDES))
        page_size = re.match(
            r"^([0-9.]+) x ([0-9.]+) pts",
            metadata.get("Page size", ""),
        )
        self.assertIsNotNone(page_size)
        self.assertAlmostEqual(float(page_size.group(1)), 960.0, delta=0.02)
        self.assertAlmostEqual(float(page_size.group(2)), 540.0, delta=0.02)


if __name__ == "__main__":
    unittest.main()
