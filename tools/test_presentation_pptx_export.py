#!/usr/bin/env python3
"""Contract tests for the generated AI startup camp presentation."""

from __future__ import annotations

import os
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
EXPECTED_SLIDES = 77
EXPECTED_VIDEO_SLIDES = 11
SLIDE_NAME = re.compile(r"ppt/slides/slide\d+\.xml$")
NOTE_NAME = re.compile(r"ppt/notesSlides/notesSlide\d+\.xml$")
PRESENTATION_NS = {
    "p": "http://schemas.openxmlformats.org/presentationml/2006/main"
}


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
