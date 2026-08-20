#!/usr/bin/env python3
"""Build both browser-only AI startup camp presentation decks."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DECKS = (
    ("ai-startup-camp-drone", ""),
    ("ai-startup-camp-drone-10min", "10min"),
)
RUNTIME_FILES = ("index.html", "support.js", "deck-stage.js")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Directory to create for the static site",
    )
    return parser.parse_args()


def build_site(output: Path) -> None:
    output = output.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"output directory is not empty: {output}")

    output.mkdir(parents=True, exist_ok=True)
    for deck_name, subdir in DECKS:
        deck_root = REPO_ROOT / "docs" / "presentations" / deck_name
        deck_output = output / subdir
        deck_output.mkdir(parents=True, exist_ok=True)

        for filename in RUNTIME_FILES:
            shutil.copy2(deck_root / filename, deck_output / filename)

        shutil.copytree(
            deck_root / "assets",
            deck_output / "assets",
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns("*.zip"),
        )
        shutil.copytree(
            deck_root / "vendor",
            deck_output / "vendor",
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns(
                "*.cjs", "*.md", "*.pdf", "*.pptx", "*.py", "*.sh", "*.zip"
            ),
        )

    (output / ".nojekyll").touch()

    file_count = sum(path.is_file() for path in output.rglob("*"))
    print(f"Built {file_count} files in {output}")


def main() -> None:
    args = parse_args()
    build_site(args.output)


if __name__ == "__main__":
    main()
