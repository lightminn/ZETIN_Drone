#!/usr/bin/env python3
"""Build the browser-only AI startup camp presentation site."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DECK_ROOT = REPO_ROOT / "docs" / "presentations" / "ai-startup-camp-drone"
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
    for filename in RUNTIME_FILES:
        shutil.copy2(DECK_ROOT / filename, output / filename)

    shutil.copytree(
        DECK_ROOT / "assets",
        output / "assets",
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns("*.zip"),
    )
    shutil.copytree(
        DECK_ROOT / "vendor",
        output / "vendor",
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
