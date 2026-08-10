"""Small shared validation helpers for Oracle web release tools."""

from __future__ import annotations

import hashlib
from pathlib import Path
import re


_SITE_NAME_RE = re.compile(r"[a-z][a-z0-9-]{0,62}\Z")
_RELEASE_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")


def validate_site_name(value: str) -> str:
    """Return a site name only when it is safe for release paths."""
    if not isinstance(value, str) or not _SITE_NAME_RE.fullmatch(value):
        raise ValueError("site must match [a-z][a-z0-9-]{0,62}")
    return value


def validate_release_id(value: str) -> str:
    """Return a release ID only when it is safe for release paths."""
    if not isinstance(value, str) or not _RELEASE_ID_RE.fullmatch(value):
        raise ValueError("release ID must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}")
    return value


def sha256_file(path: Path) -> str:
    """Compute a SHA-256 digest without reading a whole file into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
