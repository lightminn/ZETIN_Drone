"""Strict schema-1 reader for Oracle web release manifests."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import json
from pathlib import Path, PurePosixPath
import re
from typing import Any

from .common import validate_site_name


class ManifestError(ValueError):
    """Raised when a site manifest is unsafe or does not match schema 1."""


@dataclass(frozen=True)
class Backend:
    port: int
    health_path: str


@dataclass(frozen=True)
class ManifestFile:
    source: str
    destination: str


@dataclass(frozen=True)
class SiteManifest:
    schema_version: int
    site: str
    server_name: str
    public_ipv4: str
    https_health_paths: tuple[str, ...]
    backend: Backend | None
    files: tuple[ManifestFile, ...]


_SERVER_NAME_RE = re.compile(
    r"(?=.{1,253}\Z)(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?)(?:\.(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?))*\Z"
)
_GLOB_CHARS = frozenset("*?[]{}")


def _require_exact_keys(value: dict[str, Any], required: set[str], optional: set[str], label: str) -> None:
    keys = set(value)
    missing = required - keys
    unknown = keys - required - optional
    if missing or unknown:
        parts = []
        if missing:
            parts.append(f"missing {', '.join(sorted(missing))}")
        if unknown:
            parts.append(f"unknown {', '.join(sorted(unknown))}")
        raise ManifestError(f"{label} has " + "; ".join(parts))


def _require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{label} must be a non-empty string")
    return value


def _validate_relative_path(value: Any, label: str) -> str:
    path = _require_string(value, label)
    if "\\" in path or any(character in path for character in _GLOB_CHARS):
        raise ManifestError(f"{label} must not contain backslashes or glob syntax")
    parsed = PurePosixPath(path)
    canonical = parsed.as_posix()
    if (
        parsed.is_absolute()
        or canonical == "."
        or path != canonical
        or any(part in ("", ".", "..") for part in parsed.parts)
    ):
        raise ManifestError(f"{label} must be a canonical non-traversing relative POSIX path")
    return canonical


def _validate_url_path(value: Any, label: str) -> str:
    path = _require_string(value, label)
    if not path.startswith("/") or "\\" in path or "?" in path or "#" in path:
        raise ManifestError(f"{label} must be an absolute URL path")
    parts = PurePosixPath(path).parts
    if any(part in (".", "..") for part in parts):
        raise ManifestError(f"{label} must not traverse")
    return path


def _parse_backend(value: Any) -> Backend:
    if not isinstance(value, dict):
        raise ManifestError("backend must be an object")
    _require_exact_keys(value, {"port", "health_path"}, set(), "backend")
    port = value["port"]
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ManifestError("backend.port must be an integer from 1 through 65535")
    return Backend(port=port, health_path=_validate_url_path(value["health_path"], "backend.health_path"))


def load_site_manifest(path: Path) -> SiteManifest:
    """Read and fully validate a schema-1 site manifest."""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestError(f"cannot read manifest {path}: {error}") from error
    if not isinstance(raw, dict):
        raise ManifestError("manifest must be a JSON object")
    _require_exact_keys(
        raw,
        {"schema_version", "site", "server_name", "public_ipv4", "https_health_paths", "files"},
        {"backend"},
        "manifest",
    )
    if raw["schema_version"] != 1 or isinstance(raw["schema_version"], bool):
        raise ManifestError("schema_version must be integer 1")
    try:
        site = validate_site_name(raw["site"])
    except ValueError as error:
        raise ManifestError(str(error)) from error
    server_name = _require_string(raw["server_name"], "server_name")
    if not _SERVER_NAME_RE.fullmatch(server_name):
        raise ManifestError("server_name must be a lowercase DNS name")
    public_ipv4 = _require_string(raw["public_ipv4"], "public_ipv4")
    try:
        parsed_ip = ipaddress.ip_address(public_ipv4)
    except ValueError as error:
        raise ManifestError("public_ipv4 must be an IPv4 address") from error
    if not isinstance(parsed_ip, ipaddress.IPv4Address):
        raise ManifestError("public_ipv4 must be an IPv4 address")
    health_values = raw["https_health_paths"]
    if not isinstance(health_values, list) or not health_values:
        raise ManifestError("https_health_paths must be a non-empty array")
    health_paths = tuple(_validate_url_path(item, "https_health_paths item") for item in health_values)
    if len(set(health_paths)) != len(health_paths):
        raise ManifestError("https_health_paths must not contain duplicates")
    files_value = raw["files"]
    if not isinstance(files_value, list) or not files_value:
        raise ManifestError("files must be a non-empty array")
    files: list[ManifestFile] = []
    destinations: set[str] = {"release.json"}
    for index, item in enumerate(files_value):
        if not isinstance(item, dict):
            raise ManifestError(f"files[{index}] must be an object")
        _require_exact_keys(item, {"source", "destination"}, set(), f"files[{index}]")
        source = _validate_relative_path(item["source"], f"files[{index}].source")
        destination = _validate_relative_path(item["destination"], f"files[{index}].destination")
        if destination in destinations:
            raise ManifestError(f"duplicate destination: {destination}")
        destinations.add(destination)
        files.append(ManifestFile(source=source, destination=destination))
    backend = _parse_backend(raw["backend"]) if "backend" in raw else None
    return SiteManifest(
        schema_version=1,
        site=site,
        server_name=server_name,
        public_ipv4=public_ipv4,
        https_health_paths=health_paths,
        backend=backend,
        files=tuple(files),
    )
