"""Install, activate, inspect, and roll back immutable Oracle web releases."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import gzip
import hashlib
import hmac
import io
import ipaddress
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
from typing import Any, Callable, Sequence

from .common import validate_release_id, validate_site_name


APP_ROOT = Path("/srv/zetin-web/apps")
STAGING_ROOT = Path("/var/tmp/zetin-web-staging")
NGINX_TEST = ("/usr/sbin/nginx", "-t")
SYSTEMCTL = "/usr/bin/systemctl"
CURL = "/usr/bin/curl"
CommandRunner = Callable[[Sequence[str]], None]

MAX_COMPRESSED_BYTES = 8 * 1024 * 1024
MAX_DECOMPRESSED_TAR_BYTES = 40 * 1024 * 1024
MAX_ARCHIVE_MEMBERS = 256
MAX_MEMBER_BYTES = 8 * 1024 * 1024
MAX_UNPACKED_BYTES = 32 * 1024 * 1024
ROOT_COMMAND_TIMEOUT_SECONDS = 120
BACKEND_READY_TIMEOUT_SECONDS = 30
BACKEND_READY_RETRY_SECONDS = 1
BACKEND_READY_MAX_ATTEMPTS = 31

_SHA256_RE = re.compile(r"[0-9a-f]{64}\Z")
_SOURCE_COMMIT_RE = re.compile(r"[0-9a-f]{40,64}\Z")
_SERVER_NAME_RE = re.compile(
	r"(?=.{1,253}\Z)(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?)(?:\.(?:[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?))*\Z"
)


class ReleaseError(RuntimeError):
	"""Raised when a release cannot be safely installed or activated."""


def _exact_keys(value: dict[str, Any], required: set[str], optional: set[str], label: str) -> None:
	missing = required - set(value)
	unknown = set(value) - required - optional
	if missing or unknown:
		parts = []
		if missing:
			parts.append(f"missing {', '.join(sorted(missing))}")
		if unknown:
			parts.append(f"unknown {', '.join(sorted(unknown))}")
		raise ReleaseError(f"{label} has " + "; ".join(parts))


def _string(value: Any, label: str) -> str:
	if not isinstance(value, str) or not value:
		raise ReleaseError(f"{label} must be a non-empty string")
	return value


def _relative_path(value: Any, label: str) -> str:
	path = _string(value, label)
	parsed = PurePosixPath(path)
	if (
		"\\" in path
		or parsed.is_absolute()
		or parsed.as_posix() != path
		or path == "."
		or any(part in ("", ".", "..") for part in parsed.parts)
	):
		raise ReleaseError(f"{label} must be a canonical non-traversing relative POSIX path")
	return path


def _url_path(value: Any, label: str) -> str:
	path = _string(value, label)
	if not path.startswith("/") or "\\" in path or "?" in path or "#" in path:
		raise ReleaseError(f"{label} must be an absolute URL path")
	if any(part in (".", "..") for part in PurePosixPath(path).parts):
		raise ReleaseError(f"{label} must not traverse")
	return path


def _validate_manifest(raw: Any, requested_site: str, requested_release: str) -> dict[str, Any]:
	if not isinstance(raw, dict):
		raise ReleaseError("release.json must be an object")
	_exact_keys(
		raw,
		{
			"schema_version", "site", "release_id", "source_commit", "server_name",
			"public_ipv4", "https_health_paths", "members",
		},
		{"backend"},
		"release.json",
	)
	if isinstance(raw["schema_version"], bool) or raw["schema_version"] != 1:
		raise ReleaseError("release.json schema_version must be integer 1")
	try:
		site = validate_site_name(raw["site"])
		release_id = validate_release_id(raw["release_id"])
	except ValueError as error:
		raise ReleaseError(str(error)) from error
	if site != requested_site or release_id != requested_release:
		raise ReleaseError("release.json site or release_id does not match activation request")
	source_commit = _string(raw["source_commit"], "source_commit")
	if not _SOURCE_COMMIT_RE.fullmatch(source_commit):
		raise ReleaseError("source_commit must be a lowercase hexadecimal Git object ID")
	server_name = _string(raw["server_name"], "server_name")
	if not _SERVER_NAME_RE.fullmatch(server_name):
		raise ReleaseError("server_name must be a lowercase DNS name")
	public_ipv4 = _string(raw["public_ipv4"], "public_ipv4")
	try:
		address = ipaddress.ip_address(public_ipv4)
	except ValueError as error:
		raise ReleaseError("public_ipv4 must be an IPv4 address") from error
	if not isinstance(address, ipaddress.IPv4Address):
		raise ReleaseError("public_ipv4 must be an IPv4 address")
	health_values = raw["https_health_paths"]
	if not isinstance(health_values, list) or not health_values:
		raise ReleaseError("https_health_paths must be a non-empty array")
	health_paths = [_url_path(value, "https_health_paths item") for value in health_values]
	if len(set(health_paths)) != len(health_paths):
		raise ReleaseError("https_health_paths must not contain duplicates")

	backend = None
	if "backend" in raw:
		backend_value = raw["backend"]
		if backend_value is not None:
			if not isinstance(backend_value, dict):
				raise ReleaseError("backend must be an object or null")
			_exact_keys(backend_value, {"port", "health_path"}, set(), "backend")
			port = backend_value["port"]
			if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
				raise ReleaseError("backend.port must be an integer from 1 through 65535")
			backend = {"port": port, "health_path": _url_path(backend_value["health_path"], "backend.health_path")}

	members_value = raw["members"]
	if not isinstance(members_value, list) or not members_value:
		raise ReleaseError("members must be a non-empty array")
	members: list[dict[str, Any]] = []
	paths: set[str] = {"release.json"}
	for index, value in enumerate(members_value):
		if not isinstance(value, dict):
			raise ReleaseError(f"members[{index}] must be an object")
		_exact_keys(value, {"path", "sha256", "size", "mode"}, set(), f"members[{index}]")
		path = _relative_path(value["path"], f"members[{index}].path")
		if path in paths:
			raise ReleaseError(f"duplicate release member: {path}")
		paths.add(path)
		digest = _string(value["sha256"], f"members[{index}].sha256")
		if not _SHA256_RE.fullmatch(digest):
			raise ReleaseError(f"members[{index}].sha256 must be lowercase SHA-256")
		size = value["size"]
		if isinstance(size, bool) or not isinstance(size, int) or size < 0:
			raise ReleaseError(f"members[{index}].size must be a non-negative integer")
		mode = value["mode"]
		expected_mode = 0o555 if path == "run" else 0o444
		if isinstance(mode, bool) or mode != expected_mode:
			raise ReleaseError(f"members[{index}].mode does not satisfy the root-owned release contract")
		members.append({"path": path, "sha256": digest, "size": size, "mode": mode})
	member_paths = {member["path"] for member in members}
	for path in member_paths:
		parent = PurePosixPath(path).parent
		while parent.as_posix() != ".":
			if parent.as_posix() in member_paths:
				raise ReleaseError(f"release member path is also a file parent: {parent.as_posix()}")
			parent = parent.parent
	run_present = "run" in member_paths
	backend_members_present = any(path.startswith("backend/") for path in member_paths)
	if backend is None:
		if run_present or backend_members_present:
			raise ReleaseError("inconsistent release: static release contains backend runtime members")
	elif not run_present or not backend_members_present:
		raise ReleaseError("inconsistent release: backend release requires both backend/ and run members")

	validated: dict[str, Any] = {
		"schema_version": 1,
		"site": site,
		"release_id": release_id,
		"source_commit": source_commit,
		"server_name": server_name,
		"public_ipv4": public_ipv4,
		"https_health_paths": health_paths,
		"members": members,
	}
	if backend is not None:
		validated["backend"] = backend
	return validated


def _read_archive(
	archive_bytes: bytes,
	requested_site: str,
	requested_release: str,
) -> tuple[dict[str, Any], list[tuple[tarfile.TarInfo, bytes]], bytes]:
	try:
		with gzip.GzipFile(fileobj=io.BytesIO(archive_bytes), mode="rb") as compressed:
			tar_bytes = compressed.read(MAX_DECOMPRESSED_TAR_BYTES + 1)
		if len(tar_bytes) > MAX_DECOMPRESSED_TAR_BYTES:
			raise ReleaseError("decompressed tar stream exceeds size limit")
		with tarfile.open(fileobj=io.BytesIO(tar_bytes), mode="r:") as archive:
			members: list[tarfile.TarInfo] = []
			total_size = 0
			while True:
				member = archive.next()
				if member is None:
					break
				members.append(member)
				if len(members) > MAX_ARCHIVE_MEMBERS:
					raise ReleaseError(f"archive exceeds {MAX_ARCHIVE_MEMBERS} members")
				if member.size > MAX_MEMBER_BYTES:
					raise ReleaseError(f"archive member exceeds size limit: {member.name}")
				total_size += member.size
				if total_size > MAX_UNPACKED_BYTES:
					raise ReleaseError("archive exceeds total unpacked size limit")
			seen: set[str] = set()
			for member in members:
				name = _relative_path(member.name, "archive member path")
				if name in seen:
					raise ReleaseError(f"duplicate archive member: {name}")
				seen.add(name)
				if not (member.isreg() or member.isdir()):
					raise ReleaseError(f"archive member must be a directory or regular file: {name}")
			release_member = next((member for member in members if member.name == "release.json"), None)
			if release_member is None or not release_member.isreg():
				raise ReleaseError("archive is missing regular release.json")
			if release_member.mode & 0o7777 != 0o444:
				raise ReleaseError("release.json mode must be 0444")
			release_stream = archive.extractfile(release_member)
			if release_stream is None:
				raise ReleaseError("cannot read release.json")
			release_bytes = release_stream.read()
			try:
				raw_manifest = json.loads(release_bytes)
			except (UnicodeDecodeError, json.JSONDecodeError) as error:
				raise ReleaseError(f"invalid release.json: {error}") from error
			manifest = _validate_manifest(raw_manifest, requested_site, requested_release)
			expected = {member["path"]: member for member in manifest["members"]}
			expected_files = {"release.json", *expected}
			actual_files = {member.name for member in members if member.isreg()}
			if actual_files != expected_files:
				missing = expected_files - actual_files
				extra = actual_files - expected_files
				raise ReleaseError(f"archive file set mismatch; missing={sorted(missing)} extra={sorted(extra)}")
			expected_directories: set[str] = set()
			for path in expected:
				parent = PurePosixPath(path).parent
				while parent.as_posix() != ".":
					expected_directories.add(parent.as_posix())
					parent = parent.parent
			actual_directories = {member.name for member in members if member.isdir()}
			if not actual_directories <= expected_directories:
				raise ReleaseError(f"archive has unexpected directories: {sorted(actual_directories - expected_directories)}")
			payload: list[tuple[tarfile.TarInfo, bytes]] = []
			for member in members:
				if member.isdir():
					payload.append((member, b""))
					continue
				stream = archive.extractfile(member)
				if stream is None:
					raise ReleaseError(f"cannot read archive member: {member.name}")
				content = stream.read()
				if member.name != "release.json":
					metadata = expected[member.name]
					if member.size != metadata["size"] or len(content) != metadata["size"]:
						raise ReleaseError(f"release member size mismatch: {member.name}")
					if member.mode & 0o7777 != metadata["mode"]:
						raise ReleaseError(f"release member mode mismatch: {member.name}")
					if hashlib.sha256(content).hexdigest() != metadata["sha256"]:
						raise ReleaseError(f"release member SHA-256 mismatch: {member.name}")
				payload.append((member, content))
	except ReleaseError:
		raise
	except (OSError, tarfile.TarError) as error:
		raise ReleaseError(f"cannot read release archive: {error}") from error
	return manifest, payload, release_bytes


def _open_absolute_directory(path: Path, *, create: bool = False, missing_ok: bool = False) -> int | None:
	absolute = path.absolute()
	if not absolute.is_absolute() or ".." in absolute.parts:
		raise ReleaseError(f"directory path must be absolute and canonical: {path}")
	flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
	try:
		descriptor = os.open("/", flags)
		for component in absolute.parts[1:]:
			try:
				next_descriptor = os.open(component, flags, dir_fd=descriptor)
			except FileNotFoundError:
				if not create:
					if missing_ok:
						os.close(descriptor)
						return None
					raise
				os.mkdir(component, 0o755, dir_fd=descriptor)
				next_descriptor = os.open(component, flags, dir_fd=descriptor)
				os.fchmod(next_descriptor, 0o755)
			os.close(descriptor)
			descriptor = next_descriptor
		return descriptor
	except OSError as error:
		try:
			os.close(descriptor)
		except (OSError, UnboundLocalError):
			pass
		raise ReleaseError(f"cannot resolve directory without following links: {path}: {error}") from error


def _require_owned_mode(metadata: os.stat_result, mode: int, label: str) -> None:
	if metadata.st_uid != os.geteuid() or metadata.st_gid != os.getegid():
		raise ReleaseError(f"{label} must be owned by the release-helper uid/gid")
	if metadata.st_mode & 0o7777 != mode:
		raise ReleaseError(f"{label} mode must be {mode:04o}")


def _open_child_directory(parent_descriptor: int, name: str, *, create: bool, missing_ok: bool = False) -> int | None:
	flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
	try:
		return os.open(name, flags, dir_fd=parent_descriptor)
	except FileNotFoundError:
		if not create:
			if missing_ok:
				return None
			raise ReleaseError(f"required directory is missing: {name}")
		try:
			os.mkdir(name, 0o755, dir_fd=parent_descriptor)
			descriptor = os.open(name, flags, dir_fd=parent_descriptor)
			os.fchmod(descriptor, 0o755)
			return descriptor
		except OSError as error:
			raise ReleaseError(f"cannot create trusted directory {name}: {error}") from error
	except OSError as error:
		raise ReleaseError(f"cannot open trusted directory {name} without following links: {error}") from error


def _prepare_site_tree(app_root: Path, site: str, *, create: bool) -> tuple[Path, Path] | None:
	app_descriptor = _open_absolute_directory(app_root, create=create, missing_ok=not create)
	if app_descriptor is None:
		return None
	site_descriptor = releases_descriptor = None
	try:
		_require_owned_mode(os.fstat(app_descriptor), 0o755, "app root")
		site_descriptor = _open_child_directory(app_descriptor, site, create=create, missing_ok=not create)
		if site_descriptor is None:
			return None
		_require_owned_mode(os.fstat(site_descriptor), 0o755, "site root")
		releases_descriptor = _open_child_directory(site_descriptor, "releases", create=create, missing_ok=not create)
		if releases_descriptor is None:
			return None
		_require_owned_mode(os.fstat(releases_descriptor), 0o755, "releases root")
	finally:
		for descriptor in (releases_descriptor, site_descriptor, app_descriptor):
			if descriptor is not None:
				os.close(descriptor)
	return app_root / site, app_root / site / "releases"


@contextmanager
def _site_lock(site_root: Path):
	site_descriptor = _open_absolute_directory(site_root)
	if site_descriptor is None:
		raise ReleaseError("site root is missing")
	lock_descriptor = None
	try:
		try:
			lock_descriptor = os.open(
				".release.lock",
				os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC,
				0o600,
				dir_fd=site_descriptor,
			)
		except OSError as error:
			raise ReleaseError(f"cannot open site transaction lock: {error}") from error
		metadata = os.fstat(lock_descriptor)
		if not stat.S_ISREG(metadata.st_mode):
			raise ReleaseError("site transaction lock must be a regular file")
		_require_owned_mode(metadata, 0o600, "site transaction lock")
		fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
		yield
	finally:
		if lock_descriptor is not None:
			fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
			os.close(lock_descriptor)
		os.close(site_descriptor)


def _snapshot_staged_archive(
	archive: Path,
	staging_root: Path,
	site: str,
	release_id: str,
	expected_sha256: str,
) -> bytes:
	expected_archive = staging_root.absolute() / site / f"{release_id}.tar.gz"
	if archive.absolute() != expected_archive:
		raise ReleaseError(f"archive must be exactly {expected_archive}")
	root_descriptor = _open_absolute_directory(staging_root)
	site_descriptor = archive_descriptor = None
	try:
		site_descriptor = os.open(
			site,
			os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
			dir_fd=root_descriptor,
		)
		archive_descriptor = os.open(
			f"{release_id}.tar.gz",
			os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC,
			dir_fd=site_descriptor,
		)
		metadata = os.fstat(archive_descriptor)
		if not stat.S_ISREG(metadata.st_mode):
			raise ReleaseError("staged archive must be a regular file")
		if metadata.st_size > MAX_COMPRESSED_BYTES:
			raise ReleaseError("staged archive exceeds compressed size limit")
		chunks: list[bytes] = []
		remaining = MAX_COMPRESSED_BYTES + 1
		while remaining:
			chunk = os.read(archive_descriptor, min(1024 * 1024, remaining))
			if not chunk:
				break
			chunks.append(chunk)
			remaining -= len(chunk)
		archive_bytes = b"".join(chunks)
		if len(archive_bytes) > MAX_COMPRESSED_BYTES:
			raise ReleaseError("staged archive exceeds compressed size limit")
	except ReleaseError:
		raise
	except OSError as error:
		raise ReleaseError(f"cannot snapshot staged archive: {error}") from error
	finally:
		for descriptor in (archive_descriptor, site_descriptor, root_descriptor):
			if descriptor is not None:
				os.close(descriptor)
	if not hmac.compare_digest(hashlib.sha256(archive_bytes).hexdigest(), expected_sha256):
		raise ReleaseError("archive SHA-256 mismatch")
	return archive_bytes


def _copy_release(temporary: Path, manifest: dict[str, Any], payload: list[tuple[tarfile.TarInfo, bytes]]) -> None:
	modes = {member["path"]: member["mode"] for member in manifest["members"]}
	for archive_member, content in payload:
		destination = temporary.joinpath(*PurePosixPath(archive_member.name).parts)
		if archive_member.isdir():
			destination.mkdir(parents=True, exist_ok=True)
			continue
		destination.parent.mkdir(parents=True, exist_ok=True)
		with destination.open("xb") as stream:
			stream.write(content)
		os.chmod(destination, 0o444 if archive_member.name == "release.json" else modes[archive_member.name])
	for directory, child_directories, _ in os.walk(temporary, topdown=False):
		for child in child_directories:
			os.chmod(Path(directory) / child, 0o555)
	os.chmod(temporary, 0o555)


def _discard_temporary(temporary: Path) -> None:
	if not temporary.exists():
		return
	for directory, child_directories, files in os.walk(temporary, topdown=False):
		for name in files:
			(Path(directory) / name).unlink()
		for name in child_directories:
			(Path(directory) / name).rmdir()
	temporary.rmdir()


def _expected_directories(manifest: dict[str, Any]) -> set[str]:
	directories: set[str] = set()
	for member in manifest["members"]:
		parent = PurePosixPath(member["path"]).parent
		while parent.as_posix() != ".":
			directories.add(parent.as_posix())
			parent = parent.parent
	return directories


def _validate_installed_release(
	release: Path,
	manifest: dict[str, Any],
	release_bytes: bytes | None = None,
) -> None:
	try:
		root_mode = release.lstat().st_mode
	except OSError as error:
		raise ReleaseError(f"release does not exist: {release.name}") from error
	if not stat.S_ISDIR(root_mode) or root_mode & 0o777 != 0o555:
		raise ReleaseError(f"release root is not an immutable 0555 directory: {release.name}")
	_require_owned_mode(release.lstat(), 0o555, f"release root {release.name}")
	expected_files = {"release.json": {"mode": 0o444}}
	expected_files.update({member["path"]: member for member in manifest["members"]})
	expected_directories = _expected_directories(manifest)
	actual_files: set[str] = set()
	actual_directories: set[str] = set()
	for directory, child_directories, files in os.walk(release, followlinks=False):
		root = Path(directory)
		for name in child_directories:
			path = root / name
			relative = path.relative_to(release).as_posix()
			mode = path.lstat().st_mode
			if not stat.S_ISDIR(mode) or mode & 0o777 != 0o555:
				raise ReleaseError(f"installed release has unsafe directory: {relative}")
			_require_owned_mode(path.lstat(), 0o555, f"installed release directory {relative}")
			actual_directories.add(relative)
		for name in files:
			path = root / name
			relative = path.relative_to(release).as_posix()
			mode = path.lstat().st_mode
			if not stat.S_ISREG(mode):
				raise ReleaseError(f"installed release has nonregular member: {relative}")
			expected_mode = 0o444 if relative != "run" else 0o555
			_require_owned_mode(path.lstat(), expected_mode, f"installed release file {relative}")
			actual_files.add(relative)
	if actual_files != set(expected_files) or actual_directories != expected_directories:
		raise ReleaseError("installed immutable release file set differs from release.json")
	for relative, metadata in expected_files.items():
		path = release.joinpath(*PurePosixPath(relative).parts)
		if path.stat().st_mode & 0o777 != metadata["mode"]:
			raise ReleaseError(f"installed release mode mismatch: {relative}")
		content = path.read_bytes()
		if relative == "release.json":
			if release_bytes is not None and content != release_bytes:
				raise ReleaseError("installed release.json differs from staged release.json")
			continue
		if len(content) != metadata["size"] or hashlib.sha256(content).hexdigest() != metadata["sha256"]:
			raise ReleaseError(f"installed release content mismatch: {relative}")


def _current_release(site_root: Path) -> str | None:
	current = site_root / "current"
	try:
		mode = current.lstat().st_mode
	except FileNotFoundError:
		return None
	except OSError as error:
		raise ReleaseError(f"cannot inspect current: {error}") from error
	if not stat.S_ISLNK(mode):
		raise ReleaseError("current must be a relative release symlink")
	target = os.readlink(current)
	if not target.startswith("releases/"):
		raise ReleaseError("current must point to releases/<release-id>")
	release_id = target.removeprefix("releases/")
	try:
		validate_release_id(release_id)
	except ValueError as error:
		raise ReleaseError("current has an invalid release target") from error
	if target != f"releases/{release_id}":
		raise ReleaseError("current must point directly to one release")
	return release_id


def _load_installed_manifest(release: Path, site: str, release_id: str) -> dict[str, Any]:
	release_json = release / "release.json"
	try:
		mode = release_json.lstat().st_mode
		if not stat.S_ISREG(mode):
			raise ReleaseError(f"installed release.json is not regular: {release_id}")
		release_bytes = release_json.read_bytes()
		raw = json.loads(release_bytes)
	except ReleaseError:
		raise
	except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
		raise ReleaseError(f"cannot read installed release.json for {release_id}: {error}") from error
	manifest = _validate_manifest(raw, site, release_id)
	_validate_installed_release(release, manifest)
	return manifest


def _backend_fingerprint(manifest: dict[str, Any]) -> tuple[object, ...] | None:
	backend = manifest.get("backend")
	if backend is None:
		return None
	code = tuple(sorted(
		(member["path"], member["sha256"])
		for member in manifest["members"]
		if member["path"] == "run" or member["path"].startswith("backend/")
	))
	return (backend["port"], backend["health_path"], code)


def _runtime_action(previous: dict[str, Any] | None, target: dict[str, Any]) -> str | None:
	previous_has_backend = previous is not None and previous.get("backend") is not None
	target_has_backend = target.get("backend") is not None
	if target_has_backend:
		if not previous_has_backend or _backend_fingerprint(previous) != _backend_fingerprint(target):
			return "restart"
		return None
	if previous_has_backend:
		return "stop"
	return None


def _run_runtime_action(runner: CommandRunner, site: str, action: str | None) -> None:
	if action is not None:
		_command(runner, (SYSTEMCTL, action, f"zetin-webapp@{site}.service"))


def _reconcile_stale_next(site_root: Path, site: str) -> None:
	next_link = site_root / "current.next"
	try:
		mode = next_link.lstat().st_mode
	except FileNotFoundError:
		return
	except OSError as error:
		raise ReleaseError(f"cannot inspect current.next: {error}") from error
	if not stat.S_ISLNK(mode):
		raise ReleaseError("stale current.next must be a validated release symlink")
	target = os.readlink(next_link)
	if not target.startswith("releases/"):
		raise ReleaseError("stale current.next has an unsafe target")
	release_id = target.removeprefix("releases/")
	try:
		validate_release_id(release_id)
	except ValueError as error:
		raise ReleaseError("stale current.next has an invalid release target") from error
	if target != f"releases/{release_id}":
		raise ReleaseError("stale current.next must point directly to one release")
	_load_installed_manifest(site_root / "releases" / release_id, site, release_id)
	try:
		next_link.unlink()
	except OSError as error:
		raise ReleaseError(f"cannot remove validated stale current.next: {error}") from error


def _atomic_switch(site_root: Path, site: str, release_id: str) -> None:
	next_link = site_root / "current.next"
	_reconcile_stale_next(site_root, site)
	try:
		os.symlink(f"releases/{release_id}", next_link)
		os.replace(next_link, site_root / "current")
	except OSError as error:
		try:
			if next_link.is_symlink():
				next_link.unlink()
		except OSError:
			pass
		raise ReleaseError(f"cannot switch current release: {error}") from error


def _command(runner: CommandRunner, command: Sequence[str]) -> None:
	try:
		runner(command)
	except Exception as error:
		raise ReleaseError(f"command failed: {' '.join(command)}: {error}") from error


def _remove_new_current(site_root: Path, release_id: str) -> None:
	current = site_root / "current"
	try:
		if not current.is_symlink() or os.readlink(current) != f"releases/{release_id}":
			raise ReleaseError("current changed unexpectedly during activation recovery")
		current.unlink()
	except ReleaseError:
		raise
	except OSError as error:
		raise ReleaseError(f"cannot remove failed first-deploy current link: {error}") from error


def _recover_switch(
	site_root: Path,
	site: str,
	new_release: str,
	previous: str | None,
	previous_manifest: dict[str, Any] | None,
	new_manifest: dict[str, Any],
	runner: CommandRunner,
) -> list[str]:
	errors: list[str] = []
	restored = False
	try:
		if previous is None:
			_remove_new_current(site_root, new_release)
		else:
			_atomic_switch(site_root, site, previous)
		restored = True
	except ReleaseError as error:
		errors.append(str(error))
	if previous_manifest is not None:
		recovery_action = _runtime_action(new_manifest, previous_manifest)
	else:
		recovery_action = "stop" if new_manifest.get("backend") is not None else None
	if restored and recovery_action is not None:
		try:
			_run_runtime_action(runner, site, recovery_action)
		except ReleaseError as error:
			errors.append(str(error))
	if restored:
		try:
			_command(runner, (SYSTEMCTL, "reload", "nginx"))
		except ReleaseError as error:
			errors.append(str(error))
	if restored and previous_manifest is not None:
		try:
			_verify_release_health(
				previous_manifest,
				runner,
				wait_for_backend=recovery_action == "restart",
			)
		except ReleaseError as error:
			errors.append(str(error))
	return errors


def _curl_base() -> tuple[str, ...]:
	return (
		CURL, "--fail", "--silent", "--show-error", "--output", "/dev/null",
		"--noproxy", "*", "--max-time", "5",
	)


def _backend_health_command(manifest: dict[str, Any]) -> tuple[str, ...] | None:
	backend = manifest.get("backend")
	if not isinstance(backend, dict):
		return None
	return (*_curl_base(), f"http://127.0.0.1:{backend['port']}{backend['health_path']}")


def _wait_for_backend(runner: CommandRunner, command: Sequence[str]) -> None:
	deadline = time.monotonic() + BACKEND_READY_TIMEOUT_SECONDS
	last_error: ReleaseError | None = None
	for _attempt in range(BACKEND_READY_MAX_ATTEMPTS):
		try:
			_command(runner, command)
			return
		except ReleaseError as error:
			last_error = error
		remaining = deadline - time.monotonic()
		if remaining <= 0:
			break
		time.sleep(min(BACKEND_READY_RETRY_SECONDS, remaining))
	if last_error is None:
		raise ReleaseError("backend readiness check did not run")
	raise ReleaseError(f"backend readiness deadline exceeded: {last_error}") from last_error


def _verify_release_health(
	manifest: dict[str, Any],
	runner: CommandRunner,
	*,
	wait_for_backend: bool,
) -> None:
	backend_command = _backend_health_command(manifest)
	if backend_command is not None:
		if wait_for_backend:
			_wait_for_backend(runner, backend_command)
		else:
			_command(runner, backend_command)
	for path in manifest["https_health_paths"]:
		domain = manifest["server_name"]
		_command(
			runner,
			(*_curl_base(), "--resolve", f"{domain}:443:127.0.0.1", f"https://{domain}{path}"),
		)


def _reconcile_current(
	*,
	site_root: Path,
	site: str,
	release_id: str,
	manifest: dict[str, Any],
	runner: CommandRunner,
) -> dict[str, object]:
	"""Conservatively finish a possibly interrupted activation of current."""
	_command(runner, NGINX_TEST)
	_atomic_switch(site_root, site, release_id)
	runtime_action = "restart" if manifest.get("backend") is not None else "stop"
	try:
		_run_runtime_action(runner, site, runtime_action)
		_command(runner, (SYSTEMCTL, "reload", "nginx"))
		_verify_release_health(
			manifest,
			runner,
			wait_for_backend=runtime_action == "restart",
		)
	except ReleaseError as error:
		raise ReleaseError(f"current release reconciliation failed: {error}") from error
	return {
		"current": release_id,
		"previous": release_id,
		"backend_restarted": runtime_action == "restart",
		"score_reset": True,
	}


def _switch_and_verify(
	*,
	site_root: Path,
	site: str,
	release_id: str,
	manifest: dict[str, Any],
	previous: str | None,
	previous_manifest: dict[str, Any] | None,
	runner: CommandRunner,
) -> dict[str, object]:
	_command(runner, NGINX_TEST)
	_atomic_switch(site_root, site, release_id)
	runtime_action = _runtime_action(previous_manifest, manifest)
	backend_restarted = runtime_action == "restart"
	try:
		_run_runtime_action(runner, site, runtime_action)
		_command(runner, (SYSTEMCTL, "reload", "nginx"))
		_verify_release_health(
			manifest,
			runner,
			wait_for_backend=runtime_action == "restart",
		)
	except ReleaseError as activation_error:
		rollback_errors = _recover_switch(
			site_root,
			site,
			release_id,
			previous,
			previous_manifest,
			manifest,
			runner,
		)
		if rollback_errors:
			raise ReleaseError(
				f"{activation_error}; rollback failed: {'; '.join(rollback_errors)}"
			) from activation_error
		raise ReleaseError(f"{activation_error}; activation rolled back") from activation_error
	return {
		"current": release_id,
		"previous": previous,
		"backend_restarted": backend_restarted,
		"score_reset": runtime_action is not None,
	}


def _activate_prevalidated(
	*,
	site_root: Path,
	releases: Path,
	site: str,
	release_id: str,
	manifest: dict[str, Any],
	payload: list[tuple[tarfile.TarInfo, bytes]],
	release_bytes: bytes,
	runner: CommandRunner,
) -> dict[str, object]:
	previous = _current_release(site_root)
	previous_manifest = (
		_load_installed_manifest(releases / previous, site, previous)
		if previous is not None else None
	)
	final_release = releases / release_id
	if os.path.lexists(final_release):
		_validate_installed_release(final_release, manifest, release_bytes)
		if previous == release_id:
			return _reconcile_current(
				site_root=site_root,
				site=site,
				release_id=release_id,
				manifest=manifest,
				runner=runner,
			)
	else:
		temporary = Path(tempfile.mkdtemp(prefix=f".{release_id}.tmp.", dir=releases))
		try:
			_copy_release(temporary, manifest, payload)
			os.replace(temporary, final_release)
		except Exception as error:
			try:
				_discard_temporary(temporary)
			except OSError as cleanup_error:
				raise ReleaseError(f"release installation failed: {error}; temporary cleanup failed: {cleanup_error}") from error
			if isinstance(error, ReleaseError):
				raise
			raise ReleaseError(f"release installation failed: {error}") from error
	return _switch_and_verify(
		site_root=site_root,
		site=site,
		release_id=release_id,
		manifest=manifest,
		previous=previous,
		previous_manifest=previous_manifest,
		runner=runner,
	)


def activate(
	site: str,
	release_id: str,
	archive: Path,
	sha256: str,
	*,
	app_root: Path = APP_ROOT,
	staging_root: Path = STAGING_ROOT,
	runner: CommandRunner,
) -> dict[str, object]:
	"""Install and atomically activate one staged release archive."""
	try:
		site = validate_site_name(site)
		release_id = validate_release_id(release_id)
	except ValueError as error:
		raise ReleaseError(str(error)) from error
	archive = Path(archive).absolute()
	if not isinstance(sha256, str) or not _SHA256_RE.fullmatch(sha256):
		raise ReleaseError("expected SHA-256 must be 64 lowercase hexadecimal characters")
	archive_bytes = _snapshot_staged_archive(archive, Path(staging_root), site, release_id, sha256)
	manifest, payload, release_bytes = _read_archive(archive_bytes, site, release_id)

	prepared = _prepare_site_tree(Path(app_root), site, create=True)
	if prepared is None:
		raise ReleaseError("cannot prepare site tree")
	site_root, releases = prepared
	with _site_lock(site_root):
		return _activate_prevalidated(
			site_root=site_root,
			releases=releases,
			site=site,
			release_id=release_id,
			manifest=manifest,
			payload=payload,
			release_bytes=release_bytes,
			runner=runner,
		)


def rollback(
	site: str,
	release_id: str,
	*,
	app_root: Path = APP_ROOT,
	runner: CommandRunner,
) -> dict[str, object]:
	"""Atomically switch to one validated immutable release already on the host."""
	try:
		site = validate_site_name(site)
		release_id = validate_release_id(release_id)
	except ValueError as error:
		raise ReleaseError(str(error)) from error
	prepared = _prepare_site_tree(Path(app_root), site, create=False)
	if prepared is None:
		raise ReleaseError("cannot roll back a missing site")
	site_root, releases = prepared
	with _site_lock(site_root):
		previous = _current_release(site_root)
		if previous is None:
			raise ReleaseError("cannot roll back a site with no current release")
		previous_manifest = _load_installed_manifest(releases / previous, site, previous)
		manifest = _load_installed_manifest(releases / release_id, site, release_id)
		if previous == release_id:
			return {"current": release_id, "previous": previous, "backend_restarted": False, "score_reset": False}
		return _switch_and_verify(
			site_root=site_root,
			site=site,
			release_id=release_id,
			manifest=manifest,
			previous=previous,
			previous_manifest=previous_manifest,
			runner=runner,
		)


def status(site: str, *, app_root: Path = APP_ROOT) -> dict[str, object]:
	"""Return the validated current release without changing the filesystem."""
	try:
		site = validate_site_name(site)
	except ValueError as error:
		raise ReleaseError(str(error)) from error
	prepared = _prepare_site_tree(Path(app_root), site, create=False)
	if prepared is None:
		return {"current": None}
	site_root, _ = prepared
	current = _current_release(site_root)
	if current is not None:
		_load_installed_manifest(site_root / "releases" / current, site, current)
	return {"current": current}


def _default_runner(command: Sequence[str]) -> None:
	subprocess.run(
		list(command),
		check=True,
		timeout=ROOT_COMMAND_TIMEOUT_SECONDS,
		stdout=subprocess.DEVNULL,
	)


def _arguments() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description=__doc__)
	subcommands = parser.add_subparsers(dest="command", required=True)
	activate_parser = subcommands.add_parser("activate", help="install and activate a staged release")
	activate_parser.add_argument("--site", required=True)
	activate_parser.add_argument("--release-id", required=True)
	activate_parser.add_argument("--archive", required=True, type=Path)
	activate_parser.add_argument("--sha256", required=True)
	rollback_parser = subcommands.add_parser("rollback", help="activate an existing release")
	rollback_parser.add_argument("--site", required=True)
	rollback_parser.add_argument("--release-id", required=True)
	status_parser = subcommands.add_parser("status", help="show the validated current release")
	status_parser.add_argument("--site", required=True)
	return parser


def main(
	argv: Sequence[str] | None = None,
	*,
	app_root: Path = APP_ROOT,
	staging_root: Path = STAGING_ROOT,
	runner: CommandRunner | None = None,
	require_root: bool = True,
) -> int:
	"""Run the root-side CLI and emit exactly one JSON result object."""
	arguments = _arguments().parse_args(argv)
	if require_root and os.geteuid() != 0:
		print(json.dumps({"error": "zetin-web-release must run as root"}, sort_keys=True), file=sys.stderr)
		return 1
	command_runner = runner or _default_runner
	try:
		if arguments.command == "activate":
			result = activate(
				arguments.site,
				arguments.release_id,
				arguments.archive,
				arguments.sha256,
				app_root=app_root,
				staging_root=staging_root,
				runner=command_runner,
			)
		elif arguments.command == "rollback":
			result = rollback(
				arguments.site,
				arguments.release_id,
				app_root=app_root,
				runner=command_runner,
			)
		else:
			result = status(arguments.site, app_root=app_root)
	except ReleaseError as error:
		print(json.dumps({"error": str(error)}, sort_keys=True), file=sys.stderr)
		return 1
	print(json.dumps(result, sort_keys=True))
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
