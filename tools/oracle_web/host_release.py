"""Install, activate, inspect, and roll back immutable Oracle web releases."""

from __future__ import annotations

import argparse
import hashlib
import hmac
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
from typing import Any, Callable, Sequence

from .common import sha256_file, validate_release_id, validate_site_name


APP_ROOT = Path("/srv/zetin-web/apps")
STAGING_ROOT = Path("/var/tmp/zetin-web-staging")
NGINX_TEST = ("/usr/sbin/nginx", "-t")
SYSTEMCTL = "/usr/bin/systemctl"
CURL = "/usr/bin/curl"
CommandRunner = Callable[[Sequence[str]], None]

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
		if not isinstance(backend_value, dict):
			raise ReleaseError("backend must be an object")
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
	archive_path: Path,
	requested_site: str,
	requested_release: str,
) -> tuple[dict[str, Any], list[tuple[tarfile.TarInfo, bytes]], bytes]:
	try:
		with tarfile.open(archive_path, "r:gz") as archive:
			members = archive.getmembers()
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
			actual_directories.add(relative)
		for name in files:
			path = root / name
			relative = path.relative_to(release).as_posix()
			mode = path.lstat().st_mode
			if not stat.S_ISREG(mode):
				raise ReleaseError(f"installed release has nonregular member: {relative}")
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
	code = tuple(
		(member["path"], member["sha256"])
		for member in manifest["members"]
		if member["path"] == "run" or member["path"].startswith("backend/")
	)
	return (backend["port"], backend["health_path"], code)


def _atomic_switch(site_root: Path, release_id: str) -> None:
	next_link = site_root / "current.next"
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
	backend_restarted: bool,
	runner: CommandRunner,
) -> list[str]:
	errors: list[str] = []
	try:
		if previous is None:
			_remove_new_current(site_root, new_release)
		else:
			_atomic_switch(site_root, previous)
	except ReleaseError as error:
		errors.append(str(error))
	if backend_restarted:
		command = (
			(SYSTEMCTL, "restart", f"zetin-webapp@{site}.service")
			if previous_manifest is not None and previous_manifest.get("backend") is not None
			else (SYSTEMCTL, "stop", f"zetin-webapp@{site}.service")
		)
		try:
			_command(runner, command)
		except ReleaseError as error:
			errors.append(str(error))
	try:
		_command(runner, (SYSTEMCTL, "reload", "nginx"))
	except ReleaseError as error:
		errors.append(str(error))
	return errors


def _curl_base() -> tuple[str, ...]:
	return (CURL, "--fail", "--silent", "--show-error", "--max-time", "5")


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
	_atomic_switch(site_root, release_id)
	backend = manifest.get("backend")
	backend_restarted = backend is not None and (
		previous_manifest is None
		or _backend_fingerprint(previous_manifest) != _backend_fingerprint(manifest)
	)
	try:
		if backend_restarted:
			_command(runner, (SYSTEMCTL, "restart", f"zetin-webapp@{site}.service"))
		_command(runner, (SYSTEMCTL, "reload", "nginx"))
		if isinstance(backend, dict):
			_command(runner, (*_curl_base(), f"http://127.0.0.1:{backend['port']}{backend['health_path']}"))
		for path in manifest["https_health_paths"]:
			domain = manifest["server_name"]
			_command(runner, (*_curl_base(), "--resolve", f"{domain}:443:127.0.0.1", f"https://{domain}{path}"))
	except ReleaseError as activation_error:
		rollback_errors = _recover_switch(
			site_root,
			site,
			release_id,
			previous,
			previous_manifest,
			backend_restarted,
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
		"score_reset": backend_restarted,
	}


def _ensure_site_directories(app_root: Path, site: str) -> tuple[Path, Path]:
	site_root = app_root / site
	releases = site_root / "releases"
	try:
		app_root.mkdir(parents=True, exist_ok=True, mode=0o755)
		if not stat.S_ISDIR(app_root.lstat().st_mode):
			raise ReleaseError(f"app root is not a directory: {app_root}")
		for path in (site_root, releases):
			try:
				mode = path.lstat().st_mode
			except FileNotFoundError:
				path.mkdir(mode=0o755)
				mode = path.lstat().st_mode
			if not stat.S_ISDIR(mode):
				raise ReleaseError(f"site path is not a directory: {path}")
			os.chmod(path, 0o755)
	except ReleaseError:
		raise
	except OSError as error:
		raise ReleaseError(f"cannot prepare site directories: {error}") from error
	return site_root, releases


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
	expected_archive = Path(staging_root).absolute() / site / f"{release_id}.tar.gz"
	if archive != expected_archive:
		raise ReleaseError(f"archive must be exactly {expected_archive}")
	try:
		archive_mode = archive.lstat().st_mode
	except OSError as error:
		raise ReleaseError(f"cannot inspect staged archive: {error}") from error
	if not stat.S_ISREG(archive_mode):
		raise ReleaseError("staged archive must be a regular file, not a link or device")
	if not isinstance(sha256, str) or not _SHA256_RE.fullmatch(sha256):
		raise ReleaseError("expected SHA-256 must be 64 lowercase hexadecimal characters")
	if not hmac.compare_digest(sha256_file(archive), sha256):
		raise ReleaseError("archive SHA-256 mismatch")
	manifest, payload, release_bytes = _read_archive(archive, site, release_id)

	site_root, releases = _ensure_site_directories(Path(app_root), site)
	previous = _current_release(site_root)
	previous_manifest = (
		_load_installed_manifest(releases / previous, site, previous)
		if previous is not None else None
	)
	final_release = releases / release_id
	if final_release.exists():
		_validate_installed_release(final_release, manifest, release_bytes)
		if previous == release_id:
			return {"current": release_id, "previous": previous, "backend_restarted": False, "score_reset": False}
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
	site_root = Path(app_root) / site
	releases = site_root / "releases"
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
	site_root = Path(app_root) / site
	try:
		site_mode = site_root.lstat().st_mode
	except FileNotFoundError:
		return {"current": None}
	except OSError as error:
		raise ReleaseError(f"cannot inspect site: {error}") from error
	if not stat.S_ISDIR(site_mode):
		raise ReleaseError("site root is not a directory")
	current = _current_release(site_root)
	if current is not None:
		_load_installed_manifest(site_root / "releases" / current, site, current)
	return {"current": current}


def _default_runner(command: Sequence[str]) -> None:
	subprocess.run(list(command), check=True)


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
