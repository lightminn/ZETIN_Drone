"""Deploy or roll back one validated Oracle web release over fixed SSH commands."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import signal
import stat
import subprocess
import sys
import tempfile
import time
from typing import Callable, Sequence

from .common import validate_release_id, validate_site_name
from .host_release import MAX_COMPRESSED_BYTES, ReleaseError, _read_archive


SSH = "/usr/bin/ssh"
SCP = "/usr/bin/scp"
SUDO = "/usr/bin/sudo"
ROOT_HELPER = "/usr/local/sbin/zetin-web-release"
REMOTE_STAGING_ROOT = "/var/tmp/zetin-web-staging"
COMMAND_TIMEOUT = 30
MAX_HELPER_OUTPUT = 64 * 1024
SSH_CONNECT_TIMEOUT = 10
SSH_OPTIONS = ("-o", "BatchMode=yes", "-o", f"ConnectTimeout={SSH_CONNECT_TIMEOUT}")
TIMEOUT_DRAIN_SECONDS = 1.0

_TARGET_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,252}\Z")
Runner = Callable[..., subprocess.CompletedProcess[bytes]]
_REMOTE_AND = object()


class DeployError(RuntimeError):
	"""Raised when local validation or a fixed deployment step fails."""


class CommandFailure(DeployError):
	"""Preserve a subprocess exit status without exposing its output."""

	def __init__(self, step: str, returncode: int) -> None:
		super().__init__(f"{step} failed with exit code {returncode}")
		self.returncode = returncode


def validate_target(value: str) -> str:
	"""Accept only an option-safe OpenSSH host alias or DNS-style target."""
	if not isinstance(value, str) or not _TARGET_RE.fullmatch(value):
		raise DeployError("target must contain only letters, digits, dot, underscore, or hyphen")
	return value


def _validate_identifiers(target: str, site: str, release_id: str) -> tuple[str, str, str]:
	target = validate_target(target)
	try:
		site = validate_site_name(site)
		release_id = validate_release_id(release_id)
	except ValueError as error:
		raise DeployError(str(error)) from error
	return target, site, release_id


def _read_regular_archive(path: Path) -> bytes:
	if not path.is_absolute():
		raise DeployError("archive path must be absolute")
	flags = os.O_RDONLY | os.O_CLOEXEC
	if hasattr(os, "O_NOFOLLOW"):
		flags |= os.O_NOFOLLOW
	try:
		descriptor = os.open(path, flags)
	except OSError as error:
		raise DeployError("archive must be an existing non-symlink regular file") from error
	try:
		before = os.fstat(descriptor)
		if not stat.S_ISREG(before.st_mode):
			raise DeployError("archive must be a regular file")
		if before.st_size <= 0 or before.st_size > MAX_COMPRESSED_BYTES:
			raise DeployError("archive size is outside the release limit")
		chunks: list[bytes] = []
		remaining = MAX_COMPRESSED_BYTES + 1
		while remaining:
			chunk = os.read(descriptor, min(1024 * 1024, remaining))
			if not chunk:
				break
			chunks.append(chunk)
			remaining -= len(chunk)
		archive_bytes = b"".join(chunks)
		after = os.fstat(descriptor)
		if len(archive_bytes) > MAX_COMPRESSED_BYTES:
			raise DeployError("archive exceeds the compressed release limit")
		if (
			before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns
		) != (
			after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns
		):
			raise DeployError("archive changed while it was being snapshotted")
		return archive_bytes
	finally:
		os.close(descriptor)


def _write_snapshot(parent: Path | None, site: str, release_id: str, archive_bytes: bytes) -> tuple[Path, Path]:
	if parent is not None:
		parent = Path(parent).absolute()
		if not parent.is_dir():
			raise DeployError("snapshot parent must be an existing directory")
	temporary = Path(tempfile.mkdtemp(prefix="zetin-web-deploy-", dir=parent))
	os.chmod(temporary, 0o700)
	snapshot = temporary / f"{site}-{release_id}.tar.gz"
	try:
		descriptor = os.open(snapshot, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o400)
		try:
			view = memoryview(archive_bytes)
			written = 0
			while written < len(view):
				written += os.write(descriptor, view[written:])
			os.fsync(descriptor)
			os.fchmod(descriptor, 0o400)
		finally:
			os.close(descriptor)
		directory_descriptor = os.open(temporary, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
		try:
			os.fsync(directory_descriptor)
		finally:
			os.close(directory_descriptor)
	except BaseException:
		snapshot.unlink(missing_ok=True)
		temporary.rmdir()
		raise
	return temporary, snapshot


def _remove_snapshot(directory: Path, snapshot: Path) -> None:
	try:
		snapshot.unlink(missing_ok=True)
	finally:
		directory.rmdir()


def _quote_remote_token(value: str) -> str:
	if not isinstance(value, str) or "\0" in value:
		raise DeployError("remote command token must be a NUL-free string")
	return shlex.quote(value)


def _ssh_command(target: str, *remote: object) -> list[str]:
	"""Quote every data token for OpenSSH's remote shell; allow only one fixed AND sentinel."""
	quoted: list[str] = []
	for token in remote:
		if token is _REMOTE_AND:
			quoted.append("&&")
		elif isinstance(token, str):
			quoted.append(_quote_remote_token(token))
		else:
			raise DeployError("invalid remote command token")
	return [SSH, *SSH_OPTIONS, "--", target, *quoted]


def _deploy_commands(target: str, site: str, release_id: str, snapshot: Path, digest: str) -> list[list[str]]:
	remote_directory = f"{REMOTE_STAGING_ROOT}/{site}"
	remote_archive = f"{remote_directory}/{release_id}.tar.gz"
	return [
		_ssh_command(target, "/usr/bin/install", "-d", "-m", "0700", remote_directory),
		[SCP, *SSH_OPTIONS, "--", str(snapshot), f"{target}:{remote_archive}"],
		_ssh_command(
			target, SUDO, "-n", ROOT_HELPER, "activate", "--site", site,
			"--release-id", release_id, "--archive", remote_archive, "--sha256", digest,
		),
		_ssh_command(
			target, "/usr/bin/rm", "--", remote_archive, _REMOTE_AND,
			"/usr/bin/rmdir", "--ignore-fail-on-non-empty", "--", remote_directory,
		),
	]


def _rollback_command(target: str, site: str, release_id: str) -> list[str]:
	return _ssh_command(
		target, SUDO, "-n", ROOT_HELPER, "rollback", "--site", site,
		"--release-id", release_id,
	)


def _kill_process_group(process: subprocess.Popen[bytes]) -> None:
	try:
		os.killpg(process.pid, signal.SIGKILL)
	except ProcessLookupError:
		pass
	except OSError:
		try:
			process.kill()
		except ProcessLookupError:
			pass


def _bounded_subprocess(
	command: Sequence[str],
	*,
	timeout: int,
	capture_limit: int,
) -> subprocess.CompletedProcess[bytes]:
	argv = list(command)
	process = subprocess.Popen(
		argv,
		shell=False,
		stdin=subprocess.DEVNULL,
		stdout=subprocess.PIPE,
		stderr=subprocess.PIPE,
		start_new_session=True,
	)
	if process.stdout is None or process.stderr is None:
		process.kill()
		raise OSError("cannot capture subprocess output")
	selector = selectors.DefaultSelector()
	selector.register(process.stdout, selectors.EVENT_READ, "stdout")
	selector.register(process.stderr, selectors.EVENT_READ, "stderr")
	outputs = {"stdout": bytearray(), "stderr": bytearray()}
	deadline = time.monotonic() + timeout
	timed_out = False
	drain_deadline: float | None = None
	try:
		while selector.get_map():
			now = time.monotonic()
			if not timed_out and now >= deadline:
				_kill_process_group(process)
				timed_out = True
				drain_deadline = now + TIMEOUT_DRAIN_SECONDS
			if timed_out and drain_deadline is not None and now >= drain_deadline:
				for key in list(selector.get_map().values()):
					selector.unregister(key.fileobj)
					key.fileobj.close()
				continue
			wait_until = drain_deadline if timed_out else deadline
			events = selector.select(max(0.0, wait_until - now))
			for key, _ in events:
				chunk = os.read(key.fd, 64 * 1024)
				if not chunk:
					selector.unregister(key.fileobj)
					key.fileobj.close()
					continue
				output = outputs[key.data]
				remaining_capture = capture_limit + 1 - len(output)
				if remaining_capture > 0:
					output.extend(chunk[:remaining_capture])
	finally:
		selector.close()
	if not timed_out:
		try:
			returncode = process.wait(timeout=max(0.0, deadline - time.monotonic()))
		except subprocess.TimeoutExpired:
			_kill_process_group(process)
			timed_out = True
			returncode = process.wait(timeout=TIMEOUT_DRAIN_SECONDS)
	else:
		try:
			returncode = process.wait(timeout=TIMEOUT_DRAIN_SECONDS)
		except subprocess.TimeoutExpired:
			_kill_process_group(process)
			returncode = process.wait(timeout=TIMEOUT_DRAIN_SECONDS)
	stdout = bytes(outputs["stdout"])
	stderr = bytes(outputs["stderr"])
	if timed_out:
		raise subprocess.TimeoutExpired(argv, timeout, output=stdout, stderr=stderr)
	return subprocess.CompletedProcess(argv, returncode, stdout, stderr)


def _default_runner(command: Sequence[str], *, timeout: int) -> subprocess.CompletedProcess[bytes]:
	return _bounded_subprocess(command, timeout=timeout, capture_limit=MAX_HELPER_OUTPUT)


def _run(runner: Runner, command: Sequence[str], step: str) -> subprocess.CompletedProcess[bytes]:
	try:
		completed = runner(command, timeout=COMMAND_TIMEOUT)
	except subprocess.TimeoutExpired as error:
		raise CommandFailure(step, 124) from error
	except OSError as error:
		raise CommandFailure(step, 127) from error
	if completed.returncode != 0:
		raise CommandFailure(step, completed.returncode)
	return completed


def _activation_result(output: bytes, requested_release: str) -> dict[str, object]:
	if not isinstance(output, bytes) or len(output) > MAX_HELPER_OUTPUT:
		raise DeployError("activation helper returned invalid JSON")
	try:
		value = json.loads(output)
	except (UnicodeDecodeError, json.JSONDecodeError) as error:
		raise DeployError("activation helper returned invalid JSON") from error
	if not isinstance(value, dict) or set(value) != {"current", "previous", "backend_restarted", "score_reset"}:
		raise DeployError("activation helper returned an invalid result schema")
	try:
		current = validate_release_id(value["current"])
	except (TypeError, ValueError) as error:
		raise DeployError("activation helper returned an invalid current release") from error
	if current != requested_release:
		raise DeployError("activation helper current release does not match the request")
	previous_value = value["previous"]
	if previous_value is not None:
		try:
			previous_value = validate_release_id(previous_value)
		except (TypeError, ValueError) as error:
			raise DeployError("activation helper returned an invalid previous release") from error
	if type(value["backend_restarted"]) is not bool or type(value["score_reset"]) is not bool:
		raise DeployError("activation helper returned invalid restart state")
	return {
		"current": current,
		"previous": previous_value,
		"backend_restarted": value["backend_restarted"],
		"score_reset": value["score_reset"],
	}


def deploy(
	target: str,
	site: str,
	release_id: str,
	archive: Path,
	*,
	dry_run: bool,
	runner: Runner,
	snapshot_parent: Path | None,
) -> dict[str, object] | list[list[str]]:
	target, site, release_id = _validate_identifiers(target, site, release_id)
	archive_bytes = _read_regular_archive(Path(archive))
	try:
		_read_archive(archive_bytes, site, release_id)
	except ReleaseError as error:
		raise DeployError(str(error)) from error
	digest = hashlib.sha256(archive_bytes).hexdigest()
	directory, snapshot = _write_snapshot(snapshot_parent, site, release_id, archive_bytes)
	try:
		commands = _deploy_commands(target, site, release_id, snapshot, digest)
		if dry_run:
			return commands
		_run(runner, commands[0], "remote staging directory creation")
		_run(runner, commands[1], "release upload")
		activation = _run(runner, commands[2], "release activation")
		result = _activation_result(activation.stdout, release_id)
		_run(runner, commands[3], "remote staging cleanup")
		return result
	finally:
		_remove_snapshot(directory, snapshot)


def rollback(target: str, site: str, release_id: str, *, runner: Runner) -> dict[str, object]:
	target, site, release_id = _validate_identifiers(target, site, release_id)
	completed = _run(runner, _rollback_command(target, site, release_id), "release rollback")
	return _activation_result(completed.stdout, release_id)


def _arguments() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description=__doc__)
	subcommands = parser.add_subparsers(dest="command", required=True)
	deploy_parser = subcommands.add_parser("deploy")
	deploy_parser.add_argument("--target", required=True)
	deploy_parser.add_argument("--site", required=True)
	deploy_parser.add_argument("--release-id", required=True)
	deploy_parser.add_argument("--archive", required=True, type=Path)
	deploy_parser.add_argument("--dry-run", action="store_true")
	rollback_parser = subcommands.add_parser("rollback")
	rollback_parser.add_argument("--target", required=True)
	rollback_parser.add_argument("--site", required=True)
	rollback_parser.add_argument("--release-id", required=True)
	return parser


def main(
	argv: Sequence[str] | None = None,
	*,
	runner: Runner | None = None,
	snapshot_parent: Path | None = None,
) -> int:
	arguments = _arguments().parse_args(argv)
	command_runner = runner or _default_runner
	try:
		if arguments.command == "deploy":
			result = deploy(
				arguments.target,
				arguments.site,
				arguments.release_id,
				arguments.archive,
				dry_run=arguments.dry_run,
				runner=command_runner,
				snapshot_parent=snapshot_parent,
			)
			if arguments.dry_run:
				for command in result:
					print(json.dumps(command))
			else:
				print(json.dumps(result, sort_keys=True))
		else:
			result = rollback(
				arguments.target,
				arguments.site,
				arguments.release_id,
				runner=command_runner,
			)
			print(json.dumps(result, sort_keys=True))
	except CommandFailure as error:
		print(json.dumps({"error": str(error)}, sort_keys=True), file=sys.stderr)
		return error.returncode if error.returncode != 0 else 1
	except DeployError as error:
		print(json.dumps({"error": str(error)}, sort_keys=True), file=sys.stderr)
		return 1
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
