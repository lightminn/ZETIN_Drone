"""Add or remove only the ZETIN HTTP rule without rewriting firewall drift."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import shlex
import stat
import subprocess
import sys
import tempfile
from typing import Callable, Sequence


PERSISTENT_RULES = Path("/etc/iptables/rules.v4")
BACKUP_DIR = Path("/var/backups/zetin-web/firewall")
IPTABLES = "/usr/sbin/iptables"
IPTABLES_SAVE = "/usr/sbin/iptables-save"
HTTP_RULE = "-A INPUT -p tcp -m tcp --dport 80 -m comment --comment zetin-web:http -j ACCEPT"
HTTP_COMMENT = "zetin-web:http"
HTTP_ARGUMENTS = (
	"-p", "tcp", "-m", "tcp", "--dport", "80", "-m", "comment",
	"--comment", "zetin-web:http", "-j", "ACCEPT",
)
UNTAGGED_HTTP_TOKENS = (
	"-A", "INPUT", "-p", "tcp", "-m", "tcp", "--dport", "80", "-j", "ACCEPT",
)

SaveRunner = Callable[[], str]
CommandRunner = Callable[[Sequence[str]], None]
ReplaceFunction = Callable[[str | bytes | os.PathLike[str] | os.PathLike[bytes], str | bytes | os.PathLike[str] | os.PathLike[bytes]], None]


class FirewallError(RuntimeError):
	"""Raised when the exact HTTP rule cannot be changed transactionally."""


@dataclass(frozen=True)
class _FilterPolicy:
	lines: list[str]
	start: int
	end: int
	reject_line: int
	live_position: int
	equivalent_before_reject: bool


@dataclass(frozen=True)
class _FileSnapshot:
	text: str
	content: bytes
	mode: int
	uid: int
	gid: int


def _line_body(line: str) -> str:
	return line[:-2] if line.endswith("\r\n") else line[:-1] if line.endswith(("\n", "\r")) else line


def _tokens(line: str) -> tuple[str, ...]:
	try:
		return tuple(shlex.split(_line_body(line), comments=False, posix=True))
	except ValueError as error:
		raise FirewallError(f"cannot parse iptables-save rule: {error}") from error


def _is_unconditional_reject(tokens: tuple[str, ...]) -> bool:
	if tokens[:4] != ("-A", "INPUT", "-j", "REJECT"):
		return False
	return len(tokens) == 4 or (len(tokens) == 6 and tokens[4] == "--reject-with")


def _without_comment(tokens: tuple[str, ...]) -> tuple[str, ...]:
	cleaned: list[str] = []
	index = 0
	while index < len(tokens):
		if tokens[index:index + 2] == ("-m", "comment"):
			index += 2
			continue
		if tokens[index] == "--comment" and index + 1 < len(tokens):
			index += 2
			continue
		cleaned.append(tokens[index])
		index += 1
	return tuple(cleaned)


def _is_equivalent_http(tokens: tuple[str, ...]) -> bool:
	return _without_comment(tokens) == UNTAGGED_HTTP_TOKENS


def _is_tagged_http(tokens: tuple[str, ...]) -> bool:
	comment_modules = sum(
		1 for index in range(len(tokens) - 1)
		if tokens[index:index + 2] == ("-m", "comment")
	)
	comment_values = [
		tokens[index + 1] for index, token in enumerate(tokens[:-1])
		if token == "--comment"
	]
	return (
		_is_equivalent_http(tokens)
		and comment_modules == 1
		and comment_values == [HTTP_COMMENT]
	)


def _filter_policy(text: str) -> _FilterPolicy:
	lines = text.splitlines(keepends=True)
	filter_starts = [index for index, line in enumerate(lines) if _line_body(line) == "*filter"]
	if len(filter_starts) != 1:
		raise FirewallError("iptables-save text must contain exactly one filter table")
	start = filter_starts[0]
	end = next((index for index in range(start + 1, len(lines)) if _line_body(lines[index]) == "COMMIT"), None)
	if end is None:
		raise FirewallError("filter table is missing COMMIT")
	input_count = 0
	equivalent = False
	for index in range(start + 1, end):
		tokens = _tokens(lines[index])
		if tokens[:2] == ("-A", "INPUT"):
			input_count += 1
		if _is_unconditional_reject(tokens):
			return _FilterPolicy(
				lines=lines,
				start=start,
				end=end,
				reject_line=index,
				live_position=input_count,
				equivalent_before_reject=equivalent,
			)
		if _is_equivalent_http(tokens):
			equivalent = True
	raise FirewallError("filter INPUT chain has no unconditional REJECT")


def ensure_http_rule(text: str) -> tuple[str, bool]:
	"""Insert the canonical HTTP rule while preserving every existing byte."""
	policy = _filter_policy(text)
	if policy.equivalent_before_reject:
		return text, False
	reject = policy.lines[policy.reject_line]
	newline = "\r\n" if reject.endswith("\r\n") else "\n"
	policy.lines.insert(policy.reject_line, HTTP_RULE + newline)
	return "".join(policy.lines), True


def rollback_http_rule(text: str) -> tuple[str, bool]:
	"""Remove one exact canonical tagged rule from the filter table only."""
	policy = _filter_policy(text)
	for index in range(policy.start + 1, policy.end):
		if _is_tagged_http(_tokens(policy.lines[index])):
			del policy.lines[index]
			return "".join(policy.lines), True
	return text, False


def _read_snapshot(path: Path) -> _FileSnapshot:
	try:
		metadata = path.lstat()
		if not stat.S_ISREG(metadata.st_mode):
			raise FirewallError(f"persistent rules must be a regular file: {path}")
		content = path.read_bytes()
		text = content.decode("utf-8")
	except FirewallError:
		raise
	except (OSError, UnicodeDecodeError) as error:
		raise FirewallError(f"cannot read persistent rules {path}: {error}") from error
	return _FileSnapshot(
		text=text,
		content=content,
		mode=stat.S_IMODE(metadata.st_mode),
		uid=metadata.st_uid,
		gid=metadata.st_gid,
	)


def _write_all(descriptor: int, content: bytes) -> None:
	view = memoryview(content)
	while view:
		written = os.write(descriptor, view)
		if written <= 0:
			raise OSError("short write")
		view = view[written:]


def _backup(path: Path, snapshot: _FileSnapshot, backup_dir: Path) -> Path:
	try:
		backup_dir.mkdir(parents=True, mode=0o700, exist_ok=True)
		stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
		backup = backup_dir / f"{path.name}.{stamp}.{os.getpid()}.bak"
		descriptor = os.open(backup, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
		try:
			os.fchmod(descriptor, 0o600)
			_write_all(descriptor, snapshot.content)
			os.fsync(descriptor)
		finally:
			os.close(descriptor)
	except OSError as error:
		raise FirewallError(f"cannot back up persistent rules: {error}") from error
	return backup


def _atomic_write(
	path: Path,
	content: bytes,
	snapshot: _FileSnapshot,
	replace_func: ReplaceFunction,
) -> None:
	descriptor = -1
	temporary_name: str | None = None
	try:
		descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
		os.fchmod(descriptor, 0o600)
		_write_all(descriptor, content)
		os.fsync(descriptor)
		os.fchown(descriptor, snapshot.uid, snapshot.gid)
		os.fchmod(descriptor, snapshot.mode)
		os.close(descriptor)
		descriptor = -1
		replace_func(temporary_name, path)
		temporary_name = None
		directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
		try:
			os.fsync(directory)
		finally:
			os.close(directory)
	finally:
		if descriptor >= 0:
			os.close(descriptor)
		if temporary_name is not None:
			try:
				os.unlink(temporary_name)
			except FileNotFoundError:
				pass


def _default_save_runner() -> str:
	completed = subprocess.run(
		[IPTABLES_SAVE],
		check=True,
		stdout=subprocess.PIPE,
		stderr=subprocess.PIPE,
		text=True,
	)
	return completed.stdout


def _default_command_runner(command: Sequence[str]) -> None:
	subprocess.run(command, check=True)


def _insert_command(position: int) -> tuple[str, ...]:
	return (IPTABLES, "-I", "INPUT", str(position), *HTTP_ARGUMENTS)


def _delete_command() -> tuple[str, ...]:
	return (IPTABLES, "-D", "INPUT", *HTTP_ARGUMENTS)


def _restore_if_changed(
	path: Path,
	snapshot: _FileSnapshot,
	replace_func: ReplaceFunction,
) -> str | None:
	try:
		if path.read_bytes() != snapshot.content:
			_atomic_write(path, snapshot.content, snapshot, replace_func)
	except OSError as error:
		return str(error)
	return None


def ensure_http(
	*,
	persistent_path: Path = PERSISTENT_RULES,
	backup_dir: Path = BACKUP_DIR,
	save_runner: SaveRunner = _default_save_runner,
	command_runner: CommandRunner = _default_command_runner,
	replace_func: ReplaceFunction = os.replace,
) -> dict[str, bool]:
	"""Transactionally add the canonical rule to live and persistent rules."""
	snapshot = _read_snapshot(persistent_path)
	persistent_after, persistent_changed = ensure_http_rule(snapshot.text)
	try:
		live_text = save_runner()
		live_policy = _filter_policy(live_text)
	except (OSError, subprocess.SubprocessError, RuntimeError) as error:
		raise FirewallError(f"cannot inspect live firewall: {error}") from error
	live_changed = not live_policy.equivalent_before_reject
	if not persistent_changed and not live_changed:
		return {"persistent_changed": False, "live_changed": False}
	if persistent_changed:
		_backup(persistent_path, snapshot, backup_dir)
	if live_changed:
		try:
			command_runner(_insert_command(live_policy.live_position))
		except (OSError, subprocess.SubprocessError, RuntimeError) as error:
			raise FirewallError(f"cannot insert live HTTP rule; persistent rules unchanged: {error}") from error
	if persistent_changed:
		try:
			_atomic_write(persistent_path, persistent_after.encode("utf-8"), snapshot, replace_func)
		except OSError as error:
			restore_error = _restore_if_changed(persistent_path, snapshot, replace_func)
			compensation_error = None
			if live_changed:
				try:
					command_runner(_delete_command())
				except (OSError, subprocess.SubprocessError, RuntimeError) as rollback_error:
					compensation_error = str(rollback_error)
			details = [str(error)]
			if restore_error:
				details.append(f"persistent restore failed: {restore_error}")
			if compensation_error:
				details.append(f"live rollback failed: {compensation_error}")
			raise FirewallError("cannot replace persistent rules; " + "; ".join(details)) from error
	return {"persistent_changed": persistent_changed, "live_changed": live_changed}


def rollback_http(
	*,
	persistent_path: Path = PERSISTENT_RULES,
	backup_dir: Path = BACKUP_DIR,
	save_runner: SaveRunner = _default_save_runner,
	command_runner: CommandRunner = _default_command_runner,
	replace_func: ReplaceFunction = os.replace,
) -> dict[str, bool]:
	"""Transactionally remove only the canonical tagged live and durable rule."""
	snapshot = _read_snapshot(persistent_path)
	persistent_after, persistent_changed = rollback_http_rule(snapshot.text)
	try:
		live_policy = _filter_policy(save_runner())
	except (OSError, subprocess.SubprocessError, RuntimeError) as error:
		raise FirewallError(f"cannot inspect live firewall: {error}") from error
	live_changed = any(
		_is_tagged_http(_tokens(live_policy.lines[index]))
		for index in range(live_policy.start + 1, live_policy.end)
	)
	if not persistent_changed and not live_changed:
		return {"persistent_changed": False, "live_changed": False}
	if persistent_changed:
		_backup(persistent_path, snapshot, backup_dir)
		try:
			_atomic_write(persistent_path, persistent_after.encode("utf-8"), snapshot, replace_func)
		except OSError as error:
			restore_error = _restore_if_changed(persistent_path, snapshot, replace_func)
			detail = f"; persistent restore failed: {restore_error}" if restore_error else ""
			raise FirewallError(f"cannot replace persistent rules{detail}: {error}") from error
	if live_changed:
		try:
			command_runner(_delete_command())
		except (OSError, subprocess.SubprocessError, RuntimeError) as error:
			restore_error = _restore_if_changed(persistent_path, snapshot, replace_func) if persistent_changed else None
			detail = f"; persistent restore failed: {restore_error}" if restore_error else ""
			raise FirewallError(f"cannot remove live HTTP rule{detail}: {error}") from error
	return {"persistent_changed": persistent_changed, "live_changed": live_changed}


def _parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("action", choices=("ensure-http", "rollback-http"))
	return parser


def main(argv: Sequence[str] | None = None) -> int:
	args = _parser().parse_args(argv)
	if os.geteuid() != 0:
		print("zetin-web-firewall must run as root", file=sys.stderr)
		return 1
	try:
		if args.action == "ensure-http":
			ensure_http()
		else:
			rollback_http()
	except FirewallError as error:
		print(f"zetin-web-firewall: {error}", file=sys.stderr)
		return 1
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
