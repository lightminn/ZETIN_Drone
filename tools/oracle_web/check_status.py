"""Report one Oracle web site's fixed deployment and reachability probes as JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable, Sequence

from .common import validate_release_id
from .deploy_release import DeployError, ROOT_HELPER, SSH, SUDO, _bounded_subprocess, validate_target
from .site_manifest import ManifestError, SiteManifest, load_site_manifest


CURL = "/usr/bin/curl"
SYSTEMCTL = "/usr/bin/systemctl"
PROBE_TIMEOUT = 10
MAX_CAPTURE_BYTES = 64 * 1024
Runner = Callable[..., subprocess.CompletedProcess[bytes]]


def _default_runner(command: Sequence[str], *, timeout: int) -> subprocess.CompletedProcess[bytes]:
	return _bounded_subprocess(command, timeout=timeout, capture_limit=MAX_CAPTURE_BYTES)


def _execute(runner: Runner, command: Sequence[str]) -> dict[str, Any]:
	try:
		completed = runner(command, timeout=PROBE_TIMEOUT)
	except subprocess.TimeoutExpired:
		return {"kind": "timeout"}
	except OSError:
		return {"kind": "unavailable"}
	stdout = completed.stdout if isinstance(completed.stdout, bytes) else str(completed.stdout).encode()
	stderr = completed.stderr if isinstance(completed.stderr, bytes) else str(completed.stderr).encode()
	return {
		"kind": "completed",
		"returncode": completed.returncode,
		"stdout": stdout[: MAX_CAPTURE_BYTES + 1],
		"overflow": len(stdout) > MAX_CAPTURE_BYTES or len(stderr) > MAX_CAPTURE_BYTES,
	}


def _service_state(execution: dict[str, Any]) -> dict[str, object]:
	if execution["kind"] != "completed" or execution.get("returncode") == 255:
		return {"state": "unavailable"}
	if execution["returncode"] == 0:
		return {"state": "active"}
	return {"state": "inactive", "returncode": execution["returncode"]}


def _http_state(execution: dict[str, Any]) -> dict[str, object]:
	if execution["kind"] != "completed" or execution.get("returncode") == 255:
		return {"state": "unavailable"}
	if execution["returncode"] == 0:
		return {"state": "ok"}
	return {"state": "failed", "returncode": execution["returncode"]}


def _negative_state(execution: dict[str, Any]) -> dict[str, object]:
	if execution["kind"] == "unavailable":
		return {"state": "unavailable"}
	if execution["kind"] == "timeout":
		return {"state": "closed_or_filtered"}
	if execution["returncode"] == 0:
		return {"state": "open"}
	return {"state": "closed_or_filtered"}


def _current_state(execution: dict[str, Any]) -> dict[str, object]:
	if execution["kind"] != "completed" or execution.get("returncode") != 0:
		return {"state": "unavailable"}
	if execution["overflow"]:
		return {"state": "malformed"}
	try:
		value = json.loads(execution["stdout"])
	except (UnicodeDecodeError, json.JSONDecodeError):
		return {"state": "malformed"}
	if not isinstance(value, dict) or set(value) != {"current"}:
		return {"state": "malformed"}
	current = value["current"]
	if current is None:
		return {"state": "ok", "value": None}
	try:
		current = validate_release_id(current)
	except (TypeError, ValueError):
		return {"state": "malformed"}
	return {"state": "ok", "value": current}


def _ssh(target: str, *remote: str) -> list[str]:
	return [SSH, "--", target, *remote]


def _curl_base() -> list[str]:
	return [
		CURL, "--fail", "--silent", "--show-error", "--output", "/dev/null",
		"--noproxy", "*", "--max-time", "5",
	]


def _status_commands(target: str, manifest: SiteManifest) -> dict[str, Any]:
	domain = manifest.server_name
	public_ip = manifest.public_ipv4
	commands: dict[str, Any] = {
		"current": _ssh(target, SUDO, "-n", ROOT_HELPER, "status", "--site", manifest.site),
		"nginx": _ssh(target, SYSTEMCTL, "is-active", "nginx.service"),
		"remote_https": [
			_ssh(
				target,
				*_curl_base(),
				"--resolve",
				f"{domain}:443:127.0.0.1",
				f"https://{domain}{path}",
			)
			for path in manifest.https_health_paths
		],
		"local_https": [
			[
				*_curl_base(),
				"--resolve",
				f"{domain}:443:{public_ip}",
				f"https://{domain}{path}",
			]
			for path in manifest.https_health_paths
		],
		"negative": {
			"8000": [
				CURL, "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*",
				"--connect-timeout", "2", "--max-time", "3", f"http://{public_ip}:8000/",
			],
			"8443": [
				CURL, "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*",
				"--connect-timeout", "2", "--max-time", "3", "--insecure",
				f"https://{public_ip}:8443/",
			],
		},
	}
	if manifest.backend is not None:
		commands["backend"] = _ssh(
			target, SYSTEMCTL, "is-active", f"zetin-webapp@{manifest.site}.service",
		)
		commands["loopback_api"] = _ssh(
			target,
			*_curl_base(),
			f"http://127.0.0.1:{manifest.backend.port}{manifest.backend.health_path}",
		)
	return commands


def collect_status(target: str, manifest: SiteManifest, *, runner: Runner) -> dict[str, object]:
	commands = _status_commands(target, manifest)
	result: dict[str, object] = {
		"schema_version": 1,
		"site": manifest.site,
		"current_release": _current_state(_execute(runner, commands["current"])),
		"nginx": _service_state(_execute(runner, commands["nginx"])),
	}
	if manifest.backend is None:
		result["backend"] = {"state": "not_applicable"}
		result["loopback_api"] = {"state": "not_applicable"}
	else:
		result["backend"] = _service_state(_execute(runner, commands["backend"]))
		result["loopback_api"] = _http_state(_execute(runner, commands["loopback_api"]))
	result["remote_local_sni_https"] = [
		{"path": path, **_http_state(_execute(runner, command))}
		for path, command in zip(manifest.https_health_paths, commands["remote_https"])
	]
	result["local_public_ip_https"] = [
		{"path": path, **_http_state(_execute(runner, command))}
		for path, command in zip(manifest.https_health_paths, commands["local_https"])
	]
	result["negative_ports"] = {
		port: _negative_state(_execute(runner, command))
		for port, command in commands["negative"].items()
	}
	return result


def _arguments() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--target", required=True)
	parser.add_argument("--site-config", required=True, type=Path)
	return parser


def main(argv: Sequence[str] | None = None, *, runner: Runner | None = None) -> int:
	arguments = _arguments().parse_args(argv)
	try:
		target = validate_target(arguments.target)
		manifest = load_site_manifest(arguments.site_config)
	except (DeployError, ManifestError, OSError, ValueError):
		print(json.dumps({"error": "invalid status configuration"}), file=sys.stderr)
		return 1
	result = collect_status(target, manifest, runner=runner or _default_runner)
	print(json.dumps(result, sort_keys=True))
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
