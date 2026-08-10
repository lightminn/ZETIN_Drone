"""Contract tests for the local Oracle SSH deployment and status wrappers."""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tarfile
import tempfile
import unittest


DEPLOY_MODULE = "tools.oracle_web.deploy_release"
STATUS_MODULE = "tools.oracle_web.check_status"


class ScriptedRunner:
	"""Return scripted subprocess results while recording every real argv boundary."""

	def __init__(self, outcomes=(), on_call=None) -> None:
		self.outcomes = list(outcomes)
		self.on_call = on_call
		self.commands: list[tuple[str, ...]] = []
		self.timeouts: list[int] = []

	def __call__(self, command, *, timeout: int):
		recorded = tuple(command)
		self.commands.append(recorded)
		self.timeouts.append(timeout)
		if self.on_call is not None:
			self.on_call(recorded, len(self.commands))
		outcome = self.outcomes.pop(0) if self.outcomes else (0, b"", b"")
		if isinstance(outcome, BaseException):
			raise outcome
		return subprocess.CompletedProcess(recorded, outcome[0], outcome[1], outcome[2])


class OracleWebDeployTests(unittest.TestCase):
	def setUp(self) -> None:
		self.tempdir = tempfile.TemporaryDirectory(prefix="oracle-web-deploy-")
		self.addCleanup(self.tempdir.cleanup)
		self.root = Path(self.tempdir.name)
		self.snapshots = self.root / "snapshots"
		self.snapshots.mkdir()

	def _deploy_module(self):
		try:
			return importlib.import_module(DEPLOY_MODULE)
		except ModuleNotFoundError:
			self.fail(f"{DEPLOY_MODULE} is not implemented")

	def _status_module(self):
		try:
			return importlib.import_module(STATUS_MODULE)
		except ModuleNotFoundError:
			self.fail(f"{STATUS_MODULE} is not implemented")

	def _release(self, release_id: str, *, site: str = "mobile-lab") -> bytes:
		payload = {"public/index.html": b"<h1>mobile lab</h1>\n"}
		members = [
			{
				"path": path,
				"sha256": hashlib.sha256(content).hexdigest(),
				"size": len(content),
				"mode": 0o444,
			}
			for path, content in payload.items()
		]
		manifest = {
			"schema_version": 1,
			"site": site,
			"release_id": release_id,
			"source_commit": "a" * 40,
			"server_name": "uos-drone.kro.kr",
			"public_ipv4": "140.83.83.165",
			"https_health_paths": ["/", "/presenter.html"],
			"members": members,
		}
		stream = io.BytesIO()
		with tarfile.open(fileobj=stream, mode="w:gz") as archive:
			release_bytes = (json.dumps(manifest, sort_keys=True) + "\n").encode()
			metadata = tarfile.TarInfo("release.json")
			metadata.mode = 0o444
			metadata.size = len(release_bytes)
			archive.addfile(metadata, io.BytesIO(release_bytes))
			for path, content in payload.items():
				member = tarfile.TarInfo(path)
				member.mode = 0o444
				member.size = len(content)
				archive.addfile(member, io.BytesIO(content))
		return stream.getvalue()

	def _archive(self, release_id: str = "release-1", *, site: str = "mobile-lab") -> Path:
		path = self.root / f"{site}-{release_id}.tar.gz"
		path.write_bytes(self._release(release_id, site=site))
		return path

	def _deploy_main(self, arguments, runner: ScriptedRunner):
		stdout = io.StringIO()
		stderr = io.StringIO()
		with redirect_stdout(stdout), redirect_stderr(stderr):
			status = self._deploy_module().main(
				arguments,
				runner=runner,
				snapshot_parent=self.snapshots,
			)
		return status, stdout.getvalue(), stderr.getvalue()

	def test_deploy_dry_run_emits_exact_argv_order_without_execution(self) -> None:
		"""Changing command order or allowing the wrapper to execute defeats dry-run review."""
		archive = self._archive()
		digest = hashlib.sha256(archive.read_bytes()).hexdigest()
		runner = ScriptedRunner()

		status, output, error = self._deploy_main(
			[
				"deploy", "--target", "Oracle", "--site", "mobile-lab",
				"--release-id", "release-1", "--archive", str(archive), "--dry-run",
			],
			runner,
		)

		self.assertEqual(status, 0, error)
		self.assertEqual(runner.commands, [])
		commands = [json.loads(line) for line in output.splitlines()]
		self.assertEqual(len(commands), 4)
		snapshot = commands[1][-2]
		self.assertRegex(snapshot, r"/snapshots/zetin-web-deploy-[^/]+/mobile-lab-release-1\.tar\.gz\Z")
		self.assertEqual(
			commands,
			[
				[
					"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
					"Oracle", "/usr/bin/install", "-d", "-m", "0700",
					"/var/tmp/zetin-web-staging/mobile-lab",
				],
				[
					"/usr/bin/scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--", snapshot,
					"Oracle:/var/tmp/zetin-web-staging/mobile-lab/release-1.tar.gz",
				],
				[
					"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
					"Oracle", "/usr/bin/sudo", "-n",
					"/usr/local/sbin/zetin-web-release", "activate", "--site", "mobile-lab",
					"--release-id", "release-1", "--archive",
					"/var/tmp/zetin-web-staging/mobile-lab/release-1.tar.gz", "--sha256", digest,
				],
				[
					"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
					"Oracle", "/usr/bin/rm", "--",
					"/var/tmp/zetin-web-staging/mobile-lab/release-1.tar.gz", "&&",
					"/usr/bin/rmdir", "--ignore-fail-on-non-empty", "--",
					"/var/tmp/zetin-web-staging/mobile-lab",
				],
			],
		)
		self.assertEqual(list(self.snapshots.iterdir()), [], "dry-run snapshot must be removed")

	def test_every_ssh_and_scp_command_forces_noninteractive_connection_options(self) -> None:
		"""An auth prompt can stall a 50-person deployment while an inherited host-key policy stays required."""
		archive = self._archive()
		status, output, error = self._deploy_main(
			[
				"deploy", "--target", "Oracle", "--site", "mobile-lab",
				"--release-id", "release-1", "--archive", str(archive), "--dry-run",
			],
			ScriptedRunner(),
		)
		self.assertEqual(status, 0, error)
		for command in (json.loads(line) for line in output.splitlines()):
			self.assertIn(command[0], ("/usr/bin/ssh", "/usr/bin/scp"))
			self.assertIn(("-o", "BatchMode=yes"), tuple(zip(command, command[1:])))
			self.assertIn(("-o", "ConnectTimeout=10"), tuple(zip(command, command[1:])))
			self.assertNotIn("StrictHostKeyChecking=no", command)

	def test_deploy_rejects_invalid_identifiers_archive_types_and_metadata_mismatch(self) -> None:
		"""Relaxed values could become SSH options, remote paths, or activate the wrong release."""
		archive = self._archive()
		linked = self.root / "linked.tar.gz"
		linked.symlink_to(archive.name)
		cases = (
			("target", ["--target", "bad;target"]),
			("site", ["--site", "../mobile-lab"]),
			("release", ["--release-id", "../release"]),
			("relative archive", ["--archive", archive.name]),
			("linked archive", ["--archive", str(linked)]),
		)
		base = [
			"deploy", "--target", "Oracle", "--site", "mobile-lab",
			"--release-id", "release-1", "--archive", str(archive), "--dry-run",
		]
		for label, replacement in cases:
			with self.subTest(label=label):
				arguments = list(base)
				flag = replacement[0]
				arguments[arguments.index(flag) + 1] = replacement[1]
				runner = ScriptedRunner()
				status, output, _ = self._deploy_main(arguments, runner)
				self.assertNotEqual(status, 0)
				self.assertEqual(output, "")
				self.assertEqual(runner.commands, [])

		mismatched = self._archive("other")
		status, output, _ = self._deploy_main(
			[
				"deploy", "--target", "Oracle", "--site", "mobile-lab",
				"--release-id", "release-1", "--archive", str(mismatched), "--dry-run",
			],
			ScriptedRunner(),
		)
		self.assertNotEqual(status, 0)
		self.assertEqual(output, "")
		self.assertEqual(list(self.snapshots.iterdir()), [])

	def test_original_archive_replacement_cannot_change_uploaded_bytes_or_sha(self) -> None:
		"""Reopening the original after validation creates a metadata/hash/upload TOCTOU."""
		archive = self._archive()
		original = archive.read_bytes()
		digest = hashlib.sha256(original).hexdigest()
		uploaded: list[bytes] = []

		def mutate_and_capture(command: tuple[str, ...], call_number: int) -> None:
			if call_number == 1:
				replacement = self.root / "replacement"
				replacement.write_bytes(b"not the validated release")
				os.replace(replacement, archive)
			if command[0] == "/usr/bin/scp":
				uploaded.append(Path(command[-2]).read_bytes())

		activation = json.dumps(
			{"current": "release-1", "previous": None, "backend_restarted": True, "score_reset": True}
		).encode()
		runner = ScriptedRunner(
			[(0, b"", b""), (0, b"", b""), (0, activation, b""), (0, b"", b"")],
			on_call=mutate_and_capture,
		)
		status, output, error = self._deploy_main(
			[
				"deploy", "--target", "Oracle", "--site", "mobile-lab",
				"--release-id", "release-1", "--archive", str(archive),
			],
			runner,
		)

		self.assertEqual(status, 0, error)
		self.assertEqual(uploaded, [original])
		self.assertIn(digest, runner.commands[2])
		self.assertEqual(json.loads(output)["score_reset"], True)
		self.assertEqual(list(self.snapshots.iterdir()), [], "snapshot must be removed after success")

	def test_each_deploy_failure_stops_later_steps_and_preserves_exit_code(self) -> None:
		"""Continuing after staging, upload, or activation failure can activate unverified state."""
		archive = self._archive()
		activation = json.dumps(
			{"current": "release-1", "previous": None, "backend_restarted": True, "score_reset": True}
		).encode()
		for failure_index, expected_calls in ((0, 1), (1, 2), (2, 3), (3, 4)):
			with self.subTest(failure_index=failure_index):
				outcomes = [(0, b"", b""), (0, b"", b""), (0, activation, b""), (0, b"", b"")]
				outcomes[failure_index] = (31 + failure_index, b"sensitive body", b"/private/key")
				runner = ScriptedRunner(outcomes)
				status, output, error = self._deploy_main(
					[
						"deploy", "--target", "Oracle", "--site", "mobile-lab",
						"--release-id", "release-1", "--archive", str(archive),
					],
					runner,
				)
				self.assertEqual(status, 31 + failure_index)
				self.assertEqual(len(runner.commands), expected_calls)
				self.assertEqual(output, "")
				self.assertNotIn("sensitive body", error)
				self.assertNotIn("/private/key", error)
				self.assertEqual(list(self.snapshots.iterdir()), [])

	def test_activation_json_is_strict_and_only_safe_fields_are_reported(self) -> None:
		"""Passing through helper output can disclose bodies while malformed state masks failure."""
		archive = self._archive()
		valid = {
			"current": "release-1", "previous": "release-0",
			"backend_restarted": False, "score_reset": True,
		}
		for label, payload in (
			("malformed", b"not-json"),
			("unknown", json.dumps({**valid, "body": "secret nickname"}).encode()),
			("wrong type", json.dumps({**valid, "score_reset": "yes"}).encode()),
		):
			with self.subTest(label=label):
				runner = ScriptedRunner([(0, b"", b""), (0, b"", b""), (0, payload, b"")])
				status, output, error = self._deploy_main(
					[
						"deploy", "--target", "Oracle", "--site", "mobile-lab",
						"--release-id", "release-1", "--archive", str(archive),
					],
					runner,
				)
				self.assertNotEqual(status, 0)
				self.assertEqual(len(runner.commands), 3, "cleanup requires a valid successful activation")
				self.assertEqual(output, "")
				self.assertNotIn("secret nickname", error)

	def test_nonempty_site_staging_directory_does_not_hide_successful_activation(self) -> None:
		"""A stale failed archive may keep the site directory nonempty, but archive rm remains mandatory."""
		archive = self._archive()
		activation = json.dumps(
			{"current": "release-1", "previous": None, "backend_restarted": True, "score_reset": True}
		).encode()

		class StaleArtifactRunner(ScriptedRunner):
			def __call__(self, command, *, timeout: int):
				recorded = tuple(command)
				self.commands.append(recorded)
				self.timeouts.append(timeout)
				if len(self.commands) == 3:
					return subprocess.CompletedProcess(recorded, 0, activation, b"")
				if len(self.commands) == 4 and "--ignore-fail-on-non-empty" not in recorded:
					return subprocess.CompletedProcess(recorded, 39, b"", b"stale artifact")
				return subprocess.CompletedProcess(recorded, 0, b"", b"")

		runner = StaleArtifactRunner()
		status, output, error = self._deploy_main(
			[
				"deploy", "--target", "Oracle", "--site", "mobile-lab",
				"--release-id", "release-1", "--archive", str(archive),
			],
			runner,
		)
		self.assertEqual(status, 0, error)
		self.assertTrue(json.loads(output)["score_reset"])
		self.assertEqual(
			runner.commands[3][-4:],
			("/usr/bin/rmdir", "--ignore-fail-on-non-empty", "--", "/var/tmp/zetin-web-staging/mobile-lab"),
		)

	def test_rollback_invokes_only_fixed_helper_and_surfaces_score_reset(self) -> None:
		"""A wrapper-side service command or arbitrary rollback text broadens deploy authority."""
		payload = json.dumps(
			{"current": "release-0", "previous": "release-1", "backend_restarted": True, "score_reset": True}
		).encode()
		runner = ScriptedRunner([(0, payload, b"")])
		status, output, error = self._deploy_main(
			[
				"rollback", "--target", "Oracle", "--site", "mobile-lab",
				"--release-id", "release-0",
			],
			runner,
		)
		self.assertEqual(status, 0, error)
		self.assertEqual(
			runner.commands,
			[(
				"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
				"Oracle", "/usr/bin/sudo", "-n",
				"/usr/local/sbin/zetin-web-release", "rollback", "--site", "mobile-lab",
				"--release-id", "release-0",
			)],
		)
		self.assertEqual(json.loads(output), json.loads(payload))

	def test_default_runner_uses_argv_without_a_local_shell(self) -> None:
		"""Enabling a local shell turns otherwise validated argv into an injection boundary."""
		module = self._deploy_module()
		marker = self.root / "shell-expanded"
		literal = f"$(/usr/bin/touch {marker})"
		completed = module._default_runner(("/usr/bin/printf", "%s", literal), timeout=17)
		self.assertEqual(completed.returncode, 0)
		self.assertEqual(completed.stdout, literal.encode())
		self.assertFalse(marker.exists())

	def test_bounded_runner_uses_devnull_new_session_and_kills_descendant_pipe_holders(self) -> None:
		"""Killing only the direct child leaves inherited pipes open and can hang forever after timeout."""
		module = self._deploy_module()
		identity = module._bounded_subprocess(
			(
				sys.executable,
				"-c",
				"import os; print(os.readlink('/proc/self/fd/0')); print(os.getpid() == os.getpgrp())",
			),
			timeout=2,
			capture_limit=4096,
		)
		self.assertEqual(identity.stdout.splitlines(), [b"/dev/null", b"True"])

		script = (
			"import subprocess,sys,time; "
			"child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(2)']); "
			"print(child.pid, flush=True); time.sleep(2.5)"
		)
		started = __import__("time").monotonic()
		with self.assertRaises(subprocess.TimeoutExpired) as caught:
			module._bounded_subprocess(
				(sys.executable, "-c", script), timeout=0.15, capture_limit=4096,
			)
		elapsed = __import__("time").monotonic() - started
		self.assertLess(elapsed, 1.0, "descendant-held output pipes must not delay timeout return")
		child_pid = int(caught.exception.output.splitlines()[0])
		state_path = Path(f"/proc/{child_pid}/stat")
		if state_path.exists():
			try:
				state = state_path.read_text().split()[2]
			except FileNotFoundError:
				state = "gone"
			self.assertIn(state, ("Z", "gone"), "descendant must be killed with its group")

	def test_bounded_runner_keeps_timeout_when_child_closes_both_pipes_early(self) -> None:
		"""EOF on both capture pipes is not process completion and must not bypass the command deadline."""
		module = self._deploy_module()
		script = "import os,time; os.close(1); os.close(2); time.sleep(2)"
		started = __import__("time").monotonic()
		with self.assertRaises(subprocess.TimeoutExpired):
			module._bounded_subprocess(
				(sys.executable, "-c", script), timeout=0.15, capture_limit=4096,
			)
		self.assertLess(__import__("time").monotonic() - started, 1.0)


class OracleWebStatusTests(unittest.TestCase):
	def setUp(self) -> None:
		self.tempdir = tempfile.TemporaryDirectory(prefix="oracle-web-status-")
		self.addCleanup(self.tempdir.cleanup)
		self.root = Path(self.tempdir.name)

	def _module(self):
		try:
			return importlib.import_module(STATUS_MODULE)
		except ModuleNotFoundError:
			self.fail(f"{STATUS_MODULE} is not implemented")

	def _manifest(self, *, backend: bool) -> Path:
		value = {
			"schema_version": 1,
			"site": "mobile-lab",
			"server_name": "uos-drone.kro.kr",
			"public_ipv4": "140.83.83.165",
			"https_health_paths": ["/", "/presenter.html"],
			"files": [{"source": "fixture/index.html", "destination": "public/index.html"}],
		}
		if backend:
			value["backend"] = {"port": 18080, "health_path": "/api/scores"}
		path = self.root / ("api.json" if backend else "static.json")
		path.write_text(json.dumps(value) + "\n", encoding="utf-8")
		return path

	def _main(self, manifest: Path, runner: ScriptedRunner, *, tcp_connector=None):
		stdout = io.StringIO()
		stderr = io.StringIO()
		with redirect_stdout(stdout), redirect_stderr(stderr):
			if tcp_connector is None:
				def tcp_connector(_address, _timeout):
					raise ConnectionRefusedError()
			kwargs = {"runner": runner, "tcp_connector": tcp_connector}
			status = self._module().main(
				["--target", "Oracle", "--site-config", str(manifest)], **kwargs,
			)
		return status, stdout.getvalue(), stderr.getvalue()

	def _successes(self, count: int, *, current: str = "release-1"):
		return [(0, json.dumps({"current": current}).encode(), b"")] + [(0, b"", b"")] * (count - 1)

	def test_static_status_uses_fixed_topology_and_skips_backend_probes(self) -> None:
		"""Treating a static site as API-backed creates false failures and extra remote authority."""
		runner = ScriptedRunner(self._successes(8))
		status, output, error = self._main(self._manifest(backend=False), runner)
		self.assertEqual(status, 0, error)
		result = json.loads(output)
		self.assertEqual(result["current_release"], {"state": "ok", "value": "release-1"})
		self.assertEqual(result["backend"], {"state": "not_applicable"})
		self.assertEqual(result["loopback_api"], {"state": "not_applicable"})
		self.assertEqual(len(runner.commands), 6)
		joined = "\n".join(" ".join(command) for command in runner.commands)
		self.assertNotIn("zetin-webapp@", joined)
		self.assertNotIn("18080", joined)
		self.assertEqual(
			runner.commands[0],
			(
				"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
				"Oracle", "/usr/bin/sudo", "-n",
				"/usr/local/sbin/zetin-web-release", "status", "--site", "mobile-lab",
			),
		)
		self.assertEqual(
			runner.commands[1],
			(
				"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
				"Oracle", "/usr/bin/systemctl", "is-active", "nginx.service",
			),
		)
		self.assertEqual(
			runner.commands[2],
			(
				"/usr/bin/ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "--",
				"Oracle", "/usr/bin/curl", "--fail", "--silent", "--show-error", "--output",
				"/dev/null", "--noproxy", "'*'", "--max-time", "5", "--resolve",
				"uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/",
			),
		)
		self.assertEqual(
			runner.commands[4],
			(
				"/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null",
				"--noproxy", "*", "--max-time", "5", "--resolve",
				"uos-drone.kro.kr:443:140.83.83.165",
				"https://uos-drone.kro.kr/",
			),
		)
		for command in runner.commands[2:]:
			self.assertTrue(
				("--noproxy", "*") in tuple(zip(command, command[1:]))
				or ("--noproxy", "'*'") in tuple(zip(command, command[1:])),
			)

	def test_api_status_reports_every_probe_state_without_response_data(self) -> None:
		"""Collapsing probe states or echoing output hides failures and can expose score bodies."""
		secret = b'nickname="private-student" /etc/zetin-web/tls/site/privkey.pem'
		outcomes = [
			(0, b'{"current":"release-1"}', b""),
			(3, secret, secret),
			subprocess.TimeoutExpired(["systemctl"], 5, output=secret, stderr=secret),
			(22, secret, secret),
			(0, secret, secret),
			(7, secret, secret),
			(0, secret, secret),
			(0, secret, secret),
			(7, secret, secret),
			(0, secret, secret),
		]
		runner = ScriptedRunner(outcomes)

		class Connected:
			def close(self) -> None:
				pass

		def connector(address, _timeout):
			if address[1] == 8443:
				return Connected()
			raise ConnectionRefusedError()

		status, output, error = self._main(
			self._manifest(backend=True), runner, tcp_connector=connector,
		)
		self.assertEqual(status, 0, error)
		result = json.loads(output)
		self.assertEqual(result["nginx"]["state"], "inactive")
		self.assertEqual(result["backend"]["state"], "unavailable")
		self.assertEqual(result["loopback_api"]["state"], "failed")
		self.assertEqual([item["state"] for item in result["remote_local_sni_https"]], ["ok", "failed"])
		self.assertEqual([item["state"] for item in result["local_public_ip_https"]], ["ok", "ok"])
		self.assertEqual(result["negative_ports"]["8000"]["state"], "closed")
		self.assertEqual(result["negative_ports"]["8443"]["state"], "open")
		self.assertNotIn("private-student", output)
		self.assertNotIn("privkey.pem", output)
		self.assertNotIn("private-student", error)
		self.assertEqual(len(runner.commands), 8)
		joined = "\n".join(" ".join(command) for command in runner.commands)
		self.assertIn("zetin-webapp@mobile-lab.service", joined)
		self.assertIn("http://127.0.0.1:18080/api/scores", joined)

	def test_remote_manifest_tokens_remain_inert_after_openssh_argv_join(self) -> None:
		"""OpenSSH joins remote argv through a shell, so manifest paths must survive as one literal token."""
		module = self._module()
		payloads = (
			"/; /usr/bin/id",
			"/$(/usr/bin/printf EXPANDED)",
			"/\n/usr/bin/id",
			"/single'and\"double",
		)
		for index, payload in enumerate(payloads):
			with self.subTest(payload=payload):
				value = {
					"schema_version": 1,
					"site": "mobile-lab",
					"server_name": "uos-drone.kro.kr",
					"public_ipv4": "140.83.83.165",
					"https_health_paths": [payload],
					"backend": {"port": 18080, "health_path": payload},
					"files": [{"source": "fixture", "destination": "public/index.html"}],
				}
				path = self.root / f"hostile-{index}.json"
				path.write_text(json.dumps(value) + "\n", encoding="utf-8")
				manifest = module.load_site_manifest(path)
				commands = module._status_commands("Oracle", manifest)
				for command, expected in (
					(commands["remote_https"][0], f"https://uos-drone.kro.kr{payload}"),
					(commands["loopback_api"], f"http://127.0.0.1:18080{payload}"),
				):
					target_index = command.index("Oracle")
					joined = " ".join(command[target_index + 1 :])
					modeled = subprocess.run(
						[
							"/bin/sh", "-c",
							"set -- " + joined + "; /usr/bin/printf '%s\\0' \"$@\"",
						],
						capture_output=True,
						check=False,
					)
					self.assertEqual(modeled.returncode, 0, modeled.stderr.decode(errors="replace"))
					arguments = modeled.stdout.rstrip(b"\0").split(b"\0")
					self.assertIn(expected.encode(), arguments)
					self.assertNotIn(b"uid=", modeled.stdout)

	def test_negative_ports_use_tcp_connect_states_not_http_exit_codes(self) -> None:
		"""HTTP/TLS handshake errors and open sockets that do not reply must never be labeled closed."""
		calls: list[tuple[tuple[str, int], float]] = []

		class Connected:
			def close(self) -> None:
				pass

		def connector(address, timeout):
			calls.append((address, timeout))
			if address[1] == 8000:
				return Connected()
			raise ConnectionRefusedError()

		runner = ScriptedRunner(self._successes(6))
		status, output, error = self._main(
			self._manifest(backend=False), runner, tcp_connector=connector,
		)
		self.assertEqual(status, 0, error)
		result = json.loads(output)
		self.assertEqual(result["negative_ports"]["8000"]["state"], "open")
		self.assertEqual(result["negative_ports"]["8443"]["state"], "closed")
		self.assertEqual(calls, [(('140.83.83.165', 8000), 3.0), (('140.83.83.165', 8443), 3.0)])
		joined = "\n".join(" ".join(command) for command in runner.commands)
		self.assertNotIn(":8000", joined)
		self.assertNotIn(":8443", joined)

		for exception in (socket.timeout(), OSError("route unavailable")):
			with self.subTest(exception=type(exception).__name__):
				def unavailable(_address, _timeout, error=exception):
					raise error
				state = self._module()._tcp_connect_state(
					"140.83.83.165", 8000, connector=unavailable,
				)
				self.assertEqual(state, {"state": "indeterminate"})

	def test_status_helper_timeout_nonzero_malformed_and_oversize_are_calm(self) -> None:
		"""Untrusted helper output must not crash, grow output, or leak response content."""
		module = self._module()
		cases = (
			("timeout", subprocess.TimeoutExpired(["ssh"], 5, output=b"secret", stderr=b"secret"), "unavailable"),
			("nonzero", (255, b"secret", b"secret"), "unavailable"),
			("malformed", (0, b"not-json", b""), "malformed"),
			("oversize", (0, b"x" * (module.MAX_CAPTURE_BYTES + 1), b""), "malformed"),
		)
		for label, first, expected in cases:
			with self.subTest(label=label):
				runner = ScriptedRunner([first] + [(0, b"", b"")] * 7)
				status, output, error = self._main(self._manifest(backend=False), runner)
				self.assertEqual(status, 0, error)
				self.assertEqual(json.loads(output)["current_release"]["state"], expected)
				self.assertNotIn("secret", output)
				self.assertLess(len(output), 4096)

	def test_default_status_runner_bounds_stdout_and_stderr_capture(self) -> None:
		"""A remote probe that floods either stream must not create unbounded local memory output."""
		module = self._module()
		size = module.MAX_CAPTURE_BYTES * 2
		completed = module._default_runner(
			(
				sys.executable,
				"-c",
				"import sys; n=int(sys.argv[1]); "
				"sys.stdout.buffer.write(b'x'*n); sys.stderr.buffer.write(b'y'*n)",
				str(size),
			),
			timeout=5,
		)
		self.assertEqual(completed.returncode, 0)
		self.assertLessEqual(len(completed.stdout), module.MAX_CAPTURE_BYTES + 1)
		self.assertLessEqual(len(completed.stderr), module.MAX_CAPTURE_BYTES + 1)

	def test_status_rejects_invalid_target_or_manifest_before_any_probe(self) -> None:
		"""Invalid topology values must never be converted into SSH or URL arguments."""
		for target, manifest in (
			("bad;target", self._manifest(backend=False)),
			("Oracle", self.root / "missing.json"),
		):
			with self.subTest(target=target, manifest=manifest.name):
				runner = ScriptedRunner()
				stdout = io.StringIO()
				stderr = io.StringIO()
				with redirect_stdout(stdout), redirect_stderr(stderr):
					status = self._module().main(
						["--target", target, "--site-config", str(manifest)], runner=runner,
					)
				self.assertNotEqual(status, 0)
				self.assertEqual(stdout.getvalue(), "")
				self.assertEqual(runner.commands, [])

		unsafe = json.loads(self._manifest(backend=False).read_text(encoding="utf-8"))
		unsafe["https_health_paths"] = ["/nul\0path"]
		unsafe_path = self.root / "nul-path.json"
		unsafe_path.write_text(json.dumps(unsafe) + "\n", encoding="utf-8")
		runner = ScriptedRunner()
		stdout = io.StringIO()
		stderr = io.StringIO()
		with redirect_stdout(stdout), redirect_stderr(stderr):
			status = self._module().main(
				["--target", "Oracle", "--site-config", str(unsafe_path)],
				runner=runner,
				tcp_connector=lambda _address, _timeout: None,
			)
		self.assertNotEqual(status, 0)
		self.assertEqual(stdout.getvalue(), "")
		self.assertEqual(runner.commands, [])


if __name__ == "__main__":
	unittest.main()
