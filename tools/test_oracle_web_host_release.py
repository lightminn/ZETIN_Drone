"""Filesystem contract tests for the root-side Oracle release helper."""

from __future__ import annotations

import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock


class RecordingRunner:
	"""Record unavoidable host commands and optionally fail selected calls."""

	def __init__(self, failure=None) -> None:
		self.commands: list[tuple[str, ...]] = []
		self.failure = failure

	def __call__(self, command) -> None:
		recorded = tuple(command)
		self.commands.append(recorded)
		if self.failure is not None:
			message = self.failure(recorded, len(self.commands))
			if message is not None:
				raise RuntimeError(message)


class OracleWebHostReleaseTests(unittest.TestCase):
	def setUp(self) -> None:
		self.tempdir = tempfile.TemporaryDirectory(prefix="oracle-web-host-release-")
		self.addCleanup(self.tempdir.cleanup)
		root = Path(self.tempdir.name)
		self.app_root = root / "srv/zetin-web/apps"
		self.staging_root = root / "var/tmp/zetin-web-staging"
		self.runner = RecordingRunner()

	def _module(self):
		try:
			return importlib.import_module("tools.oracle_web.host_release")
		except ModuleNotFoundError:
			self.fail("tools.oracle_web.host_release is not implemented")

	def _release(self, release_id: str, files: dict[str, bytes] | None = None) -> tuple[dict[str, object], dict[str, bytes]]:
		payload = files or {
			"public/index.html": b"<h1>mobile lab</h1>\n",
			"backend/server.py": b"print('server')\n",
			"run": b"#!/bin/sh\nexec python3 backend/server.py\n",
		}
		members = []
		for path, content in payload.items():
			members.append({
				"path": path,
				"sha256": hashlib.sha256(content).hexdigest(),
				"size": len(content),
				"mode": 0o555 if path == "run" else 0o444,
			})
		manifest: dict[str, object] = {
			"schema_version": 1,
			"site": "mobile-lab",
			"release_id": release_id,
			"source_commit": "a" * 40,
			"server_name": "uos-drone.kro.kr",
			"public_ipv4": "140.83.83.165",
			"https_health_paths": ["/", "/presenter.html"],
			"backend": {"port": 18080, "health_path": "/api/scores"},
			"members": members,
		}
		return manifest, payload

	def _archive(
		self,
		release_id: str,
		*,
		manifest: dict[str, object] | None = None,
		files: dict[str, bytes] | None = None,
	) -> tuple[Path, str]:
		if manifest is None or files is None:
			default_manifest, default_files = self._release(release_id, files)
			manifest = default_manifest if manifest is None else manifest
			files = default_files if files is None else files
		archive_path = self.staging_root / "mobile-lab" / f"{release_id}.tar.gz"
		archive_path.parent.mkdir(parents=True, exist_ok=True)
		with tarfile.open(archive_path, "w:gz") as archive:
			release_bytes = (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode()
			metadata = tarfile.TarInfo("release.json")
			metadata.mode = 0o444
			metadata.size = len(release_bytes)
			archive.addfile(metadata, io.BytesIO(release_bytes))
			for path, content in files.items():
				member = tarfile.TarInfo(path)
				member.mode = 0o555 if path == "run" else 0o444
				member.size = len(content)
				archive.addfile(member, io.BytesIO(content))
		return archive_path, hashlib.sha256(archive_path.read_bytes()).hexdigest()

	def _raw_archive(
		self,
		release_id: str,
		manifest: dict[str, object] | None,
		entries: list[tuple[tarfile.TarInfo, bytes | None]],
	) -> tuple[Path, str]:
		archive_path = self.staging_root / "mobile-lab" / f"{release_id}.tar.gz"
		archive_path.parent.mkdir(parents=True, exist_ok=True)
		with tarfile.open(archive_path, "w:gz") as archive:
			if manifest is not None:
				release_bytes = (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode()
				metadata = tarfile.TarInfo("release.json")
				metadata.mode = 0o444
				metadata.size = len(release_bytes)
				archive.addfile(metadata, io.BytesIO(release_bytes))
			for member, content in entries:
				if content is not None:
					member.size = len(content)
				archive.addfile(member, None if content is None else io.BytesIO(content))
		return archive_path, hashlib.sha256(archive_path.read_bytes()).hexdigest()

	def _activate(self, release_id: str, *, runner: RecordingRunner | None = None, files=None):
		archive, digest = self._archive(release_id, files=files)
		return self._module().activate(
			"mobile-lab",
			release_id,
			archive,
			digest,
			app_root=self.app_root,
			staging_root=self.staging_root,
			runner=runner or self.runner,
		)

	def _current_target(self) -> str | None:
		current = self.app_root / "mobile-lab/current"
		return os.readlink(current) if current.is_symlink() else None

	def _assert_activation_rejected(
		self,
		release_id: str,
		archive: Path,
		digest: str,
		*,
		runner: RecordingRunner | None = None,
		site: str = "mobile-lab",
	) -> Exception:
		before = self._current_target()
		caught: Exception | None = None
		try:
			self._module().activate(
				site,
				release_id,
				archive,
				digest,
				app_root=self.app_root,
				staging_root=self.staging_root,
				runner=runner or self.runner,
			)
		except Exception as error:  # Contract assertion below checks the public error type.
			caught = error
			self.assertIsInstance(error, self._module().ReleaseError)
		else:
			self.fail("unsafe activation unexpectedly succeeded")
		self.assertEqual(self._current_target(), before)
		self.assertTrue(archive.exists(), "the helper must never remove a staged archive")
		releases = self.app_root / "mobile-lab/releases"
		if releases.exists():
			self.assertEqual(list(releases.glob(f".{release_id}.tmp.*")), [])
		self.assertIsNotNone(caught)
		return caught

	def test_activate_installs_valid_archive_and_switches_relative_current(self) -> None:
		"""Skipping verified extraction, modes, or the atomic relative link breaks activation."""
		archive, digest = self._archive("release-1")

		result = self._module().activate(
			"mobile-lab",
			"release-1",
			archive,
			digest,
			app_root=self.app_root,
			staging_root=self.staging_root,
			runner=self.runner,
		)

		release = self.app_root / "mobile-lab/releases/release-1"
		self.assertEqual((release / "public/index.html").read_bytes(), b"<h1>mobile lab</h1>\n")
		self.assertEqual((release / "backend/server.py").read_bytes(), b"print('server')\n")
		self.assertEqual(os.stat(release).st_mode & 0o777, 0o555)
		self.assertEqual(os.stat(release / "public").st_mode & 0o777, 0o555)
		self.assertEqual(os.stat(release / "public/index.html").st_mode & 0o777, 0o444)
		self.assertEqual(os.stat(release / "run").st_mode & 0o777, 0o555)
		current = self.app_root / "mobile-lab/current"
		self.assertTrue(current.is_symlink())
		self.assertEqual(os.readlink(current), "releases/release-1")
		self.assertEqual(
			result,
			{"current": "release-1", "previous": None, "backend_restarted": True, "score_reset": True},
		)
		self.assertEqual(
			self.runner.commands,
			[
				("/usr/sbin/nginx", "-t"),
				("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
				("/usr/bin/systemctl", "reload", "nginx"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "http://127.0.0.1:18080/api/scores"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/presenter.html"),
			],
		)
		self.assertTrue(archive.is_file(), "the host helper must leave staging cleanup to the deploy wrapper")

	def test_preflight_rejects_checksum_identifiers_and_staging_symlink_without_switch(self) -> None:
		"""Weak request validation could install bytes from an untrusted path or release name."""
		self._activate("old")
		self.runner.commands.clear()
		archive, digest = self._archive("new")
		self._assert_activation_rejected("new", archive, "0" * 64)

		for site, release_id in (("../mobile-lab", "new"), ("mobile-lab", "../new")):
			with self.subTest(site=site, release_id=release_id):
				self._assert_activation_rejected(release_id, archive, digest, site=site)

		real_archive, real_digest = self._archive("linked")
		moved = real_archive.with_suffix(".real")
		real_archive.rename(moved)
		os.symlink(moved.name, real_archive)
		self._assert_activation_rejected("linked", real_archive, real_digest)
		self.assertEqual(self.runner.commands, [])

	def test_preflight_rejects_unsafe_tar_types_and_paths_without_touching_other_trees(self) -> None:
		"""Accepting noncanonical or nonregular tar members permits extraction outside the release."""
		self._activate("old")
		self.runner.commands.clear()
		other_release = self.app_root / "mobile-lab/releases/keep-me/marker"
		other_release.parent.mkdir()
		other_release.write_text("keep\n")
		other_site = self.app_root / "other-site/releases/other/marker"
		other_site.parent.mkdir(parents=True)
		other_site.write_text("other\n")

		manifest, files = self._release("bad")
		base_entries = []
		for path, content in files.items():
			member = tarfile.TarInfo(path)
			member.mode = 0o555 if path == "run" else 0o444
			base_entries.append((member, content))
		absolute_escape = Path(self.tempdir.name) / "absolute-escape"
		cases: dict[str, tarfile.TarInfo] = {}
		absolute = tarfile.TarInfo(str(absolute_escape))
		absolute.mode = 0o444
		cases["absolute"] = absolute
		traversal = tarfile.TarInfo("../traversal-escape")
		traversal.mode = 0o444
		cases["traversal"] = traversal
		symlink = tarfile.TarInfo("public/link")
		symlink.type = tarfile.SYMTYPE
		symlink.linkname = "/etc/passwd"
		cases["symlink"] = symlink
		hardlink = tarfile.TarInfo("public/hardlink")
		hardlink.type = tarfile.LNKTYPE
		hardlink.linkname = "public/index.html"
		cases["hardlink"] = hardlink
		device = tarfile.TarInfo("public/device")
		device.type = tarfile.CHRTYPE
		device.devmajor = 1
		device.devminor = 3
		cases["device"] = device

		for label, hostile in cases.items():
			with self.subTest(label=label):
				archive, digest = self._raw_archive(
					"bad",
					manifest,
					[*base_entries, (hostile, b"escape" if hostile.isreg() else None)],
				)
				self._assert_activation_rejected("bad", archive, digest)
				self.assertFalse(absolute_escape.exists())
				self.assertFalse((self.app_root / "mobile-lab/releases/traversal-escape").exists())
				self.assertEqual(other_release.read_text(), "keep\n")
				self.assertEqual(other_site.read_text(), "other\n")
		self.assertEqual(self.runner.commands, [])

	def test_site_root_symlink_is_rejected_before_creating_release_directories(self) -> None:
		"""Following a hostile site symlink would let the root helper write outside the app root."""
		archive, digest = self._archive("new")
		outside = Path(self.tempdir.name) / "outside-site"
		outside.mkdir()
		self.app_root.mkdir(parents=True)
		os.symlink(outside, self.app_root / "mobile-lab")

		try:
			self._module().activate(
				"mobile-lab", "new", archive, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			)
		except Exception as error:
			self.assertIsInstance(error, self._module().ReleaseError)
		else:
			self.fail("site-root symlink unexpectedly accepted")

		self.assertEqual(list(outside.iterdir()), [])
		self.assertEqual(self.runner.commands, [])

	def test_preflight_rejects_missing_mismatched_or_additional_manifest_members(self) -> None:
		"""Failing to bind every regular file to release.json defeats the immutable allowlist."""
		self._activate("old")
		self.runner.commands.clear()
		manifest, files = self._release("bad-manifest")
		entries = []
		for path, content in files.items():
			member = tarfile.TarInfo(path)
			member.mode = 0o555 if path == "run" else 0o444
			entries.append((member, content))

		archive, digest = self._raw_archive("bad-manifest", None, entries)
		self._assert_activation_rejected("bad-manifest", archive, digest)

		mutations: list[tuple[str, dict[str, object], list[tuple[tarfile.TarInfo, bytes | None]]]] = []
		wrong_site = dict(manifest)
		wrong_site["site"] = "other-site"
		mutations.append(("site mismatch", wrong_site, entries))
		wrong_release = dict(manifest)
		wrong_release["release_id"] = "other-release"
		mutations.append(("release mismatch", wrong_release, entries))
		unknown_key = dict(manifest)
		unknown_key["unexpected"] = True
		mutations.append(("unknown manifest key", unknown_key, entries))
		missing_key = dict(manifest)
		del missing_key["server_name"]
		mutations.append(("missing manifest key", missing_key, entries))
		invalid_health = dict(manifest)
		invalid_health["https_health_paths"] = ["not/absolute"]
		mutations.append(("invalid health path", invalid_health, entries))
		mutations.append(("missing file", manifest, entries[:-1]))
		bad_content = list(entries)
		bad_content[0] = (bad_content[0][0], b"different bytes\n")
		mutations.append(("hash and size mismatch", manifest, bad_content))
		bad_mode = list(entries)
		mode_member = tarfile.TarInfo("public/index.html")
		mode_member.mode = 0o644
		bad_mode[0] = (mode_member, files["public/index.html"])
		mutations.append(("mode mismatch", manifest, bad_mode))
		extra = tarfile.TarInfo("public/extra.txt")
		extra.mode = 0o444
		mutations.append(("additional file", manifest, [*entries, (extra, b"extra\n")]))

		for label, changed_manifest, changed_entries in mutations:
			with self.subTest(label=label):
				archive, digest = self._raw_archive("bad-manifest", changed_manifest, changed_entries)
				self._assert_activation_rejected("bad-manifest", archive, digest)
		self.assertEqual(self.runner.commands, [])

	def test_same_current_release_reentry_reconciles_partial_runtime_and_stale_next(self) -> None:
		"""Returning early can bless a switched symlink whose backend was never restarted."""
		archive, digest = self._archive("same")
		self._module().activate(
			"mobile-lab", "same", archive, digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.runner.commands.clear()
		next_link = self.app_root / "mobile-lab/current.next"
		os.symlink("releases/same", next_link)

		class StaleBackendRunner(RecordingRunner):
			def __init__(self) -> None:
				super().__init__()
				self.runtime_restarted = False

			def __call__(self, command) -> None:
				recorded = tuple(command)
				self.commands.append(recorded)
				if recorded == ("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"):
					self.runtime_restarted = True
				if (
					recorded[-1] == "http://127.0.0.1:18080/api/scores"
					and not self.runtime_restarted
				):
					raise RuntimeError("stale backend still serves the old current target")

		reconciliation = StaleBackendRunner()

		result = self._module().activate(
			"mobile-lab", "same", archive, digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=reconciliation,
		)
		self.assertEqual(
			result,
			{"current": "same", "previous": "same", "backend_restarted": True, "score_reset": True},
		)
		self.assertFalse(next_link.exists())
		self.assertTrue(reconciliation.runtime_restarted)
		self.assertEqual(
			reconciliation.commands,
			[
				("/usr/sbin/nginx", "-t"),
				("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
				("/usr/bin/systemctl", "reload", "nginx"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "http://127.0.0.1:18080/api/scores"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/presenter.html"),
			],
		)

		installed = self.app_root / "mobile-lab/releases/same/public/index.html"
		os.chmod(installed, 0o644)
		installed.write_bytes(b"tampered\n")
		os.chmod(installed, 0o444)
		self._assert_activation_rejected("same", archive, digest)
		self.assertEqual(installed.read_bytes(), b"tampered\n")

	def test_nginx_preflight_failure_keeps_current_and_installed_release_for_diagnosis(self) -> None:
		"""Moving current before nginx -t would expose a release after a config failure."""
		self._activate("old")
		archive, digest = self._archive("new")
		failing = RecordingRunner(lambda command, _index: "nginx invalid" if command == ("/usr/sbin/nginx", "-t") else None)

		error = self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertIn("nginx invalid", str(error))
		self.assertTrue((self.app_root / "mobile-lab/releases/new/release.json").is_file())
		self.assertEqual(failing.commands, [("/usr/sbin/nginx", "-t")])

	def test_static_only_update_preserves_backend_process_and_reports_previous(self) -> None:
		"""Restarting an unchanged backend on a static update would erase the in-memory score table."""
		self._activate("old")
		self.runner.commands.clear()
		_, files = self._release("new")
		files["public/index.html"] = b"<h1>updated static page</h1>\n"

		result = self._activate("new", files=files)

		self.assertEqual(self._current_target(), "releases/new")
		self.assertEqual(
			result,
			{"current": "new", "previous": "old", "backend_restarted": False, "score_reset": False},
		)
		self.assertNotIn(
			("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
			self.runner.commands,
		)
		self.assertEqual(self.runner.commands[0], ("/usr/sbin/nginx", "-t"))
		self.assertEqual(self.runner.commands[1], ("/usr/bin/systemctl", "reload", "nginx"))

	def test_backend_or_run_change_restarts_backend_and_reports_score_reset(self) -> None:
		"""Missing a backend hash comparison could serve new static files from old application code."""
		self._activate("old")
		self.runner.commands.clear()
		_, files = self._release("new")
		files["backend/server.py"] = b"print('new server')\n"

		result = self._activate("new", files=files)

		self.assertTrue(result["backend_restarted"])
		self.assertTrue(result["score_reset"])
		self.assertEqual(
			self.runner.commands[:3],
			[
				("/usr/sbin/nginx", "-t"),
				("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
				("/usr/bin/systemctl", "reload", "nginx"),
			],
		)

		self.runner.commands.clear()
		run_only_files = dict(files)
		run_only_files["run"] = b"#!/bin/sh\nexec python3 -u backend/server.py\n"
		run_result = self._activate("run-change", files=run_only_files)
		self.assertTrue(run_result["backend_restarted"])
		self.assertTrue(run_result["score_reset"])
		self.assertIn(
			("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
			self.runner.commands,
		)

	def test_backend_restart_waits_for_delayed_type_simple_readiness(self) -> None:
		"""A type=simple restart may return before the backend accepts loopback requests."""
		module = self._module()

		class DelayedReadyRunner(RecordingRunner):
			def __init__(self) -> None:
				super().__init__()
				self.backend_attempts = 0

			def __call__(self, command) -> None:
				recorded = tuple(command)
				self.commands.append(recorded)
				if recorded[-1] == "http://127.0.0.1:18080/api/scores":
					self.backend_attempts += 1
					if self.backend_attempts < 3:
						raise RuntimeError("backend is still starting")

		runner = DelayedReadyRunner()
		archive, digest = self._archive("delayed-ready")
		with mock.patch("time.sleep") as sleep:
			try:
				result = module.activate(
					"mobile-lab", "delayed-ready", archive, digest,
					app_root=self.app_root, staging_root=self.staging_root, runner=runner,
				)
			except module.ReleaseError as error:
				self.fail(f"delayed backend readiness was not retried: {error}")

		self.assertEqual(result["current"], "delayed-ready")
		self.assertEqual(runner.backend_attempts, 3)
		self.assertEqual(sleep.call_count, 2)
		for command in runner.commands:
			if command[0] == "/usr/bin/curl":
				self.assertIn(("--max-time", "5"), tuple(zip(command, command[1:])))
				self.assertIn(("--output", "/dev/null"), tuple(zip(command, command[1:])))
				self.assertIn(("--noproxy", "*"), tuple(zip(command, command[1:])))

	def test_loopback_health_failure_restores_previous_release_and_runtime(self) -> None:
		"""A failed backend health check must perform a real symlink and runtime rollback."""
		self._activate("old")
		_, files = self._release("new")
		files["backend/server.py"] = b"raise SystemExit('broken')\n"
		archive, digest = self._archive("new", files=files)
		module = self._module()
		failing = RecordingRunner(
			lambda command, _index: "new loopback health failed"
			if (
				self._current_target() == "releases/new"
				and command[-1] == "http://127.0.0.1:18080/api/scores"
			) else None
		)

		with mock.patch.object(module, "BACKEND_READY_TIMEOUT_SECONDS", 0, create=True):
			error = self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertIn("new loopback health failed", str(error))
		self.assertEqual(self._current_target(), "releases/old")
		self.assertTrue((self.app_root / "mobile-lab/releases/new").is_dir())
		self.assertEqual(
			failing.commands,
			[
				("/usr/sbin/nginx", "-t"),
				("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
				("/usr/bin/systemctl", "reload", "nginx"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "http://127.0.0.1:18080/api/scores"),
				("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
				("/usr/bin/systemctl", "reload", "nginx"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "http://127.0.0.1:18080/api/scores"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/presenter.html"),
			],
		)

	def test_https_health_failure_restores_static_release_without_score_reset(self) -> None:
		"""Local-SNI health failure must restore current even when the backend stayed running."""
		self._activate("old")
		_, files = self._release("new")
		files["public/index.html"] = b"broken static response\n"
		archive, digest = self._archive("new", files=files)
		failing = RecordingRunner(
			lambda command, _index: "local SNI failed"
			if (
				self._current_target() == "releases/new"
				and command[-1] == "https://uos-drone.kro.kr/presenter.html"
			) else None
		)

		self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertEqual(self._current_target(), "releases/old")
		self.assertEqual(
			failing.commands[-1],
			("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/presenter.html"),
		)
		self.assertNotIn(
			("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
			failing.commands,
		)

	def test_rollback_command_failure_is_reported_with_original_health_error(self) -> None:
		"""Masking recovery failure would falsely imply that the old runtime is healthy again."""
		self._activate("old")
		_, files = self._release("new")
		files["run"] = b"#!/bin/sh\nexit 1\n"
		archive, digest = self._archive("new", files=files)

		def fail_new_and_recovery(command: tuple[str, ...], index: int) -> str | None:
			if (
				self._current_target() == "releases/new"
				and command[-1] == "http://127.0.0.1:18080/api/scores"
			):
				return "original health error"
			if index > 4 and command == ("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"):
				return "recovery restart error"
			return None

		failing = RecordingRunner(fail_new_and_recovery)
		with mock.patch.object(self._module(), "BACKEND_READY_TIMEOUT_SECONDS", 0, create=True):
			error = self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertEqual(self._current_target(), "releases/old")
		self.assertIn("original health error", str(error))
		self.assertIn("rollback failed", str(error))
		self.assertIn("recovery restart error", str(error))

	def test_recovery_rejects_previous_release_when_its_backend_dies(self) -> None:
		"""Restoring only the symlink must not be reported as a successful rollback."""
		self._activate("old")
		_, files = self._release("new")
		files["backend/server.py"] = b"print('new backend')\n"
		archive, digest = self._archive("new", files=files)
		new_https_failed = False

		def fail_new_then_previous(command: tuple[str, ...], _index: int) -> str | None:
			nonlocal new_https_failed
			if (
				self._current_target() == "releases/new"
				and command[-1] == "https://uos-drone.kro.kr/"
			):
				new_https_failed = True
				return "new release HTTPS failed"
			if (
				new_https_failed
				and self._current_target() == "releases/old"
				and command[-1] == "http://127.0.0.1:18080/api/scores"
			):
				return "previous backend is dead"
			return None

		module = self._module()
		failing = RecordingRunner(fail_new_then_previous)
		with mock.patch.object(module, "BACKEND_READY_TIMEOUT_SECONDS", 0, create=True):
			error = self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertEqual(self._current_target(), "releases/old")
		self.assertIn("new release HTTPS failed", str(error))
		self.assertIn("rollback failed", str(error))
		self.assertIn("previous backend is dead", str(error))
		self.assertNotIn("activation rolled back", str(error))

	def test_first_deploy_post_switch_failure_removes_only_current_link(self) -> None:
		"""With no previous release, recovery must unlink current but retain the immutable release."""
		archive, digest = self._archive("first")
		failing = RecordingRunner(
			lambda command, _index: "reload failed"
			if command == ("/usr/bin/systemctl", "reload", "nginx") else None
		)

		self._assert_activation_rejected("first", archive, digest, runner=failing)

		self.assertIsNone(self._current_target())
		self.assertTrue((self.app_root / "mobile-lab/releases/first").is_dir())

	def test_rollback_switches_only_to_a_validated_existing_release_without_deleting(self) -> None:
		"""Rollback must reuse an immutable release and keep both histories available."""
		self._activate("old")
		_, files = self._release("new")
		files["public/index.html"] = b"new static bytes\n"
		self._activate("new", files=files)
		self.runner.commands.clear()

		result = self._module().rollback(
			"mobile-lab",
			"old",
			app_root=self.app_root,
			runner=self.runner,
		)

		self.assertEqual(
			result,
			{"current": "old", "previous": "new", "backend_restarted": False, "score_reset": False},
		)
		self.assertEqual(self._current_target(), "releases/old")
		self.assertTrue((self.app_root / "mobile-lab/releases/old").is_dir())
		self.assertTrue((self.app_root / "mobile-lab/releases/new").is_dir())
		self.assertEqual(self.runner.commands[0], ("/usr/sbin/nginx", "-t"))
		self.assertEqual(self.runner.commands[1], ("/usr/bin/systemctl", "reload", "nginx"))

	def test_rollback_rejects_missing_or_tampered_target_before_switch(self) -> None:
		"""A release directory name alone is insufficient evidence that rollback is safe."""
		self._activate("old")
		_, files = self._release("new")
		files["public/index.html"] = b"new static bytes\n"
		self._activate("new", files=files)
		self.runner.commands.clear()

		for target in ("missing", "../old"):
			with self.subTest(target=target):
				try:
					self._module().rollback("mobile-lab", target, app_root=self.app_root, runner=self.runner)
				except Exception as error:
					self.assertIsInstance(error, self._module().ReleaseError)
				else:
					self.fail("unsafe rollback unexpectedly succeeded")
				self.assertEqual(self._current_target(), "releases/new")

		target_file = self.app_root / "mobile-lab/releases/old/backend/server.py"
		os.chmod(target_file, 0o644)
		target_file.write_bytes(b"tampered target\n")
		os.chmod(target_file, 0o444)
		with self.assertRaises(self._module().ReleaseError):
			self._module().rollback("mobile-lab", "old", app_root=self.app_root, runner=self.runner)
		self.assertEqual(self._current_target(), "releases/new")
		self.assertEqual(self.runner.commands, [])

	def test_rollback_health_failure_automatically_restores_release_being_left(self) -> None:
		"""An explicit rollback is still transactional when the older release fails health checks."""
		self._activate("old")
		_, files = self._release("new")
		files["public/index.html"] = b"new static bytes\n"
		self._activate("new", files=files)
		failing = RecordingRunner(
			lambda command, _index: "old release HTTPS failed"
			if (
				self._current_target() == "releases/old"
				and command[-1] == "https://uos-drone.kro.kr/"
			) else None
		)

		with self.assertRaises(self._module().ReleaseError) as raised:
			self._module().rollback("mobile-lab", "old", app_root=self.app_root, runner=failing)

		self.assertIn("old release HTTPS failed", str(raised.exception))
		self.assertEqual(self._current_target(), "releases/new")

	def test_status_is_read_only_and_validates_the_current_release(self) -> None:
		"""Status must not claim a corrupt target is the active validated release."""
		self.assertEqual(self._module().status("mobile-lab", app_root=self.app_root), {"current": None})
		self._activate("current")
		before = tuple(sorted(path.relative_to(self.app_root).as_posix() for path in self.app_root.rglob("*")))

		self.assertEqual(self._module().status("mobile-lab", app_root=self.app_root), {"current": "current"})
		after = tuple(sorted(path.relative_to(self.app_root).as_posix() for path in self.app_root.rglob("*")))
		self.assertEqual(after, before)

		target = self.app_root / "mobile-lab/releases/current/public/index.html"
		os.chmod(target, 0o644)
		with self.assertRaises(self._module().ReleaseError):
			self._module().status("mobile-lab", app_root=self.app_root)

	def test_cli_activate_prints_only_machine_readable_result_json(self) -> None:
		"""Changing CLI field names or adding prose would break the remote deploy wrapper contract."""
		archive, digest = self._archive("cli-release")
		output = io.StringIO()

		with redirect_stdout(output):
			exit_code = self._module().main(
				[
					"activate", "--site", "mobile-lab", "--release-id", "cli-release",
					"--archive", str(archive), "--sha256", digest,
				],
				app_root=self.app_root,
				staging_root=self.staging_root,
				runner=self.runner,
				require_root=False,
			)

		self.assertEqual(exit_code, 0)
		self.assertEqual(
			json.loads(output.getvalue()),
			{"current": "cli-release", "previous": None, "backend_restarted": True, "score_reset": True},
		)
		self.assertEqual(output.getvalue().count("\n"), 1)

	def test_real_health_children_cannot_contaminate_cli_stdout_with_response_bodies(self) -> None:
		"""Inherited curl stdout must not prepend API, static, or font bytes to helper JSON."""
		manifest, files = self._release("body-output")
		manifest["https_health_paths"] = ["/", "/assets/demo.woff2"]
		archive, digest = self._archive("body-output", manifest=manifest, files=files)
		fake_curl = Path(self.tempdir.name) / "fake-curl"
		fake_curl.write_text(
			f"#!{sys.executable}\n"
			"import sys\n"
			"url = sys.argv[-1]\n"
			"if '/api/scores' in url:\n"
			"\tbody = b'API_RESPONSE_BODY'\n"
			"elif url.endswith('.woff2'):\n"
			"\tbody = b'FONT_RESPONSE_BODY'\n"
			"else:\n"
			"\tbody = b'STATIC_RESPONSE_BODY'\n"
			"if '--output' in sys.argv:\n"
			"\tdestination = sys.argv[sys.argv.index('--output') + 1]\n"
			"\twith open(destination, 'wb') as stream:\n"
			"\t\tstream.write(body)\n"
			"else:\n"
			"\tsys.stdout.buffer.write(body)\n"
			"\tsys.stdout.buffer.flush()\n",
			encoding="utf-8",
		)
		os.chmod(fake_curl, 0o755)
		fake_root_command = Path(self.tempdir.name) / "fake-root-command"
		fake_root_command.write_text(
			f"#!{sys.executable}\n"
			"import sys\n"
			"sys.stdout.write('ROOT_CHILD_STDOUT')\n"
			"sys.stdout.flush()\n",
			encoding="utf-8",
		)
		os.chmod(fake_root_command, 0o755)
		driver = """
import sys
from pathlib import Path
from tools.oracle_web import host_release

host_release.CURL = sys.argv[1]
host_release.NGINX_TEST = (sys.argv[6], "nginx-test")
host_release.SYSTEMCTL = sys.argv[6]

raise SystemExit(host_release.main(
	[
		"activate", "--site", "mobile-lab", "--release-id", "body-output",
		"--archive", sys.argv[2], "--sha256", sys.argv[3],
	],
	app_root=Path(sys.argv[4]),
	staging_root=Path(sys.argv[5]),
	require_root=False,
))
"""
		completed = subprocess.run(
			[
				sys.executable, "-c", driver, str(fake_curl), str(archive), digest,
				str(self.app_root), str(self.staging_root), str(fake_root_command),
			],
			cwd=Path(__file__).resolve().parents[1],
			capture_output=True,
			check=False,
		)
		expected = {
			"current": "body-output", "previous": None,
			"backend_restarted": True, "score_reset": True,
		}
		self.assertEqual(completed.returncode, 0, completed.stderr.decode(errors="replace"))
		self.assertEqual(completed.stdout, (json.dumps(expected, sort_keys=True) + "\n").encode())

	def test_default_root_runner_bounds_a_hung_system_command(self) -> None:
		"""A stuck systemctl child must not hold the root helper transaction forever."""
		module = self._module()
		started = time.monotonic()
		with mock.patch.object(module, "ROOT_COMMAND_TIMEOUT_SECONDS", 0.05, create=True):
			with self.assertRaises(module.ReleaseError):
				module._command(
					module._default_runner,
					(sys.executable, "-c", "import time; time.sleep(0.4)"),
				)
		self.assertLess(time.monotonic() - started, 0.3)

	def test_archive_bytes_cannot_change_between_checksum_and_manifest_parse(self) -> None:
		"""Reopening the staging pathname after hashing can activate different, unverified bytes."""
		safe_files = {
			"public/index.html": b"verified safe bytes\n",
			"backend/server.py": b"print('server')\n",
			"run": b"#!/bin/sh\nexec python3 backend/server.py\n",
		}
		safe_manifest, _ = self._release("race", safe_files)
		archive, digest = self._archive("race", manifest=safe_manifest, files=safe_files)
		safe_archive = archive.read_bytes()
		hostile_files = dict(safe_files)
		hostile_files["public/index.html"] = b"replacement bytes\n"
		hostile_manifest, _ = self._release("race", hostile_files)
		self._archive("race", manifest=hostile_manifest, files=hostile_files)
		replacement_archive = archive.read_bytes()
		archive.write_bytes(safe_archive)
		module = self._module()
		original_reader = module._read_archive

		def replace_path_then_parse(source, *arguments):
			archive.write_bytes(replacement_archive)
			return original_reader(source, *arguments)

		with mock.patch.object(module, "_read_archive", side_effect=replace_path_then_parse):
			module.activate(
				"mobile-lab", "race", archive, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			)

		installed = self.app_root / "mobile-lab/releases/race/public/index.html"
		self.assertEqual(installed.read_bytes(), b"verified safe bytes\n")

	def test_staging_ancestor_symlink_is_rejected_without_app_tree_mutation(self) -> None:
		"""Leaf lstat alone permits a staging parent symlink to redirect the root helper."""
		real_staging = Path(self.tempdir.name) / "real-staging"
		symlink_parent = Path(self.tempdir.name) / "staging-parent"
		symlink_parent.mkdir()
		os.symlink(real_staging, symlink_parent / "link")
		staging_root = symlink_parent / "link"
		manifest, files = self._release("linked-parent")
		archive = real_staging / "mobile-lab/linked-parent.tar.gz"
		archive.parent.mkdir(parents=True)
		with tarfile.open(archive, "w:gz") as built:
			release_bytes = (json.dumps(manifest, sort_keys=True, indent=2) + "\n").encode()
			metadata = tarfile.TarInfo("release.json")
			metadata.mode = 0o444
			metadata.size = len(release_bytes)
			built.addfile(metadata, io.BytesIO(release_bytes))
			for path, content in files.items():
				member = tarfile.TarInfo(path)
				member.mode = 0o555 if path == "run" else 0o444
				member.size = len(content)
				built.addfile(member, io.BytesIO(content))
		digest = hashlib.sha256(archive.read_bytes()).hexdigest()

		with self.assertRaises(self._module().ReleaseError):
			self._module().activate(
				"mobile-lab", "linked-parent", staging_root / "mobile-lab/linked-parent.tar.gz", digest,
				app_root=self.app_root, staging_root=staging_root, runner=self.runner,
			)

		self.assertFalse(self.app_root.exists())

	def test_all_public_operations_reject_untrusted_app_tree_modes_and_symlinks(self) -> None:
		"""Rollback must not follow a site symlink or accept a writable app tree rejected elsewhere."""
		self._activate("old")
		site_root = self.app_root / "mobile-lab"
		moved_site = self.app_root / "moved-site"
		site_root.rename(moved_site)
		os.symlink(moved_site.name, site_root)
		archive, digest = self._archive("new")

		operations = (
			lambda: self._module().activate(
				"mobile-lab", "new", archive, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			),
			lambda: self._module().rollback("mobile-lab", "old", app_root=self.app_root, runner=self.runner),
			lambda: self._module().status("mobile-lab", app_root=self.app_root),
		)
		for operation in operations:
			with self.assertRaises(self._module().ReleaseError):
				operation()
		self.assertEqual(os.readlink(site_root), moved_site.name)

		site_root.unlink()
		moved_site.rename(site_root)
		os.chmod(site_root, 0o777)
		for operation in operations:
			with self.assertRaises(self._module().ReleaseError):
				operation()
		self.assertEqual(os.stat(site_root).st_mode & 0o777, 0o777)

	def test_new_trusted_directories_use_fixed_modes_despite_restrictive_umask(self) -> None:
		"""Root contract modes must not depend on the invoking shell or service umask."""
		archive, digest = self._archive("umask")
		previous_umask = os.umask(0o077)
		try:
			try:
				self._module().activate(
					"mobile-lab", "umask", archive, digest,
					app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
				)
			except self._module().ReleaseError as error:
				self.fail(f"fixed directory modes depended on umask: {error}")
		finally:
			os.umask(previous_umask)
		self.assertEqual(os.stat(self.app_root).st_mode & 0o777, 0o755)
		self.assertEqual(os.stat(self.app_root / "mobile-lab").st_mode & 0o777, 0o755)
		self.assertEqual(os.stat(self.app_root / "mobile-lab/releases").st_mode & 0o777, 0o755)

	def test_runtime_reconciliation_handles_remove_reorder_and_failed_remove(self) -> None:
		"""Runtime state must follow backend presence, not only the target manifest's backend flag."""
		first_static_manifest, first_static_files = self._release(
			"first-static", {"public/index.html": b"first static\n"},
		)
		first_static_manifest.pop("backend")
		first_archive, first_digest = self._archive(
			"first-static", manifest=first_static_manifest, files=first_static_files,
		)
		first_result = self._module().activate(
			"mobile-lab", "first-static", first_archive, first_digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.assertFalse(first_result["backend_restarted"])
		self.assertFalse(first_result["score_reset"])
		self.assertFalse(any(command[1] in ("restart", "stop") for command in self.runner.commands if len(command) > 1))

		second_static_manifest, second_static_files = self._release(
			"second-static", {"public/index.html": b"second static\n"},
		)
		second_static_manifest.pop("backend")
		second_archive, second_digest = self._archive(
			"second-static", manifest=second_static_manifest, files=second_static_files,
		)
		self.runner.commands.clear()
		second_result = self._module().activate(
			"mobile-lab", "second-static", second_archive, second_digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.assertFalse(second_result["backend_restarted"])
		self.assertFalse(second_result["score_reset"])
		self.assertFalse(any(command[1] in ("restart", "stop") for command in self.runner.commands if len(command) > 1))

		self.runner.commands.clear()
		self._activate("backend-old")
		self.assertIn(("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"), self.runner.commands)
		self.runner.commands.clear()
		static_manifest, static_files = self._release(
			"static", {"public/index.html": b"static only\n"},
		)
		static_manifest.pop("backend")
		archive, digest = self._archive("static", manifest=static_manifest, files=static_files)

		result = self._module().activate(
			"mobile-lab", "static", archive, digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.assertIn(("/usr/bin/systemctl", "stop", "zetin-webapp@mobile-lab.service"), self.runner.commands)
		self.assertFalse(result["backend_restarted"])
		self.assertTrue(result["score_reset"])

		# Reordering identical backend members is not a code change.
		self.runner.commands.clear()
		manifest, files = self._release("reordered")
		manifest["members"] = list(reversed(manifest["members"]))
		archive, digest = self._archive("reordered", manifest=manifest, files=files)
		result = self._module().activate(
			"mobile-lab", "reordered", archive, digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.assertTrue(result["backend_restarted"], "adding backend after static must restart it")

		manifest2, files2 = self._release("reordered-again")
		manifest2["members"] = [manifest2["members"][1], manifest2["members"][0], manifest2["members"][2]]
		archive2, digest2 = self._archive("reordered-again", manifest=manifest2, files=files2)
		self.runner.commands.clear()
		result2 = self._module().activate(
			"mobile-lab", "reordered-again", archive2, digest2,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.assertFalse(result2["backend_restarted"])
		self.assertFalse(result2["score_reset"])

		# Removing backend and then failing HTTPS must restart the prior backend on recovery.
		static2_manifest, static2_files = self._release(
			"static-fail", {"public/index.html": b"unhealthy static\n"},
		)
		static2_manifest.pop("backend")
		archive3, digest3 = self._archive("static-fail", manifest=static2_manifest, files=static2_files)
		failing = RecordingRunner(
			lambda command, _index: "static HTTPS failed"
			if (
				self._current_target() == "releases/static-fail"
				and command[-1] == "https://uos-drone.kro.kr/"
			) else None
		)
		with self.assertRaises(self._module().ReleaseError):
			self._module().activate(
				"mobile-lab", "static-fail", archive3, digest3,
				app_root=self.app_root, staging_root=self.staging_root, runner=failing,
			)
		self.assertEqual(self._current_target(), "releases/reordered-again")
		self.assertIn(
			("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
			failing.commands,
		)
		self.assertEqual(
			failing.commands[-1],
			("/usr/bin/curl", "--fail", "--silent", "--show-error", "--output", "/dev/null", "--noproxy", "*", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/presenter.html"),
		)

	def test_explicit_null_backend_is_a_static_release(self) -> None:
		"""Literal JSON null must select the static topology without a service transition."""
		manifest, files = self._release(
			"null-static", {"public/index.html": b"explicit null static\n"},
		)
		manifest["backend"] = None
		archive, digest = self._archive("null-static", manifest=manifest, files=files)
		with tarfile.open(archive, "r:gz") as built:
			release_bytes = built.extractfile("release.json").read()
		self.assertIn(b'"backend": null', release_bytes)

		try:
			result = self._module().activate(
				"mobile-lab", "null-static", archive, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			)
		except self._module().ReleaseError as error:
			self.fail(f"explicit null backend was rejected: {error}")

		self.assertEqual(
			result,
			{"current": "null-static", "previous": None, "backend_restarted": False, "score_reset": False},
		)
		self.assertEqual(self._current_target(), "releases/null-static")
		self.assertFalse(any(command[-1] == "zetin-webapp@mobile-lab.service" for command in self.runner.commands))

	def test_manifest_runtime_members_must_match_backend_topology(self) -> None:
		"""Static releases cannot smuggle runtime files, and API releases need both runtime parts."""
		static_manifest, static_files = self._release("static-has-runtime")
		static_manifest.pop("backend")
		missing_run_files = {
			"public/index.html": b"api without run\n",
			"backend/server.py": b"print('api')\n",
		}
		missing_run_manifest, _ = self._release("api-missing-run", missing_run_files)
		missing_backend_files = {
			"public/index.html": b"api without backend code\n",
			"run": b"#!/bin/sh\nexit 0\n",
		}
		missing_backend_manifest, _ = self._release("api-missing-backend", missing_backend_files)
		cases = (
			("static-has-runtime", static_manifest, static_files),
			("api-missing-run", missing_run_manifest, missing_run_files),
			("api-missing-backend", missing_backend_manifest, missing_backend_files),
		)

		for release_id, manifest, files in cases:
			with self.subTest(release_id=release_id):
				archive, digest = self._archive(release_id, manifest=manifest, files=files)
				case_app_root = Path(self.tempdir.name) / f"apps-{release_id}"
				runner = RecordingRunner()
				with self.assertRaises(self._module().ReleaseError):
					self._module().activate(
						"mobile-lab", release_id, archive, digest,
						app_root=case_app_root, staging_root=self.staging_root, runner=runner,
					)
				self.assertFalse(case_app_root.exists())
				self.assertEqual(runner.commands, [])

	def test_concurrent_activation_cannot_let_older_recovery_overwrite_newer_success(self) -> None:
		"""The site transaction lock must cover current inspection through health recovery."""
		self._activate("old")
		_, files_a = self._release("new-a")
		files_a["backend/server.py"] = b"new a\n"
		archive_a, digest_a = self._archive("new-a", files=files_a)
		_, files_b = self._release("new-b")
		files_b["backend/server.py"] = b"new b\n"
		archive_b, digest_b = self._archive("new-b", files=files_b)
		a_at_health = threading.Event()
		release_a = threading.Event()
		b_finished = threading.Event()
		results: dict[str, object] = {}

		def runner_a(command, _index=[0]):
			_index[0] += 1
			if (
				self._current_target() == "releases/new-a"
				and command[-1] == "http://127.0.0.1:18080/api/scores"
			):
				a_at_health.set()
				self.assertTrue(release_a.wait(5))
				raise RuntimeError("new-a health failed")

		def activate_a() -> None:
			try:
				self._module().activate(
					"mobile-lab", "new-a", archive_a, digest_a,
					app_root=self.app_root, staging_root=self.staging_root, runner=runner_a,
				)
			except Exception as error:
				results["a"] = error

		def activate_b() -> None:
			try:
				results["b"] = self._module().activate(
					"mobile-lab", "new-b", archive_b, digest_b,
					app_root=self.app_root, staging_root=self.staging_root, runner=RecordingRunner(),
				)
			except Exception as error:
				results["b"] = error
			finally:
				b_finished.set()

		thread_a = threading.Thread(target=activate_a)
		thread_b = threading.Thread(target=activate_b)
		with mock.patch.object(self._module(), "BACKEND_READY_TIMEOUT_SECONDS", 0, create=True):
			thread_a.start()
			self.assertTrue(a_at_health.wait(5))
			thread_b.start()
			time.sleep(0.1)
			self.assertFalse(b_finished.is_set(), "new-b must wait for new-a's transaction lock")
			release_a.set()
			thread_a.join(5)
			thread_b.join(5)
		self.assertFalse(thread_a.is_alive())
		self.assertFalse(thread_b.is_alive())
		self.assertIsInstance(results["a"], self._module().ReleaseError)
		self.assertIsInstance(results["b"], dict)
		self.assertEqual(self._current_target(), "releases/new-b")

	def test_only_validated_stale_current_next_is_reconciled_under_site_lock(self) -> None:
		"""A crash-stale valid link may be removed, but an attacker-controlled next path must block."""
		self._activate("old")
		next_link = self.app_root / "mobile-lab/current.next"
		os.symlink("releases/old", next_link)
		try:
			self._activate("new")
		except self._module().ReleaseError as error:
			self.fail(f"validated stale current.next was not reconciled: {error}")
		self.assertEqual(self._current_target(), "releases/new")
		self.assertFalse(next_link.exists())

		os.symlink("/tmp/not-a-release", next_link)
		archive, digest = self._archive("blocked")
		with self.assertRaises(self._module().ReleaseError):
			self._module().activate(
				"mobile-lab", "blocked", archive, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			)
		self.assertEqual(self._current_target(), "releases/new")
		self.assertEqual(os.readlink(next_link), "/tmp/not-a-release")

	def test_file_prefix_collision_fails_before_creating_site_tree(self) -> None:
		"""A regular file cannot also be the parent directory of another manifest member."""
		files = {"public": b"file named public\n", "public/index.html": b"nested\n"}
		manifest, _ = self._release("prefix", files)
		manifest.pop("backend")
		archive, digest = self._archive("prefix", manifest=manifest, files=files)

		with self.assertRaises(self._module().ReleaseError):
			self._module().activate(
				"mobile-lab", "prefix", archive, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			)

		self.assertFalse(self.app_root.exists())
		self.assertEqual(self.runner.commands, [])

	def test_archive_resource_limits_reject_before_creating_site_tree(self) -> None:
		"""Compressed, count, member, and total limits must stop tar bombs before app writes."""
		module = self._module()
		cases: list[tuple[str, str, Path, str]] = []
		archive_path = self.staging_root / "mobile-lab/resource-compressed.tar.gz"
		archive_path.parent.mkdir(parents=True, exist_ok=True)
		archive_path.write_bytes(os.urandom(8 * 1024 * 1024 + 1))
		cases.append(("compressed", "resource-compressed", archive_path, hashlib.sha256(archive_path.read_bytes()).hexdigest()))

		large = b"x" * (8 * 1024 * 1024 + 1)
		large_manifest, _ = self._release("resource-member", {"public/large.bin": large})
		large_manifest.pop("backend")
		large_archive, large_digest = self._archive(
			"resource-member", manifest=large_manifest, files={"public/large.bin": large},
		)
		cases.append(("member", "resource-member", large_archive, large_digest))

		empty_files = {f"public/{index:03d}.txt": b"" for index in range(257)}
		count_manifest, _ = self._release("resource-count", empty_files)
		count_manifest.pop("backend")
		count_archive, count_digest = self._archive("resource-count", manifest=count_manifest, files=empty_files)
		cases.append(("count", "resource-count", count_archive, count_digest))

		chunk = b"z" * (1024 * 1024)
		total_files = {f"public/{index:02d}.bin": chunk for index in range(33)}
		total_manifest, _ = self._release("resource-total", total_files)
		total_manifest.pop("backend")
		total_archive, total_digest = self._archive("resource-total", manifest=total_manifest, files=total_files)
		cases.append(("total", "resource-total", total_archive, total_digest))

		for label, release_id, archive, digest in cases:
			with self.subTest(label=label):
				with self.assertRaises(module.ReleaseError):
					module.activate(
						"mobile-lab", release_id, archive, digest,
						app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
					)
				self.assertFalse(self.app_root.exists())

	def test_expanded_pax_metadata_is_bounded_before_tar_parsing(self) -> None:
		"""A tiny gzip must not make tarfile expand attacker-sized PAX metadata first."""
		release_id = "resource-pax"
		archive_path = self.staging_root / "mobile-lab" / f"{release_id}.tar.gz"
		archive_path.parent.mkdir(parents=True, exist_ok=True)
		oversized_name = "x" * (40 * 1024 * 1024 + 1024)
		with tarfile.open(archive_path, "w:gz", format=tarfile.PAX_FORMAT) as archive:
			member = tarfile.TarInfo(oversized_name)
			member.mode = 0o444
			member.size = 0
			archive.addfile(member, io.BytesIO(b""))
		self.assertLess(archive_path.stat().st_size, 128 * 1024)
		digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()

		with self.assertRaisesRegex(
			self._module().ReleaseError,
			"decompressed tar stream exceeds size limit",
		):
			self._module().activate(
				"mobile-lab", release_id, archive_path, digest,
				app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
			)

		self.assertFalse(self.app_root.exists())
		self.assertEqual(self.runner.commands, [])


if __name__ == "__main__":
	unittest.main()
