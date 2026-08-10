"""Filesystem contract tests for the root-side Oracle release helper."""

from __future__ import annotations

import hashlib
import importlib
import io
import json
import os
from pathlib import Path
import tarfile
import tempfile
import unittest
from contextlib import redirect_stdout


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
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--max-time", "5", "http://127.0.0.1:18080/api/scores"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/"),
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--max-time", "5", "--resolve", "uos-drone.kro.kr:443:127.0.0.1", "https://uos-drone.kro.kr/presenter.html"),
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

	def test_existing_release_is_idempotent_only_while_all_immutable_bytes_match(self) -> None:
		"""Overwriting an existing release ID would make rollback history mutable."""
		archive, digest = self._archive("same")
		self._module().activate(
			"mobile-lab", "same", archive, digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.runner.commands.clear()

		result = self._module().activate(
			"mobile-lab", "same", archive, digest,
			app_root=self.app_root, staging_root=self.staging_root, runner=self.runner,
		)
		self.assertEqual(
			result,
			{"current": "same", "previous": "same", "backend_restarted": False, "score_reset": False},
		)
		self.assertEqual(self.runner.commands, [])

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

	def test_loopback_health_failure_restores_previous_release_and_runtime(self) -> None:
		"""A failed backend health check must perform a real symlink and runtime rollback."""
		self._activate("old")
		_, files = self._release("new")
		files["backend/server.py"] = b"raise SystemExit('broken')\n"
		archive, digest = self._archive("new", files=files)
		failing = RecordingRunner(
			lambda command, _index: "new loopback health failed"
			if command[-1] == "http://127.0.0.1:18080/api/scores" else None
		)

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
				("/usr/bin/curl", "--fail", "--silent", "--show-error", "--max-time", "5", "http://127.0.0.1:18080/api/scores"),
				("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"),
				("/usr/bin/systemctl", "reload", "nginx"),
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
			if command[-1] == "https://uos-drone.kro.kr/presenter.html" else None
		)

		self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertEqual(self._current_target(), "releases/old")
		self.assertEqual(failing.commands[-1], ("/usr/bin/systemctl", "reload", "nginx"))
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
			if command[-1] == "http://127.0.0.1:18080/api/scores":
				return "original health error"
			if index > 4 and command == ("/usr/bin/systemctl", "restart", "zetin-webapp@mobile-lab.service"):
				return "recovery restart error"
			return None

		failing = RecordingRunner(fail_new_and_recovery)
		error = self._assert_activation_rejected("new", archive, digest, runner=failing)

		self.assertEqual(self._current_target(), "releases/old")
		self.assertIn("original health error", str(error))
		self.assertIn("rollback failed", str(error))
		self.assertIn("recovery restart error", str(error))

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
			if command[-1] == "https://uos-drone.kro.kr/" else None
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


if __name__ == "__main__":
	unittest.main()
