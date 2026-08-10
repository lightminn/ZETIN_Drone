"""Black-box contract tests for allowlisted Oracle web release archives."""

from __future__ import annotations

import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODULE = "tools.oracle_web.build_release"
SOURCE_PREFIX = "docs/presentations/ai-startup-camp-drone"
MOBILE_LAB = f"{SOURCE_PREFIX}/mobile-lab"
FONTS = f"{SOURCE_PREFIX}/vendor/uos-slide-template/fonts"

FILE_PAIRS = (
    (f"{MOBILE_LAB}/index.html", "public/index.html"),
    (f"{MOBILE_LAB}/presenter.html", "public/presenter.html"),
    (f"{MOBILE_LAB}/styles.css", "public/styles.css"),
    (f"{MOBILE_LAB}/src/app.mjs", "public/src/app.mjs"),
    (f"{MOBILE_LAB}/src/challenge.mjs", "public/src/challenge.mjs"),
    (f"{MOBILE_LAB}/src/imu.mjs", "public/src/imu.mjs"),
    (f"{MOBILE_LAB}/src/joystick.mjs", "public/src/joystick.mjs"),
    (f"{MOBILE_LAB}/src/presenter.mjs", "public/src/presenter.mjs"),
    (f"{MOBILE_LAB}/src/score-client.mjs", "public/src/score-client.mjs"),
    (f"{MOBILE_LAB}/src/scoring.mjs", "public/src/scoring.mjs"),
    (f"{MOBILE_LAB}/vendor/qrcode-generator/LICENSE", "public/vendor/qrcode-generator/LICENSE"),
    (f"{MOBILE_LAB}/vendor/qrcode-generator/README.md", "public/vendor/qrcode-generator/README.md"),
    (f"{MOBILE_LAB}/vendor/qrcode-generator/qrcode.js", "public/vendor/qrcode-generator/qrcode.js"),
    (f"{FONTS}/NotoSansCJKkr-Regular.woff2", "public/vendor/uos-slide-template/fonts/NotoSansCJKkr-Regular.woff2"),
    (f"{FONTS}/NotoSansCJKkr-Medium.woff2", "public/vendor/uos-slide-template/fonts/NotoSansCJKkr-Medium.woff2"),
    (f"{FONTS}/NotoSansCJKkr-Bold.woff2", "public/vendor/uos-slide-template/fonts/NotoSansCJKkr-Bold.woff2"),
    (f"{MOBILE_LAB}/server.py", "backend/server.py"),
    ("tools/oracle_web/sites/mobile-lab.run", "run"),
)
EXPECTED_MEMBERS = ("release.json",) + tuple(destination for _, destination in FILE_PAIRS)


def run_git(repo: Path, *args: str) -> None:
    subprocess.run(["git", *args], cwd=repo, check=True, capture_output=True, text=True)


class OracleWebReleaseTests(unittest.TestCase):
    """The builder's public CLI must be safe against hostile manifests and repos."""

    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory(prefix="oracle-web-release-")
        self.addCleanup(self.tempdir.cleanup)
        self.repo = Path(self.tempdir.name) / "repo"
        self.repo.mkdir()
        self.config_path = self.repo / "tools/oracle_web/sites/mobile-lab.json"
        self._write_fixture()
        run_git(self.repo, "init", "-q")
        run_git(self.repo, "config", "user.email", "oracle-web-test@example.test")
        run_git(self.repo, "config", "user.name", "Oracle web release test")
        run_git(self.repo, "add", ".")
        run_git(self.repo, "commit", "-qm", "fixture")

    def _write_file(self, relative: str, content: bytes) -> None:
        path = self.repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)

    def _manifest(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "site": "mobile-lab",
            "server_name": "uos-drone.kro.kr",
            "public_ipv4": "140.83.83.165",
            "https_health_paths": [
                "/",
                "/presenter.html",
                "/src/app.mjs",
                "/vendor/uos-slide-template/fonts/NotoSansCJKkr-Regular.woff2",
            ],
            "backend": {"port": 18080, "health_path": "/api/scores"},
            "files": [
                {"source": source, "destination": destination}
                for source, destination in FILE_PAIRS
            ],
        }

    def _write_fixture(self) -> None:
        for number, (source, _) in enumerate(FILE_PAIRS, start=1):
            if source.endswith("mobile-lab.run"):
                content = (
                    b"#!/bin/sh\n"
                    b"set -eu\n"
                    b"release_root=$(CDPATH= cd -- \"$(dirname -- \"$0\")\" && pwd)\n"
                    b"exec /usr/bin/python3 \"$release_root/backend/server.py\" --host 127.0.0.1 "
                    b"--port \"${ZETIN_WEB_PORT:?ZETIN_WEB_PORT is required}\" "
                    b"--static-root \"$release_root/public\"\n"
                )
            else:
                content = f"allowlisted fixture file {number}: {source}\n".encode()
            self._write_file(source, content)
        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        self.config_path.write_text(json.dumps(self._manifest(), indent=2) + "\n")
        self._write_file(f"{MOBILE_LAB}/tests/hidden.test.mjs", b"not deployable\n")
        self._write_file(f"{MOBILE_LAB}/README.md", b"not deployable\n")
        self._write_file(f"{SOURCE_PREFIX}/deck.html", b"presentation\n")
        self._write_file(f"{SOURCE_PREFIX}/deck.pptx", b"presentation\n")
        self._write_file("scripts/control_dualsense.py", b"not deployable\n")
        self._write_file("docs/cascade_vs_single_pid.pdf", b"not deployable\n")
        self._write_file("secrets/certificate.pem", b"not deployable\n")
        self._write_file("secrets/private.key", b"not deployable\n")

    def _build(self, output: Path, release_id: str = "abc123") -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(PROJECT_ROOT)
        return subprocess.run(
            [
                sys.executable,
                "-m",
                MODULE,
                "--repo-root",
                str(self.repo),
                "--site-config",
                str(self.config_path),
                "--release-id",
                release_id,
                "--output",
                str(output),
            ],
            cwd=self.repo,
            env=environment,
            capture_output=True,
            text=True,
        )

    def _commit_config(self, manifest: dict[str, object]) -> None:
        self.config_path.write_text(json.dumps(manifest, indent=2) + "\n")
        run_git(self.repo, "add", self.config_path.relative_to(self.repo).as_posix())
        run_git(self.repo, "commit", "-qm", "change manifest")

    def test_builds_exact_deterministic_allowlist_with_normalized_metadata(self) -> None:
        """Removing allowlist enforcement or tar normalization breaks this contract."""
        archive_one = Path(self.tempdir.name) / "one.tar.gz"
        archive_two = Path(self.tempdir.name) / "two.tar.gz"
        first = self._build(archive_one)
        second = self._build(archive_two)

        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertEqual(hashlib.sha256(archive_one.read_bytes()).hexdigest(), hashlib.sha256(archive_two.read_bytes()).hexdigest())
        self.assertTrue((Path(str(archive_one) + ".sha256")).is_file())
        self.assertEqual(
            Path(str(archive_one) + ".sha256").read_text(),
            f"{hashlib.sha256(archive_one.read_bytes()).hexdigest()}  {archive_one.name}\n",
        )

        with gzip.open(archive_one, "rb") as compressed:
            with tarfile.open(fileobj=compressed, mode="r:") as archive:
                members = archive.getmembers()
                self.assertEqual(tuple(member.name for member in members), EXPECTED_MEMBERS)
                self.assertTrue(all(member.isreg() for member in members))
                for member in members:
                    self.assertEqual(member.uid, 0)
                    self.assertEqual(member.gid, 0)
                    self.assertEqual(member.uname, "root")
                    self.assertEqual(member.gname, "root")
                    self.assertEqual(member.mtime, 0)
                    self.assertEqual(member.mode, 0o555 if member.name == "run" else 0o444)
                release = json.loads(archive.extractfile("release.json").read())

        self.assertEqual(release["schema_version"], 1)
        self.assertEqual(release["site"], "mobile-lab")
        self.assertEqual(release["release_id"], "abc123")
        self.assertEqual(release["source_commit"], subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=self.repo, text=True).strip())
        self.assertEqual(release["server_name"], "uos-drone.kro.kr")
        self.assertEqual(release["public_ipv4"], "140.83.83.165")
        self.assertEqual(release["https_health_paths"], self._manifest()["https_health_paths"])
        self.assertEqual(release["backend"], {"port": 18080, "health_path": "/api/scores"})
        self.assertEqual(tuple(member["path"] for member in release["members"]), tuple(destination for _, destination in FILE_PAIRS))
        self.assertTrue(all(set(member) == {"path", "sha256", "size", "mode"} for member in release["members"]))
        for member, (source, destination) in zip(release["members"], FILE_PAIRS, strict=True):
            with self.subTest(member=destination):
                content = (self.repo / source).read_bytes()
                self.assertEqual(member["sha256"], hashlib.sha256(content).hexdigest())
                self.assertEqual(member["size"], len(content))
                self.assertEqual(member["mode"], 0o555 if destination == "run" else 0o444)

    def test_excludes_everything_outside_the_literal_allowlist(self) -> None:
        """Replacing the allowlist with a directory copy leaks unsafe repository files."""
        archive = Path(self.tempdir.name) / "release.tar.gz"
        result = self._build(archive)
        self.assertEqual(result.returncode, 0, result.stderr)
        with tarfile.open(archive, "r:gz") as built:
            names = set(built.getnames())
        forbidden = {
            "public/tests/hidden.test.mjs",
            "public/README.md",
            "deck.html",
            "deck.pptx",
            "scripts/control_dualsense.py",
            "docs/cascade_vs_single_pid.pdf",
            "secrets/certificate.pem",
            "secrets/private.key",
        }
        self.assertTrue(forbidden.isdisjoint(names))
        self.assertIn("public/vendor/qrcode-generator/README.md", names)

    def test_dirty_or_missing_allowlisted_file_fails_but_unrelated_pdf_does_not(self) -> None:
        """Dropping tracked-clean checks allows an unreviewed deployment payload."""
        allowed = self.repo / FILE_PAIRS[0][0]
        allowed.write_text("dirty\n")
        self.assertNotEqual(self._build(Path(self.tempdir.name) / "dirty.tar.gz").returncode, 0)
        run_git(self.repo, "checkout", "--", FILE_PAIRS[0][0])
        allowed.unlink()
        self.assertNotEqual(self._build(Path(self.tempdir.name) / "missing.tar.gz").returncode, 0)
        run_git(self.repo, "checkout", "--", FILE_PAIRS[0][0])
        (self.repo / "user-provided.pdf").write_bytes(b"untracked but unrelated\n")
        result = self._build(Path(self.tempdir.name) / "unrelated.tar.gz")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_rejects_invalid_release_id(self) -> None:
        """Relaxing release IDs permits unsafe release-directory names downstream."""
        for release_id in ("", "-bad", "has space", "a/../b", "a" * 65):
            with self.subTest(release_id=release_id):
                self.assertNotEqual(self._build(Path(self.tempdir.name) / "invalid.tar.gz", release_id).returncode, 0)

    def test_rejects_unsafe_or_duplicate_manifest_paths(self) -> None:
        """Removing path validation permits an archive member or source to escape its boundary."""
        cases = {
            "absolute source": ("source", "/etc/passwd"),
            "traversal source": ("source", "../outside.txt"),
            "backslash source": ("source", "docs\\outside.txt"),
            "glob source": ("source", "docs/*.html"),
            "absolute destination": ("destination", "/outside.txt"),
            "traversal destination": ("destination", "../outside.txt"),
            "backslash destination": ("destination", "public\\outside.txt"),
            "glob destination": ("destination", "public/*.html"),
        }
        for label, (field, value) in cases.items():
            with self.subTest(label=label):
                manifest = self._manifest()
                manifest["files"][0][field] = value
                self._commit_config(manifest)
                result = self._build(Path(self.tempdir.name) / f"{label}.tar.gz")
                self.assertNotEqual(result.returncode, 0)
                run_git(self.repo, "reset", "--hard", "HEAD~1")
        manifest = self._manifest()
        manifest["files"][1]["destination"] = manifest["files"][0]["destination"]
        self._commit_config(manifest)
        self.assertNotEqual(self._build(Path(self.tempdir.name) / "duplicate.tar.gz").returncode, 0)

    def test_rejects_symlink_or_nonregular_allowlisted_source(self) -> None:
        """Checking only path existence lets symlinks and directories enter a release."""
        source = self.repo / FILE_PAIRS[0][0]
        source.unlink()
        os.symlink("../presenter.html", source)
        run_git(self.repo, "add", "-A")
        run_git(self.repo, "commit", "-qm", "symlink source")
        self.assertNotEqual(self._build(Path(self.tempdir.name) / "symlink.tar.gz").returncode, 0)

        run_git(self.repo, "checkout", "HEAD~1", "--", FILE_PAIRS[0][0])
        source.unlink()
        source.mkdir()
        (source / "inside.txt").write_text("directory source\n")
        manifest = self._manifest()
        manifest["files"][0]["source"] = FILE_PAIRS[0][0]
        self.config_path.write_text(json.dumps(manifest, indent=2) + "\n")
        run_git(self.repo, "add", "-A")
        run_git(self.repo, "commit", "-qm", "directory source")
        self.assertNotEqual(self._build(Path(self.tempdir.name) / "directory.tar.gz").returncode, 0)

    def test_rejects_invalid_health_paths_and_backend_port(self) -> None:
        """Weak operational metadata validation produces unusable remote health checks."""
        cases = (
            ("health path", "https_health_paths", ["not-a-path"]),
            ("backend health", "backend", {"port": 18080, "health_path": "api/scores"}),
            ("zero port", "backend", {"port": 0, "health_path": "/api/scores"}),
            ("large port", "backend", {"port": 65536, "health_path": "/api/scores"}),
            ("boolean port", "backend", {"port": True, "health_path": "/api/scores"}),
        )
        for label, key, value in cases:
            with self.subTest(label=label):
                manifest = self._manifest()
                manifest[key] = value
                self._commit_config(manifest)
                self.assertNotEqual(self._build(Path(self.tempdir.name) / f"{label}.tar.gz").returncode, 0)
                run_git(self.repo, "reset", "--hard", "HEAD~1")


if __name__ == "__main__":
    unittest.main()
