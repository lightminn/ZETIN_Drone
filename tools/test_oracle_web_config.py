"""Contract tests for reusable Oracle Nginx and systemd configuration."""

from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODULE = "tools.oracle_web.render_site"
SITE_CONFIG = PROJECT_ROOT / "tools/oracle_web/sites/mobile-lab.json"
TEMPLATE_ROOT = PROJECT_ROOT / "tools/oracle_web/templates"


class OracleWebConfigTests(unittest.TestCase):
	def setUp(self) -> None:
		self.tempdir = tempfile.TemporaryDirectory(prefix="oracle-web-config-")
		self.addCleanup(self.tempdir.cleanup)
		self.root = Path(self.tempdir.name)
		self.output = self.root / "output"
		self.manifest = json.loads(SITE_CONFIG.read_text(encoding="utf-8"))

	def _module(self):
		try:
			return importlib.import_module(MODULE)
		except ModuleNotFoundError:
			self.fail(f"{MODULE} is not implemented")

	def _write_manifest(self, value: dict[str, object] | None = None) -> Path:
		path = self.root / "site.json"
		path.write_text(json.dumps(value or self.manifest) + "\n", encoding="utf-8")
		return path

	def _render(
		self,
		*,
		manifest: Path | None = None,
		certificate: str = "/etc/zetin-web/tls/uos-drone.kro.kr/fullchain.pem",
		private_key: str = "/etc/zetin-web/tls/uos-drone.kro.kr/privkey.pem",
	) -> subprocess.CompletedProcess[str]:
		environment = os.environ.copy()
		environment["PYTHONPATH"] = str(PROJECT_ROOT)
		return subprocess.run(
			[
				sys.executable,
				"-m",
				MODULE,
				"--site-config",
				str(manifest or SITE_CONFIG),
				"--certificate",
				certificate,
				"--private-key",
				private_key,
				"--output-dir",
				str(self.output),
			],
			cwd=PROJECT_ROOT,
			env=environment,
			capture_output=True,
			text=True,
		)

	def test_cli_renders_locked_down_nginx_site_and_exact_environment(self) -> None:
		"""Dropping a route, limit, timeout, or header weakens the public boundary."""
		result = self._render()
		self.assertEqual(result.returncode, 0, result.stderr)

		site_path = self.output / "mobile-lab.conf"
		env_path = self.output / "mobile-lab.env"
		self.assertEqual(env_path.read_bytes(), b"ZETIN_WEB_PORT=18080\n")
		self.assertEqual(os.stat(site_path).st_mode & 0o777, 0o644)
		self.assertEqual(os.stat(env_path).st_mode & 0o777, 0o644)

		site = site_path.read_text(encoding="utf-8")
		self.assertIn("listen 80;", site)
		self.assertIn("server_name uos-drone.kro.kr;", site)
		self.assertIn("return 308 https://uos-drone.kro.kr$request_uri;", site)
		self.assertIn("listen 443 ssl;", site)
		self.assertIn("root /srv/zetin-web/apps/mobile-lab/current/public;", site)
		self.assertIn("ssl_certificate /etc/zetin-web/tls/uos-drone.kro.kr/fullchain.pem;", site)
		self.assertIn("ssl_certificate_key /etc/zetin-web/tls/uos-drone.kro.kr/privkey.pem;", site)
		self.assertIn("location = /api/scores {", site)
		self.assertIn("proxy_pass http://127.0.0.1:18080;", site)
		self.assertIn("client_max_body_size 4096;", site)
		self.assertIn("limit_req zone=zetin_web_api burst=100 nodelay;", site)
		self.assertIn("limit_except GET POST", site)
		self.assertIn("proxy_connect_timeout 2s;", site)
		self.assertIn("proxy_send_timeout 5s;", site)
		self.assertIn("proxy_read_timeout 5s;", site)
		self.assertIn("proxy_hide_header Permissions-Policy;", site)
		self.assertIn("access_log off;", site)
		self.assertIn("autoindex off;", site)
		self.assertIn("Content-Security-Policy", site)
		self.assertIn("Strict-Transport-Security", site)
		self.assertIn("X-Content-Type-Options \"nosniff\"", site)
		self.assertIn("Referrer-Policy", site)
		self.assertIn("X-Frame-Options \"DENY\"", site)
		self.assertIn("accelerometer=(self), gyroscope=(self)", site)
		self.assertRegex(site, r"(?s)location ~\* \\.\(\?:woff\|woff2\)\$.*?expires 1d;")
		self.assertNotRegex(site, r"(?i)(?:html|css|mjs|js)[^}]*immutable")
		self.assertNotIn("location /api/", site)
		self.assertNotIn("alias ", site)
		self.assertNotRegex(site, r"@@[A-Z_]+@@")

	def test_http_redirect_and_https_content_servers_both_disable_access_logs(self) -> None:
		"""Neither public listener may inherit a global request access log."""
		result = self._render()
		self.assertEqual(result.returncode, 0, result.stderr)
		site = (self.output / "mobile-lab.conf").read_text(encoding="utf-8")
		server_blocks = site.split("server {")[1:]
		self.assertEqual(2, len(server_blocks), site)
		http_block = next(block for block in server_blocks if "listen 80;" in block)
		https_block = next(block for block in server_blocks if "listen 443 ssl;" in block)
		self.assertIn("access_log off;", http_block)
		self.assertIn("access_log off;", https_block)

	def test_module_scripts_have_javascript_mime_under_nosniff(self) -> None:
		"""Ubuntu's stock mime.types omits mjs, so the site must type modules explicitly."""
		result = self._render()
		self.assertEqual(result.returncode, 0, result.stderr)
		site = (self.output / "mobile-lab.conf").read_text(encoding="utf-8")
		module_location = (
			"location ~* \\.mjs$ {\n"
			"\t\tdefault_type application/javascript;\n"
			"\t\texpires -1;\n"
			"\t\ttry_files $uri =404;\n"
			"\t}"
		)
		self.assertIn(module_location, site)
		self.assertLess(site.index(module_location), site.index("location ~* \\.(?:html|css|js)$"))

	def test_rejects_invalid_domain_backend_port_and_nonabsolute_tls_paths(self) -> None:
		"""Relaxed values permit config injection, privileged ports, or ambiguous TLS files."""
		cases: list[tuple[str, object, str | None, str | None]] = [
			("uppercase domain", {**self.manifest, "server_name": "UOS-drone.kro.kr"}, None, None),
			("space domain", {**self.manifest, "server_name": "uos drone.kro.kr"}, None, None),
			("low port", {**self.manifest, "backend": {"port": 1023, "health_path": "/api/scores"}}, None, None),
			("high port", {**self.manifest, "backend": {"port": 65536, "health_path": "/api/scores"}}, None, None),
			("relative certificate", self.manifest, "cert.pem", None),
			("relative key", self.manifest, None, "privkey.pem"),
		]
		for label, manifest, certificate, private_key in cases:
			with self.subTest(label=label):
				self.output = self.root / label.replace(" ", "-")
				result = self._render(
					manifest=self._write_manifest(manifest),
					certificate=certificate or "/etc/zetin-web/tls/site/fullchain.pem",
					private_key=private_key or "/etc/zetin-web/tls/site/privkey.pem",
				)
				self.assertNotEqual(result.returncode, 0)
				self.assertFalse(self.output.exists())

	def test_template_renderer_rejects_missing_unknown_and_unresolved_tokens(self) -> None:
		"""Loosening token checks can silently emit incomplete or attacker-controlled config."""
		module = self._module()
		replacements = {
			"@@SITE@@": "mobile-lab",
			"@@DOMAIN@@": "uos-drone.kro.kr",
		}
		with self.assertRaises(module.RenderError):
			module.render_template("server_name @@DOMAIN@@;", replacements)
		with self.assertRaises(module.RenderError):
			module.render_template("@@SITE@@ @@DOMAIN@@ @@UNSAFE@@", replacements)
		with self.assertRaises(module.RenderError):
			module.render_template("@@SITE@@ @@DOMAIN@@ @@", replacements)

	def test_systemd_template_confines_the_single_release_launcher(self) -> None:
		"""Removing a sandbox directive or adding writable state broadens app authority."""
		unit = (TEMPLATE_ROOT / "zetin-webapp@.service").read_text(encoding="utf-8")
		self.assertIn("DynamicUser=yes", unit)
		self.assertEqual(unit.count("ExecStart="), 1)
		self.assertIn("ExecStart=/srv/zetin-web/apps/%i/current/run", unit)
		self.assertIn("ProtectSystem=strict", unit)
		self.assertIn("ProtectHome=true", unit)
		self.assertRegex(unit, r"(?m)^CapabilityBoundingSet=$")
		self.assertRegex(unit, r"(?m)^AmbientCapabilities=$")
		self.assertIn("RestrictAddressFamilies=AF_INET AF_UNIX", unit)
		self.assertIn("MemoryMax=128M", unit)
		self.assertIn("TasksMax=128", unit)
		self.assertIn("Restart=on-failure", unit)
		self.assertNotIn("StateDirectory=", unit)
		self.assertNotIn("ReadWritePaths=", unit)

	def test_api_method_guard_rejects_implicit_head_with_405(self) -> None:
		"""Nginx treats HEAD as GET unless an exact request-method guard rejects it."""
		result = self._render()
		self.assertEqual(result.returncode, 0, result.stderr)
		site = (self.output / "mobile-lab.conf").read_text(encoding="utf-8")
		self.assertIn("if ($request_method !~ ^(GET|POST)$) {", site)
		self.assertIn("return 405;", site)
		self.assertNotIn("GET|HEAD|POST", site)

	def test_bootstrap_policy_guard_is_executable_and_always_restores_prior_target(self) -> None:
		"""Apt maintainer scripts must be suppressed without consuming an existing policy."""
		bubblewrap = shutil.which("bwrap")
		if bubblewrap is None:
			self.skipTest("bubblewrap is unavailable")
		bootstrap = PROJECT_ROOT / "tools/oracle_web/bootstrap_host.sh"
		cases = (
			("absent-success", "absent", False),
			("file-success", "file", False),
			("symlink-error", "symlink", True),
		)
		for label, prior_kind, fail_install in cases:
			with self.subTest(label=label):
				fixture = self.root / label
				upper = fixture / "upper"
				work = fixture / "work"
				bin_dir = upper / "bin"
				sbin_dir = upper / "sbin"
				bin_dir.mkdir(parents=True)
				sbin_dir.mkdir()
				work.mkdir()
				apt = bin_dir / "apt-get"
				systemctl = bin_dir / "systemctl"
				apt.write_text(
					"#!/bin/sh\n"
					"test -x /usr/sbin/policy-rc.d || exit 91\n"
					"policy_status=0\n"
					"/usr/sbin/policy-rc.d fixture-action || policy_status=$?\n"
					"test \"$policy_status\" -eq 101 || exit 92\n"
					"printf '%s\\n' \"$*\" >>/usr/sbin/apt.log\n"
					"if test \"${FAKE_APT_FAIL:-0}\" = 1 && test \"$1\" = install; then exit 42; fi\n",
					encoding="utf-8",
				)
				systemctl.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
				nginx = sbin_dir / "nginx"
				nginx.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
				for executable in (apt, systemctl, nginx):
					os.chmod(executable, 0o755)
				policy = sbin_dir / "policy-rc.d"
				original = sbin_dir / "original-policy"
				if prior_kind == "file":
					policy.write_bytes(b"#!/bin/sh\nexit 23\n")
					os.chmod(policy, 0o751)
				elif prior_kind == "symlink":
					original.write_bytes(b"#!/bin/sh\nexit 23\n")
					os.chmod(original, 0o751)
					policy.symlink_to(original.name)
				command = [
					bubblewrap,
					"--unshare-user", "--uid", "0", "--gid", "0",
					"--ro-bind", "/", "/",
					"--overlay-src", "/usr", "--overlay", str(upper), str(work), "/usr",
					"--tmpfs", "/etc", "--ro-bind", "/etc/passwd", "/etc/passwd",
					"--ro-bind", "/etc/group", "/etc/group",
					"--tmpfs", "/srv", "--tmpfs", "/var", "--dir", "/var/tmp",
					"--tmpfs", "/usr/local",
					"--dev", "/dev", "--proc", "/proc",
					"--setenv", "FAKE_APT_FAIL", "1" if fail_install else "0",
					"/usr/bin/bash", str(bootstrap),
				]
				completed = subprocess.run(command, capture_output=True, text=True)
				self.assertEqual(completed.returncode, 42 if fail_install else 0, completed.stderr)
				self.assertEqual((sbin_dir / "apt.log").read_text(encoding="utf-8").splitlines()[0], "update")
				if prior_kind == "absent":
					self.assertFalse(policy.exists() or policy.is_symlink())
				elif prior_kind == "file":
					self.assertFalse(policy.is_symlink())
					self.assertEqual(policy.read_bytes(), b"#!/bin/sh\nexit 23\n")
					self.assertEqual(os.stat(policy).st_mode & 0o777, 0o751)
				else:
					self.assertTrue(policy.is_symlink())
					self.assertEqual(os.readlink(policy), original.name)

	def test_bootstrap_restores_each_prior_unit_state_without_unnecessary_transitions(self) -> None:
		"""Rerunning bootstrap must preserve both service-state axes without avoidable restarts."""
		bubblewrap = shutil.which("bwrap")
		if bubblewrap is None:
			self.skipTest("bubblewrap is unavailable")
		bootstrap = PROJECT_ROOT / "tools/oracle_web/bootstrap_host.sh"
		units = ("nginx.service", "certbot.timer")
		cases = (
			("first-install", (), (), units, units, False),
			("active-enabled-rerun", units, units, units, units, True),
			("mixed-rerun", ("nginx.service",), ("certbot.timer",), units, units, False),
			("inverse-package-effect", units, units, (), (), False),
		)
		for label, prior_active, prior_enabled, apt_active, apt_enabled, expect_no_mutations in cases:
			with self.subTest(label=label):
				fixture = self.root / label
				upper = fixture / "upper"
				work = fixture / "work"
				bin_dir = upper / "bin"
				sbin_dir = upper / "sbin"
				state_dir = sbin_dir / "systemd-state"
				active_dir = state_dir / "active"
				enabled_dir = state_dir / "enabled"
				bin_dir.mkdir(parents=True)
				active_dir.mkdir(parents=True)
				enabled_dir.mkdir()
				work.mkdir()
				for unit in prior_active:
					(active_dir / unit).touch()
				for unit in prior_enabled:
					(enabled_dir / unit).touch()

				apt = bin_dir / "apt-get"
				apt.write_text(
					"#!/bin/sh\n"
					"test -x /usr/sbin/policy-rc.d || exit 91\n"
					"policy_status=0\n"
					"/usr/sbin/policy-rc.d fixture-action || policy_status=$?\n"
					"test \"$policy_status\" -eq 101 || exit 92\n"
					"if test \"$1\" = install; then\n"
					"  for unit in nginx.service certbot.timer; do\n"
					"    rm -f -- \"/usr/sbin/systemd-state/active/$unit\"\n"
					"    rm -f -- \"/usr/sbin/systemd-state/enabled/$unit\"\n"
					"  done\n"
					"  for unit in ${FAKE_APT_ACTIVE:-}; do\n"
					"    : >\"/usr/sbin/systemd-state/active/$unit\"\n"
					"  done\n"
					"  for unit in ${FAKE_APT_ENABLED:-}; do\n"
					"    : >\"/usr/sbin/systemd-state/enabled/$unit\"\n"
					"  done\n"
					"fi\n",
					encoding="utf-8",
				)
				systemctl = bin_dir / "systemctl"
				systemctl.write_text(
					"#!/bin/sh\n"
					"state=/usr/sbin/systemd-state\n"
					"command=$1; shift\n"
					"case \"$command\" in\n"
					"  is-active|is-enabled)\n"
					"    if test \"${1:-}\" = --quiet; then shift; fi\n"
					"    test \"$#\" -eq 1 || exit 93\n"
					"    test -f \"$state/${command#is-}/$1\"\n"
					"    ;;\n"
					"  start|stop|enable|disable)\n"
					"    now=0\n"
					"    if test \"${1:-}\" = --now; then now=1; shift; fi\n"
					"    for unit do\n"
					"      printf '%s %s\\n' \"$command\" \"$unit\" >>\"$state/operations.log\"\n"
					"      case \"$command\" in\n"
					"        start) : >\"$state/active/$unit\" ;;\n"
					"        stop) rm -f -- \"$state/active/$unit\" ;;\n"
					"        enable) : >\"$state/enabled/$unit\" ;;\n"
					"        disable)\n"
					"          rm -f -- \"$state/enabled/$unit\"\n"
					"          if test \"$now\" -eq 1; then rm -f -- \"$state/active/$unit\"; fi\n"
					"          ;;\n"
					"      esac\n"
					"    done\n"
					"    ;;\n"
					"  daemon-reload) ;;\n"
					"  *) exit 94 ;;\n"
					"esac\n",
					encoding="utf-8",
				)
				nginx = sbin_dir / "nginx"
				nginx.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
				for executable in (apt, systemctl, nginx):
					os.chmod(executable, 0o755)

				command = [
					bubblewrap,
					"--unshare-user", "--uid", "0", "--gid", "0",
					"--ro-bind", "/", "/",
					"--overlay-src", "/usr", "--overlay", str(upper), str(work), "/usr",
					"--tmpfs", "/etc", "--ro-bind", "/etc/passwd", "/etc/passwd",
					"--ro-bind", "/etc/group", "/etc/group",
					"--tmpfs", "/srv", "--tmpfs", "/var", "--dir", "/var/tmp",
					"--tmpfs", "/usr/local", "--dev", "/dev", "--proc", "/proc",
					"--setenv", "FAKE_APT_ACTIVE", " ".join(apt_active),
					"--setenv", "FAKE_APT_ENABLED", " ".join(apt_enabled),
					"/usr/bin/bash", str(bootstrap),
				]
				completed = subprocess.run(command, capture_output=True, text=True)
				self.assertEqual(completed.returncode, 0, completed.stderr)
				for unit in units:
					self.assertEqual((active_dir / unit).exists(), unit in prior_active, f"{unit} active drift")
					self.assertEqual((enabled_dir / unit).exists(), unit in prior_enabled, f"{unit} enabled drift")
				if expect_no_mutations:
					operations = state_dir / "operations.log"
					self.assertFalse(operations.exists(), operations.read_text() if operations.exists() else "")

	def test_bootstrap_retains_recovery_material_when_policy_restoration_fails(self) -> None:
		"""Failed guard removal or prior-policy restore must remain visible and recoverable."""
		bubblewrap = shutil.which("bwrap")
		if bubblewrap is None:
			self.skipTest("bubblewrap is unavailable")
		bootstrap = PROJECT_ROOT / "tools/oracle_web/bootstrap_host.sh"
		cases = (
			("rm-after-success", "rm", False, 73),
			("cp-after-success", "cp", False, 74),
			("cp-after-apt-error", "cp", True, 42),
		)
		for label, failure, fail_install, expected_status in cases:
			with self.subTest(label=label):
				fixture = self.root / label
				upper = fixture / "upper"
				work = fixture / "work"
				bin_dir = upper / "bin"
				sbin_dir = upper / "sbin"
				libexec_dir = upper / "libexec"
				var_dir = fixture / "var"
				bin_dir.mkdir(parents=True)
				sbin_dir.mkdir()
				libexec_dir.mkdir()
				(var_dir / "tmp").mkdir(parents=True)
				work.mkdir()
				shutil.copy2("/usr/bin/rm", libexec_dir / "zetin-test-rm")
				shutil.copy2("/usr/bin/cp", libexec_dir / "zetin-test-cp")
				apt = bin_dir / "apt-get"
				apt.write_text(
					"#!/bin/sh\n"
					"test -x /usr/sbin/policy-rc.d || exit 91\n"
					"status=0; /usr/sbin/policy-rc.d fixture || status=$?\n"
					"test \"$status\" -eq 101 || exit 92\n"
					"if test \"${FAKE_APT_FAIL:-0}\" = 1 && test \"$1\" = install; then exit 42; fi\n",
					encoding="utf-8",
				)
				(bin_dir / "systemctl").write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
				(sbin_dir / "nginx").write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
				(bin_dir / "rm").write_text(
					"#!/bin/sh\n"
					"for arg do\n"
					"  if test \"$arg\" = /usr/sbin/policy-rc.d; then\n"
					"    count=0; test ! -f /usr/sbin/rm-count || read -r count </usr/sbin/rm-count\n"
					"    count=$((count + 1)); printf '%s\\n' \"$count\" >/usr/sbin/rm-count\n"
					"    if test \"${FAIL_POLICY_RM:-0}\" = 1 && test \"$count\" -ge 2; then\n"
					"      echo 'injected policy rm failure' >&2; exit 73\n"
					"    fi\n"
					"  fi\n"
					"done\n"
					"exec /usr/libexec/zetin-test-rm \"$@\"\n",
					encoding="utf-8",
				)
				(bin_dir / "cp").write_text(
					"#!/bin/sh\n"
					"last=; for arg do last=$arg; done\n"
					"if test \"${FAIL_POLICY_CP:-0}\" = 1 && test \"$last\" = /usr/sbin/policy-rc.d; then\n"
					"  echo 'injected policy cp failure' >&2; exit 74\n"
					"fi\n"
					"exec /usr/libexec/zetin-test-cp \"$@\"\n",
					encoding="utf-8",
				)
				for executable in (
					apt, bin_dir / "systemctl", sbin_dir / "nginx", bin_dir / "rm", bin_dir / "cp",
				):
					os.chmod(executable, 0o755)
				policy = sbin_dir / "policy-rc.d"
				original_bytes = b"#!/bin/sh\nexit 23\n"
				if failure == "cp":
					policy.write_bytes(original_bytes)
					os.chmod(policy, 0o751)
				command = [
					bubblewrap,
					"--unshare-user", "--uid", "0", "--gid", "0",
					"--ro-bind", "/", "/",
					"--overlay-src", "/usr", "--overlay", str(upper), str(work), "/usr",
					"--tmpfs", "/etc", "--ro-bind", "/etc/passwd", "/etc/passwd",
					"--ro-bind", "/etc/group", "/etc/group",
					"--tmpfs", "/srv", "--bind", str(var_dir), "/var",
					"--tmpfs", "/usr/local", "--dev", "/dev", "--proc", "/proc",
					"--setenv", "FAKE_APT_FAIL", "1" if fail_install else "0",
					"--setenv", "FAIL_POLICY_RM", "1" if failure == "rm" else "0",
					"--setenv", "FAIL_POLICY_CP", "1" if failure == "cp" else "0",
					"/usr/bin/bash", str(bootstrap),
				]
				completed = subprocess.run(command, capture_output=True, text=True)
				self.assertEqual(completed.returncode, expected_status, completed.stderr)
				self.assertIn("policy-rc.d restoration failed", completed.stderr)
				recovery_dirs = list((var_dir / "tmp").glob("zetin-web-bootstrap.*"))
				self.assertEqual(len(recovery_dirs), 1)
				if failure == "rm":
					self.assertEqual(policy.read_bytes(), b"#!/bin/sh\nexit 101\n")
				else:
					self.assertFalse(policy.exists() or policy.is_symlink())
					backup = recovery_dirs[0] / "policy-rc.d.original"
					self.assertEqual(backup.read_bytes(), original_bytes)
					self.assertEqual(os.stat(backup).st_mode & 0o777, 0o751)


if __name__ == "__main__":
	unittest.main()
