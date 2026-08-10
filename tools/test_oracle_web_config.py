"""Contract tests for reusable Oracle Nginx and systemd configuration."""

from __future__ import annotations

import importlib
import json
import os
from pathlib import Path
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


if __name__ == "__main__":
	unittest.main()
