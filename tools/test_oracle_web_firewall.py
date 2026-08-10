"""Drift-preservation and transaction tests for the Oracle firewall helper."""

from __future__ import annotations

import importlib
import os
from pathlib import Path
import tempfile
import unittest


HTTP_RULE = "-A INPUT -p tcp -m tcp --dport 80 -m comment --comment zetin-web:http -j ACCEPT"
QUOTED_HTTP_RULE = '-A INPUT -p tcp -m tcp --dport 80 -m comment --comment "zetin-web:http" -j ACCEPT'
USER_HTTP_RULE = "-A INPUT -p tcp -m tcp --dport 80 -j ACCEPT"
OTHER_COMMENT_RULE = '-A INPUT -p tcp -m tcp --dport 80 -m comment --comment "user:http" -j ACCEPT'
REJECT_RULE = "-A INPUT -j REJECT --reject-with icmp-port-unreachable"


def rules_text(*, live_wireguard: bool = False, http_rule: str | None = None) -> str:
	before_reject = ""
	if live_wireguard:
		before_reject += "-A INPUT -p udp -m udp --dport 51820 -j ACCEPT\n"
	if http_rule is not None:
		before_reject += f"{http_rule}\n"
	return (
		"# iptables-save fixture\n"
		"*filter\n"
		":INPUT ACCEPT [17:2048]\n"
		":FORWARD ACCEPT [0:0]\n"
		":OUTPUT ACCEPT [0:0]\n"
		"-A INPUT -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT\n"
		"-A INPUT -p tcp -m tcp --dport 25565 -j ACCEPT\n"
		f"{before_reject}"
		f"{REJECT_RULE}\n"
		"-A INPUT -p tcp -m tcp --dport 12222 -j ACCEPT\n"
		"COMMIT\n"
		"# preserve this exact boundary\n"
		"*nat\n"
		":PREROUTING ACCEPT [0:0]\n"
		":InstanceServices - [0:0]\n"
		"-A PREROUTING -j InstanceServices\n"
		"-A InstanceServices -d 169.254.0.0/16 -j ACCEPT\n"
		"COMMIT\n"
	)


class RecordingRunner:
	def __init__(self, fail_insert: bool = False) -> None:
		self.commands: list[tuple[str, ...]] = []
		self.fail_insert = fail_insert

	def __call__(self, command) -> None:
		recorded = tuple(command)
		self.commands.append(recorded)
		if self.fail_insert and "-I" in recorded:
			raise RuntimeError("fixture live insert failure")


class OracleWebFirewallTests(unittest.TestCase):
	def setUp(self) -> None:
		self.tempdir = tempfile.TemporaryDirectory(prefix="oracle-web-firewall-")
		self.addCleanup(self.tempdir.cleanup)
		self.root = Path(self.tempdir.name)
		self.persistent = self.root / "etc/iptables/rules.v4"
		self.persistent.parent.mkdir(parents=True)
		self.backup_dir = self.root / "backups"

	def _module(self):
		try:
			return importlib.import_module("tools.oracle_web.host_firewall")
		except ModuleNotFoundError:
			self.fail("tools.oracle_web.host_firewall is not implemented")

	def _write_persistent(self, content: str, mode: int = 0o640) -> None:
		self.persistent.write_text(content, encoding="utf-8")
		os.chmod(self.persistent, mode)

	def test_ensure_inserts_only_tagged_http_before_first_unconditional_reject(self) -> None:
		"""Appending or rebuilding rules would lose post-REJECT and non-filter bytes."""
		module = self._module()
		original = rules_text()
		expected = original.replace(REJECT_RULE, f"{HTTP_RULE}\n{REJECT_RULE}", 1)

		changed, inserted = module.ensure_http_rule(original)

		self.assertTrue(inserted)
		self.assertEqual(changed.encode(), expected.encode())
		self.assertEqual(changed.count(HTTP_RULE), 1)
		self.assertIn("-A INPUT -p tcp -m tcp --dport 12222 -j ACCEPT\nCOMMIT", changed)
		self.assertIn("*nat\n:PREROUTING ACCEPT [0:0]", changed)

	def test_ensure_is_idempotent_for_equivalent_rule_before_reject(self) -> None:
		"""Ignoring an existing untagged equivalent rule duplicates public access rules."""
		module = self._module()
		for existing in (HTTP_RULE, QUOTED_HTTP_RULE, USER_HTTP_RULE):
			with self.subTest(existing=existing):
				original = rules_text(http_rule=existing)
				changed, inserted = module.ensure_http_rule(original)
				self.assertFalse(inserted)
				self.assertEqual(changed, original)

	def test_ensure_rejects_missing_filter_or_unconditional_reject(self) -> None:
		"""Guessing a fallback insertion point can place HTTP after an unknown policy boundary."""
		module = self._module()
		without_filter = "*nat\n:PREROUTING ACCEPT [0:0]\nCOMMIT\n"
		without_reject = rules_text().replace(f"{REJECT_RULE}\n", "")
		for value in (without_filter, without_reject):
			with self.subTest(value=value[:20]):
				with self.assertRaises(module.FirewallError):
					module.ensure_http_rule(value)

	def test_rollback_removes_only_the_exact_tagged_rule(self) -> None:
		"""Broad dport-80 deletion would remove a user-owned firewall rule."""
		module = self._module()
		original = rules_text(http_rule=QUOTED_HTTP_RULE).replace(
			f"{QUOTED_HTTP_RULE}\n{REJECT_RULE}",
			f"{USER_HTTP_RULE}\n{OTHER_COMMENT_RULE}\n{QUOTED_HTTP_RULE}\n{REJECT_RULE}",
		)
		expected = original.replace(f"{QUOTED_HTTP_RULE}\n", "", 1)

		changed, removed = module.rollback_http_rule(original)

		self.assertTrue(removed)
		self.assertEqual(changed.encode(), expected.encode())
		self.assertIn(USER_HTTP_RULE, changed)
		self.assertIn(OTHER_COMMENT_RULE, changed)

	def test_quoted_live_and_persistent_rule_are_idempotent_and_rollback_exactly(self) -> None:
		"""Real iptables-save quoting must not hide the helper-owned live rule."""
		module = self._module()
		quoted = rules_text(live_wireguard=True, http_rule=QUOTED_HTTP_RULE)
		self._write_persistent(quoted)
		runner = RecordingRunner()

		self.assertEqual(
			module.ensure_http(
				persistent_path=self.persistent,
				backup_dir=self.backup_dir,
				save_runner=lambda: quoted,
				command_runner=runner,
			),
			{"persistent_changed": False, "live_changed": False},
		)
		self.assertEqual(runner.commands, [])

		result = module.rollback_http(
			persistent_path=self.persistent,
			backup_dir=self.backup_dir,
			save_runner=lambda: quoted,
			command_runner=runner,
		)
		self.assertEqual(result, {"persistent_changed": True, "live_changed": True})
		self.assertNotIn(QUOTED_HTTP_RULE, self.persistent.read_text(encoding="utf-8"))
		self.assertEqual(runner.commands, [
			("/usr/sbin/iptables", "-D", "INPUT", "-p", "tcp", "-m", "tcp", "--dport", "80", "-m", "comment", "--comment", "zetin-web:http", "-j", "ACCEPT"),
		])

	def test_transaction_preserves_live_drift_and_persistent_metadata(self) -> None:
		"""Saving the live rules wholesale would leak live-only WireGuard into rules.v4."""
		module = self._module()
		persistent_before = rules_text()
		live_before = rules_text(live_wireguard=True)
		self._write_persistent(persistent_before)
		runner = RecordingRunner()

		result = module.ensure_http(
			persistent_path=self.persistent,
			backup_dir=self.backup_dir,
			save_runner=lambda: live_before,
			command_runner=runner,
		)

		persistent_after = self.persistent.read_text(encoding="utf-8")
		self.assertEqual(persistent_after, persistent_before.replace(REJECT_RULE, f"{HTTP_RULE}\n{REJECT_RULE}", 1))
		self.assertNotIn("51820", persistent_after)
		self.assertEqual(os.stat(self.persistent).st_mode & 0o777, 0o640)
		backups = list(self.backup_dir.glob("rules.v4.*.bak"))
		self.assertEqual(len(backups), 1)
		self.assertEqual(backups[0].read_text(encoding="utf-8"), persistent_before)
		self.assertEqual(os.stat(backups[0]).st_mode & 0o777, 0o600)
		self.assertEqual(
			runner.commands,
			[("/usr/sbin/iptables", "-I", "INPUT", "4", "-p", "tcp", "-m", "tcp", "--dport", "80", "-m", "comment", "--comment", "zetin-web:http", "-j", "ACCEPT")],
		)
		self.assertEqual(result, {"persistent_changed": True, "live_changed": True})

	def test_live_insert_failure_leaves_persistent_bytes_restored(self) -> None:
		"""A failed live update must not leave an unmatched reboot-only opening."""
		module = self._module()
		original = rules_text()
		self._write_persistent(original)
		runner = RecordingRunner(fail_insert=True)

		with self.assertRaises(module.FirewallError):
			module.ensure_http(
				persistent_path=self.persistent,
				backup_dir=self.backup_dir,
				save_runner=lambda: rules_text(live_wireguard=True),
				command_runner=runner,
			)

		self.assertEqual(self.persistent.read_bytes(), original.encode())
		self.assertEqual(runner.commands[0][1:4], ("-I", "INPUT", "4"))

	def test_persistent_replace_failure_removes_new_live_rule(self) -> None:
		"""A failed durable update must compensate the newly inserted live opening."""
		module = self._module()
		original = rules_text()
		self._write_persistent(original)
		runner = RecordingRunner()

		def fail_replace(source, destination) -> None:
			raise OSError("fixture replace failure")

		with self.assertRaises(module.FirewallError):
			module.ensure_http(
				persistent_path=self.persistent,
				backup_dir=self.backup_dir,
				save_runner=lambda: rules_text(live_wireguard=True),
				command_runner=runner,
				replace_func=fail_replace,
			)

		self.assertEqual(self.persistent.read_bytes(), original.encode())
		self.assertEqual(len(runner.commands), 2)
		self.assertEqual(runner.commands[0][1:4], ("-I", "INPUT", "4"))
		self.assertEqual(runner.commands[1][1:3], ("-D", "INPUT"))
		self.assertEqual(runner.commands[1][3:], runner.commands[0][4:])

	def test_rollback_restores_persistent_snapshot_when_replace_then_sync_fails(self) -> None:
		"""A post-replace durability failure must not leave persistence ahead of live state."""
		module = self._module()
		original = rules_text(http_rule=HTTP_RULE)
		self._write_persistent(original, mode=0o640)
		runner = RecordingRunner()
		calls = 0

		def replace_then_fail_once(source, destination) -> None:
			nonlocal calls
			calls += 1
			os.replace(source, destination)
			if calls == 1:
				raise OSError("fixture failure after replace")

		with self.assertRaises(module.FirewallError):
			module.rollback_http(
				persistent_path=self.persistent,
				backup_dir=self.backup_dir,
				save_runner=lambda: original,
				command_runner=runner,
				replace_func=replace_then_fail_once,
			)

		self.assertEqual(self.persistent.read_bytes(), original.encode())
		self.assertEqual(os.stat(self.persistent).st_mode & 0o777, 0o640)
		self.assertEqual(runner.commands, [])
		self.assertEqual(calls, 2)


if __name__ == "__main__":
	unittest.main()
