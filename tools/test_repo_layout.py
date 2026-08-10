import re
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_repo_layout import (  # noqa: E402
    check_markdown_links,
    check_migration_map,
    check_sketch_names,
    check_stale_tokens,
    maintained_markdown_files,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
ORACLE_RUNBOOK = REPO_ROOT / "docs" / "oracle_web_hosting.md"
BASH_BLOCK_RE = re.compile(r"```bash\n(.*?)\n```", re.DOTALL)


class RepoLayoutChecksTest(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.repo = Path(self.tempdir.name)

    def tearDown(self):
        self.tempdir.cleanup()

    def write(self, relative, content=""):
        path = self.repo / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return path

    def test_markdown_links_report_only_missing_local_target(self):
        self.write("docs/good.md", "ok")
        readme = self.write(
            "README.md",
            "[good](docs/good.md) [bad](docs/missing.md) "
            "[web](https://example.com)",
        )
        errors = check_markdown_links(self.repo, [readme])
        self.assertEqual(1, len(errors))
        self.assertIn("docs/missing.md", errors[0])

    def test_maintained_scan_includes_oracle_web_runbook_links(self):
        runbook = self.write(
            "docs/oracle_web_hosting.md",
            "[broken](missing-oracle-target.md)",
        )
        errors = check_markdown_links(
            self.repo,
            maintained_markdown_files(self.repo),
        )
        self.assertTrue(
            any(
                str(runbook.relative_to(self.repo)) in error
                and "missing-oracle-target.md" in error
                for error in errors
            ),
            errors,
        )

    def test_maintained_scan_includes_mobile_lab_readme_links(self):
        readme = self.write(
            "docs/presentations/ai-startup-camp-drone/mobile-lab/README.md",
            "[broken](missing-mobile-lab-target.md)",
        )
        errors = check_markdown_links(
            self.repo,
            maintained_markdown_files(self.repo),
        )
        self.assertTrue(
            any(
                str(readme.relative_to(self.repo)) in error
                and "missing-mobile-lab-target.md" in error
                for error in errors
            ),
            errors,
        )

    def test_sketch_directory_must_match_ino_basename(self):
        self.write("firmware/flight/right/right.ino")
        self.write("firmware/flight/wrong/not_wrong.ino")
        errors = check_sketch_names(self.repo)
        self.assertEqual(1, len(errors))
        self.assertIn("not_wrong.ino", errors[0])

    def test_migration_requires_missing_old_and_existing_new(self):
        mapping = self.write(
            "docs/migration_map.md",
            "| Old path | New path |\n|---|---|\n"
            "| `old/file.txt` | `new/file.txt` |\n",
        )
        self.write("new/file.txt")
        self.assertEqual([], check_migration_map(self.repo, mapping, 1))
        self.write("old/file.txt")
        errors = check_migration_map(self.repo, mapping, 1)
        self.assertTrue(any("old path still exists" in error for error in errors))

    def test_stale_token_is_rejected(self):
        readme = self.write("README.md", "Run Drone_Reciever.py")
        errors = check_stale_tokens(self.repo, [readme])
        self.assertEqual(1, len(errors))
        self.assertIn("Drone_Reciever.py", errors[0])


class OracleRunbookSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = ORACLE_RUNBOOK.read_text(encoding="utf-8")
        cls.bash_blocks = BASH_BLOCK_RE.findall(cls.text)

    def block_containing(self, fragment):
        matches = [block for block in self.bash_blocks if fragment in block]
        self.assertEqual(1, len(matches), (fragment, len(matches)))
        return matches[0]

    def block_containing_all(self, *fragments):
        matches = [
            block for block in self.bash_blocks
            if all(fragment in block for fragment in fragments)
        ]
        self.assertEqual(1, len(matches), (fragments, len(matches)))
        return matches[0]

    def assert_fail_fast(self, block):
        self.assertRegex(block, r"(?m)^\(\nset -euo pipefail$")

    def test_bootstrap_uses_only_the_tracked_head_snapshot(self):
        block = self.block_containing("bootstrap_remote=")
        self.assertIn("git archive --format=tar HEAD:tools/oracle_web", block)
        self.assertIn('tar -x -C "$bootstrap_local/oracle_web"', block)
        self.assertNotIn("rsync ", block)
        self.assertNotIn("git diff --quiet", block)

    def test_bootstrap_transfer_and_root_execution_are_fail_fast(self):
        block = self.block_containing("bootstrap_remote=")
        self.assert_fail_fast(block)
        self.assertIn("test ! -e '$bootstrap_remote'", block)
        self.assertIn('test -s "$backup_remote/READY"', block)
        self.assertLess(
            block.index('test -s "$backup_remote/READY"'),
            block.index('sudo -n bash "$bootstrap_remote/oracle_web/bootstrap_host.sh"'),
        )

    def test_firewall_backup_is_root_written_nonempty_and_fail_fast(self):
        block = self.block_containing_all("iptables-save.txt", "ss -lntup")
        self.assert_fail_fast(block)
        self.assertNotIn("| sudo -n tee", block)
        self.assertIn("sudo -n /bin/sh -c", block)
        self.assertIn("test ! -e '$backup_remote'", block)
        self.assertIn('sudo -n test -s "$backup_remote/iptables-save.txt"', block)
        self.assertIn("printf", block)
        self.assertIn("$backup_remote/READY", block)
        self.assertIn('sudo -n test -s "$backup_remote/READY"', block)

    def test_firewall_before_ensure_after_audit_is_fail_fast(self):
        block = self.block_containing("live.before")
        self.assert_fail_fast(block)
        self.assertIn('test -s "$firewall_audit/live.before"', block)
        self.assertIn('test -s "$firewall_audit/persistent.before"', block)
        self.assertIn('test -s "$firewall_audit/live.after"', block)
        self.assertIn('test -s "$firewall_audit/persistent.after"', block)

    def test_certbot_backup_and_issuance_are_one_fail_fast_sequence(self):
        block = self.block_containing("certbot --nginx")
        self.assert_fail_fast(block)
        self.assertIn("test ! -e", block)
        self.assertLess(block.index("cp -a"), block.index("certbot --nginx"))

    def test_access_log_validation_and_deletion_are_fail_fast(self):
        block = self.block_containing("truncate -s 0")
        self.assert_fail_fast(block)
        self.assertLess(block.index("nginx -t"), block.index("truncate -s 0"))
        self.assertLess(block.index("truncate -s 0"), block.index("unlink --"))

    def test_stock_api_path_contract_and_local_sni_probe_are_explicit(self):
        self.assertIn(
            "stock template에서는 `backend.health_path`도 `/api/scores`여야 한다",
            self.text,
        )
        self.assertIn(
            "다른 API path는 Nginx template와 테스트를 먼저 검토",
            self.text,
        )
        block = self.block_containing('https://$domain/presenter.html')
        self.assertIn('https://$domain/api/scores', block)


if __name__ == "__main__":
    unittest.main()
