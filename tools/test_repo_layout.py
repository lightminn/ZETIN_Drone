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


if __name__ == "__main__":
    unittest.main()
