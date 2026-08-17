import os
from pathlib import Path
import socket
import subprocess
import tempfile
import textwrap
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = (
    REPO_ROOT
    / "docs"
    / "presentations"
    / "ai-startup-camp-drone"
    / "present.sh"
)


def _unused_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _port_accepts_connections(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.25)
        return sock.connect_ex(("127.0.0.1", port)) == 0


class PresentationLauncherTests(unittest.TestCase):
    def _run_launcher(self, browser_exit):
        self.assertTrue(LAUNCHER.exists(), f"launcher is missing: {LAUNCHER}")

        temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temp_dir.cleanup)
        root = Path(temp_dir.name)
        runtime_root = root / "runtime"
        runtime_root.mkdir()
        browser = root / "fake-chrome"
        browser_args = root / "browser-args.txt"
        response_body = root / "response.html"
        browser.write_text(
            textwrap.dedent(
                """\
                #!/usr/bin/env bash
                set -euo pipefail
                printf '%s\n' "$@" > "$PRESENTATION_TEST_BROWSER_ARGS"
                url=""
                for arg in "$@"; do
                  case "$arg" in
                    --app=*) url="${arg#--app=}" ;;
                  esac
                done
                test -n "$url"
                curl --fail --silent --show-error "$url" > "$PRESENTATION_TEST_BODY"
                exit "$PRESENTATION_TEST_BROWSER_EXIT"
                """
            )
        )
        browser.chmod(0o755)

        port = _unused_port()
        env = os.environ.copy()
        env.update(
            {
                "TMPDIR": str(runtime_root),
                "PRESENTATION_CHROME_BIN": str(browser),
                "PRESENTATION_PYTHON_BIN": "/home/light/anaconda3/bin/python",
                "PRESENTATION_TEST_BROWSER_ARGS": str(browser_args),
                "PRESENTATION_TEST_BODY": str(response_body),
                "PRESENTATION_TEST_BROWSER_EXIT": str(browser_exit),
            }
        )
        result = subprocess.run(
            [str(LAUNCHER), str(port)],
            cwd=REPO_ROOT,
            env=env,
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
        return result, port, runtime_root, browser_args, response_body

    def test_closing_browser_stops_server_and_removes_runtime_profile(self):
        result, port, runtime_root, browser_args, response_body = (
            self._run_launcher(browser_exit=0)
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("자작 드론", response_body.read_text())
        self.assertIn(
            f"--app=http://127.0.0.1:{port}/", browser_args.read_text()
        )
        self.assertFalse(_port_accepts_connections(port))
        self.assertEqual(list(runtime_root.iterdir()), [])

    def test_browser_failure_code_is_preserved_after_server_cleanup(self):
        result, port, runtime_root, _browser_args, response_body = (
            self._run_launcher(browser_exit=7)
        )

        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertIn("자작 드론", response_body.read_text())
        self.assertFalse(_port_accepts_connections(port))
        self.assertEqual(list(runtime_root.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
