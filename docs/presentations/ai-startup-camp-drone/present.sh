#!/usr/bin/env bash
set -euo pipefail

deck_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
port="${1:-8000}"
python_bin="${PRESENTATION_PYTHON_BIN:-/home/light/anaconda3/bin/python}"
chrome_bin="${PRESENTATION_CHROME_BIN:-}"
server_pid=""
browser_pid=""
runtime_dir=""

cleanup() {
  exit_code=$?
  trap - EXIT

  if [[ -n "$browser_pid" ]] && kill -0 "$browser_pid" 2>/dev/null; then
    kill "$browser_pid" 2>/dev/null || true
    wait "$browser_pid" 2>/dev/null || true
  fi
  if [[ -n "$server_pid" ]] && kill -0 "$server_pid" 2>/dev/null; then
    kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  fi
  if [[ -n "$runtime_dir" ]] && [[ -d "$runtime_dir" ]]; then
    find "$runtime_dir" -mindepth 1 -delete
    rmdir "$runtime_dir"
  fi

  exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

if [[ ! "$port" =~ ^[0-9]+$ ]] || ((port < 1 || port > 65535)); then
  echo "오류: 포트는 1~65535 범위의 정수여야 합니다: $port" >&2
  exit 2
fi

if ! command -v "$python_bin" >/dev/null 2>&1; then
  echo "오류: Python 실행 파일을 찾을 수 없습니다: $python_bin" >&2
  exit 1
fi

if [[ -z "$chrome_bin" ]]; then
  for candidate in google-chrome-stable google-chrome chromium chromium-browser; do
    if command -v "$candidate" >/dev/null 2>&1; then
      chrome_bin="$(command -v "$candidate")"
      break
    fi
  done
fi
if [[ -z "$chrome_bin" ]] || ! command -v "$chrome_bin" >/dev/null 2>&1; then
  echo "오류: Chrome 또는 Chromium 실행 파일을 찾을 수 없습니다." >&2
  exit 1
fi

runtime_root="${TMPDIR:-/tmp}"
runtime_dir="$(mktemp -d "$runtime_root/zetin-drone-presentation.XXXXXX")"
server_log="$runtime_dir/server.log"
profile_dir="$runtime_dir/chrome-profile"
url="http://127.0.0.1:$port/"

"$python_bin" -m http.server "$port" \
  --bind 127.0.0.1 \
  --directory "$deck_dir" \
  >"$server_log" 2>&1 &
server_pid=$!

server_ready=0
for _attempt in {1..50}; do
  if ! kill -0 "$server_pid" 2>/dev/null; then
    echo "오류: 발표자료 서버를 시작하지 못했습니다." >&2
    wait "$server_pid" 2>/dev/null || true
    sed -n '1,120p' "$server_log" >&2
    exit 1
  fi
  if "$python_bin" -c \
    'import sys, urllib.request; urllib.request.urlopen(sys.argv[1], timeout=0.2).close()' \
    "$url" >/dev/null 2>&1; then
    server_ready=1
    break
  fi
  sleep 0.1
done

if ((server_ready == 0)); then
  echo "오류: 발표자료 서버가 5초 안에 준비되지 않았습니다." >&2
  sed -n '1,120p' "$server_log" >&2
  exit 1
fi

echo "발표자료를 엽니다: $url"
echo "Chrome 창을 닫으면 서버도 자동으로 종료됩니다."

"$chrome_bin" \
  --user-data-dir="$profile_dir" \
  --no-first-run \
  --no-default-browser-check \
  --disable-background-mode \
  --app="$url" &
browser_pid=$!

set +e
wait "$browser_pid"
browser_status=$?
set -e
browser_pid=""

exit "$browser_status"
