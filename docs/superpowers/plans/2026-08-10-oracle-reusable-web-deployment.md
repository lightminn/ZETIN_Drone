# Oracle Reusable Web Deployment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Oracle 개인 서버에 정적 웹앱과 선택적 Python API를 반복해서 안전하게 배포할 수 있는 Nginx·systemd·원자적 release 기반을 만들고, 첫 사이트로 AI 창업 캠프 모바일 드론 랩을 `https://uos-drone.kro.kr/`에 배포한다.

**Architecture:** Nginx가 공개 80/443, TLS, 정적 파일, 요청 제한을 담당한다. 모바일 랩의 표준 라이브러리 Python 점수 API는 `127.0.0.1:18080`의 단일 hardened systemd 프로세스로만 실행한다. 로컬 도구가 명시적 allowlist release를 결정적으로 만들고, 서버의 최소 권한 activation helper가 tar 안전성·해시를 검증한 뒤 `current` symlink를 원자적으로 교체한다.

**Tech Stack:** Python 3.12 표준 라이브러리, POSIX shell, Nginx 1.24, systemd, Certbot, SSH/SCP, Node `node:test`, Python `unittest`, Chrome/Chromium.

## Global Constraints

- 사용자 소유 미추적 파일 `docs/cascade_vs_single_pid.pdf`를 수정·삭제·스테이징·배포하지 않는다.
- 기존 발표 HTML/PPTX를 수정하거나 재생성하지 않는다. release에는 모바일 랩 allowlist만 포함한다.
- 현재 미커밋 `server.py`와 `tools/test_mobile_lab_server.py`의 idle TLS 회귀 patch를 reset하거나 덮어쓰지 않고 Task 1에서 보완해 함께 커밋한다.
- 실제 드론·펌웨어·지상국과 연결되는 UDP, serial, WebUSB, WebBluetooth, arm, throttle, disarm, gain 경로를 추가하지 않는다.
- Oracle의 기존 `22`, `25565`, `12222`, `13389`, `127.0.0.1:18443` listener, SSH Match 블록, reverse tunnel 계정·키, Samba와 기존 iptables 규칙을 변경하지 않는다.
- UFW를 설치하거나 활성화하지 않는다. 기존 iptables-nft의 첫 최종 INPUT REJECT 바로 앞에 누락된 80 허용 규칙만 comment-tagged 형태로 멱등 추가한다. live rules와 `/etc/iptables/rules.v4`가 이미 drift하므로 `netfilter-persistent save`로 전체 live rules를 덮어쓰지 않고, 영속 파일에도 같은 한 줄만 원자적으로 삽입한다. 443 규칙은 중복 추가하지 않는다.
- 인증서·개인키·ACME 계정·TXT token을 저장소, release, 명령 출력, 로그에 넣지 않는다.
- OCI 제어면 또는 DNS에 접근할 구성된 자격이 없으면 서버 준비와 내부 검증까지 완료하고, 정확한 단일 사용자 조치만 요청한다.
- 실제 Oracle 재부팅은 기존 터널을 끊을 수 있으므로 별도 명시 승인 없이 하지 않는다. 신규 Nginx와 모바일 랩 서비스 restart/reload만 허용한다.
- 모든 테스트는 `/home/light/anaconda3/bin/python`을 사용하고 bytecode/cache는 `/tmp`로 보낸다.

---

## Task 1: 공개 점수 API 상한과 직접 TLS 회귀를 먼저 고정한다

**Files:**
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/server.py`
- Modify: `tools/test_mobile_lab_server.py`
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/score-client.test.mjs`
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/tests/challenge.test.mjs`

- [ ] **Step 1: 500개 상한의 실패 테스트 작성**

`tools/test_mobile_lab_server.py`에 실제 HTTP 경계 시험을 추가한다.

1. 서로 다른 유효 UUID 500개를 제출하면 모두 201이고 count가 500이다.
2. 이미 저장된 payload의 동일 retry는 상한 이후에도 200, `duplicate: true`다.
3. 501번째 새 UUID는 503과 정확한 JSON `{"error":"score submission capacity reached"}`다.
4. 같은 기존 ID의 다른 payload는 상한에서도 409를 유지한다.
5. 거부 뒤 GET count와 top 10 snapshot은 바뀌지 않는다.
6. 499개를 채운 뒤 서로 다른 50개를 동시에 제출하면 정확히 1개만 성공하고 49개는
   cap 거부이며 최종 count는 500이다.
7. `submitScore()`가 503을 받아도 호출자가 보유한 payload를 바꾸지 않고 기존 계약인
   calm `{status: "rejected", response}`를 반환한다. 화면의 로컬 결과는 유지된다.

Run and confirm RED:

```bash
PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-red \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_mobile_lab_server.MobileLabServerTest.test_submission_capacity_keeps_existing_retries_idempotent -v
```

Expected: 501번째 요청이 현재 201이어서 실패한다.

- [ ] **Step 2: 잠금 안에서 상한을 구현**

`server.py`에 `MAX_UNIQUE_SUBMISSIONS = 500`과 `SubmissionCapacityExceeded`를 추가하고,
`ScoreStore(max_submissions=MAX_UNIQUE_SUBMISSIONS)`와 `build_server(...,
max_submissions=MAX_UNIQUE_SUBMISSIONS)`로 작은 cap을 주입할 수 있게 한다.
`ScoreStore.submit()`은 payload 검증 후 lock 안에서 반드시 다음 순서로 처리한다.

```python
existing = self._by_id.get(submission_id)
if existing is not None:
    # 동일 payload retry 또는 conflict를 먼저 처리
if len(self._by_id) >= self._max_submissions:
    raise SubmissionCapacityExceeded("score submission capacity reached")
# 신규 record 삽입
```

생성자는 양의 정수 cap만 `self._max_submissions`에 저장한다. HTTP handler는 이 예외만
503으로 직렬화하고 `Cache-Control: no-store`를 유지한다. 클라이언트 결과 보존 동작은
기존 `score-client.mjs`의 rejected 계약을 바꾸지 않는다.

- [ ] **Step 3: idle TLS를 accept HOL에서 제한된 worker 경계로 옮긴다**

먼저 기존 black-box self-signed TLS 시험을 확장해 다음 RED를 만든다.

- 한 idle raw TCP 연결 중 두 번째 정상 TLS `/api/scores`가 2초 안에 200이다.
- handshake 상한보다 많은 idle raw 연결을 열어도 idle worker는 짧은 timeout 뒤
  회수되고 다음 정상 TLS GET이 성공한다.
- 상한 초과 연결은 무한 worker로 남지 않고 닫히며, server close가 정해진 시간 안에
  끝난다.

`MobileLabHTTPServer.process_request_thread()`가 TLS socket에 한해 worker 안에서
명시적으로 `do_handshake()`를 수행하도록 한다. handshake timeout, 정상 HTTP read
timeout, `threading.BoundedSemaphore` 기반 동시 handshake 상한을 상수로 둔다.
timeout·TLS 오류·상한 초과는 socket을 정확히 한 번 닫고 semaphore를 반드시 반환한다.
listener는 `do_handshake_on_connect=False`를 유지한다. Oracle production에서는 여전히
Nginx가 TLS를 종료하며 이 직접 TLS 경로를 공개하지 않는다.

- [ ] **Step 4: 기존 challenge 합성 시험의 빠진 명시 assertion 보강**

각 seed의 20초 calibration helper가 `elapsedMs === 20_000`, `finished === true`를 assert하고, update 전 snapshot을 구조 복사해 입력 state가 바뀌지 않았음을 직접 assert한다. 제품 코드 변경 없이 시험이 통과해야 한다.

- [ ] **Step 5: 관련 시험 실행 및 커밋**

```bash
node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
PYTHONPYCACHEPREFIX=/tmp/zetin-mobile-lab-task1 \
  /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server -v
git diff --check
git add docs/presentations/ai-startup-camp-drone/mobile-lab/server.py \
  docs/presentations/ai-startup-camp-drone/mobile-lab/tests/challenge.test.mjs \
  docs/presentations/ai-startup-camp-drone/mobile-lab/tests/score-client.test.mjs \
  tools/test_mobile_lab_server.py
git commit -m "fix: harden mobile lab score service"
```

---

## Task 2: allowlist 기반 결정적 release builder를 TDD로 만든다

**Files:**
- Create: `tools/oracle_web/__init__.py`
- Create: `tools/oracle_web/common.py`
- Create: `tools/oracle_web/site_manifest.py`
- Create: `tools/oracle_web/build_release.py`
- Create: `tools/oracle_web/sites/mobile-lab.json`
- Create: `tools/oracle_web/sites/mobile-lab.run`
- Create: `tools/test_oracle_web_release.py`

- [ ] **Step 1: archive 계약 실패 테스트 작성**

임시 repo fixture로 builder CLI를 실행해 다음을 검증한다.

- 산출물 member가 정확히 `release.json`, `public/index.html`, `public/presenter.html`, `public/styles.css`, 7개 `public/src/*.mjs`, QR 라이브러리 3파일, 글꼴 3파일, `backend/server.py`, `run`이다.
- `tests/`, 모바일 랩 top-level `README.md`, presentation HTML/PPTX,
  `scripts/control_dualsense.py`, PDF, `.git`, 인증서·키는 없다. 공개 allowlist에 명시된
  QR 라이브러리의 `public/vendor/qrcode-generator/README.md`는 포함한다.
- archive member는 regular file뿐이고 uid/gid 0, uname/gname `root`, mtime 0이다.
- launcher만 0555, 나머지 archive 파일은 0444다. host helper가 필요한 directory를
  0555로 만든다.
- 동일 입력과 release ID로 두 번 만든 `.tar.gz` SHA-256이 같다.
- 누락·dirty allowlist 파일, 잘못된 release ID, repo 밖으로 탈출하는 source/member
  경로는 비정상 종료한다. output archive는 명시적 `/tmp` 등 repo 밖 경로를 허용한다.
  allowlist 밖 사용자 PDF가 dirty/untracked인 것은 build를 막지 않는다.
- manifest의 absolute/traversal/backslash/glob source·destination, 중복 destination,
  symlink/non-regular source, 잘못된 health path·port를 거부한다.

Run and confirm RED:

```bash
PYTHONPYCACHEPREFIX=/tmp/oracle-web-release-red \
  /home/light/anaconda3/bin/python -m unittest tools.test_oracle_web_release -v
```

- [ ] **Step 2: 공통 검증과 결정적 tar 구현**

`common.py`에 다음 pure helper를 구현한다.

```python
validate_site_name(value: str) -> str
validate_release_id(value: str) -> str
sha256_file(path: Path) -> str
```

site는 `[a-z][a-z0-9-]{0,62}`, release는 `[A-Za-z0-9][A-Za-z0-9._-]{0,63}`만 허용한다.

`site_manifest.py`는 schema 1 JSON을 dataclass로 읽고 strict validation한다. 사이트
manifest에는 glob 없이 source→destination 18개를 모두 적으며 다음 운영 메타데이터를
포함한다.

```json
{
  "schema_version": 1,
  "site": "mobile-lab",
  "server_name": "uos-drone.kro.kr",
  "public_ipv4": "140.83.83.165",
  "https_health_paths": [
    "/",
    "/presenter.html",
    "/src/app.mjs",
    "/vendor/uos-slide-template/fonts/NotoSansCJKkr-Regular.woff2"
  ],
  "backend": {"port": 18080, "health_path": "/api/scores"},
  "files": []
}
```

`files`에는 3개 top-level UI, 7개 MJS, QR 3파일, WOFF2 3파일,
`server.py → backend/server.py`, `mobile-lab.run → run`을 literal로 기록한다.

`build_release.py`는 site config와 관련 manifest source가 모두 tracked regular
file이고 `git diff --quiet HEAD -- <literal paths>`인지를 확인한다. 그 뒤
`gzip.GzipFile(mtime=0, filename="")`와 `tarfile`을 사용해 release를 결정적으로
만든다. CLI는 다음으로 고정한다.

```bash
/home/light/anaconda3/bin/python -m tools.oracle_web.build_release \
  --repo-root . --site-config tools/oracle_web/sites/mobile-lab.json \
  --release-id <clean-git-sha> --output /tmp/mobile-lab-<sha>.tar.gz
```

`release.json`은 schema 1, site, release ID, source commit, `server_name`, `public_ipv4`,
`https_health_paths`, optional backend port/health path, 각 member의 SHA-256, byte size와
mode를 key 정렬 JSON으로 기록한다. 원격 activation과 status는 이 immutable metadata를
읽는다. archive `.sha256` sidecar도 만든다.
launcher는 자신의 release root를 계산해 다음만 실행한다.

```text
/usr/bin/python3 backend/server.py --host 127.0.0.1 --port $ZETIN_WEB_PORT --static-root public
```

- [ ] **Step 3: 시험 및 구현 파일 커밋**

```bash
PYTHONPYCACHEPREFIX=/tmp/oracle-web-release \
  /home/light/anaconda3/bin/python -m unittest tools.test_oracle_web_release -v
git diff --check
git add tools/oracle_web/__init__.py tools/oracle_web/common.py \
  tools/oracle_web/site_manifest.py tools/oracle_web/build_release.py \
  tools/oracle_web/sites tools/test_oracle_web_release.py
git commit -m "feat: build allowlisted Oracle web releases"
```

- [ ] **Step 4: clean commit의 제품 repo에서 실물 archive 검사**

```bash
release_id=$(git rev-parse --short=12 HEAD)
/home/light/anaconda3/bin/python -m tools.oracle_web.build_release \
  --repo-root . --site-config tools/oracle_web/sites/mobile-lab.json \
  --release-id "$release_id" \
  --output "/tmp/mobile-lab-$release_id.tar.gz"
tar -tzf "/tmp/mobile-lab-$release_id.tar.gz"
sha256sum "/tmp/mobile-lab-$release_id.tar.gz"
```

`tar -tzf` 전체 목록을 확인해 사용자 PDF, PPTX, 펌웨어, 지상국, 테스트가 없음을 확인한다.

---

## Task 3: 안전한 원격 release activation·rollback helper를 TDD로 만든다

**Files:**
- Create: `tools/oracle_web/host_release.py`
- Create: `tools/test_oracle_web_host_release.py`

- [ ] **Step 1: temp root에서 실제 파일시스템 실패 테스트 작성**

mock SSH가 아니라 임시 `/srv` 대체 root와 실제 tar를 사용한다.

- 올바른 SHA와 manifest이면 `releases/<id>`를 root 계약 mode로 만들고 `current`를 상대 symlink `releases/<id>`로 원자 교체한다.
- 기존 current가 있으면 새 활성화 뒤 `previous_release`로 보고한다.
- checksum mismatch, site/release validation 실패, absolute/`..` path, symlink, hardlink, device, manifest 누락·불일치·추가 file이면 current가 그대로다.
- 이미 같은 immutable release가 있고 manifest가 같으면 멱등 성공한다. 내용이 다르면 덮어쓰지 않고 실패한다.
- 실패 중 임시 디렉터리만 해당 site 아래에서 정리하고 다른 site/release는 유지한다.
- rollback은 존재하는 검증된 release로만 current를 교체하고 삭제하지 않는다.
- pre-switch `nginx -t` 실패는 current를 바꾸지 않는다.
- static member만 바뀐 release는 backend service를 restart하지 않아 기존 점수를
  보존한다. `backend/` 또는 `run` hash 변경과 첫 배포만 service를 restart하고 결과에
  `score_reset: true`를 명시한다.
- switch 후 loopback API 또는 local-SNI HTTPS health 실패는 이전 current로 원자
  복구하고 필요한 service restart와 Nginx reload를 수행한다. rollback 실패도 원래
  오류와 함께 비정상 종료로 보존한다.

- [ ] **Step 2: extraction과 atomic symlink 구현**

CLI는 서버에서 root로 다음처럼 사용한다.

```bash
sudo -n /usr/local/sbin/zetin-web-release activate \
  --site mobile-lab --release-id <id> \
  --archive /var/tmp/zetin-web-staging/mobile-lab/<id>.tar.gz --sha256 <hex>
sudo -n /usr/local/sbin/zetin-web-release rollback \
  --site mobile-lab --release-id <previous-id>
sudo -n /usr/local/sbin/zetin-web-release status --site mobile-lab
```

production app root와 staging root는 각각 코드 상수 `/srv/zetin-web/apps`와
`/var/tmp/zetin-web-staging`이다. 테스트만 함수 인자로 임시 root와 command runner를
전달한다. `tarfile.extractall()`은 사용하지 않고 전체 archive와 `release.json`을 먼저
검증한 뒤 directory/regular file을 직접 복사한다. archive는 성공 여부와 관계없이
helper가 삭제하지 않아 로컬 deploy wrapper가 명시적으로 정리한다.

activation의 고정 순서는 다음이다.

1. `/usr/sbin/nginx -t`
2. `current.next` relative symlink를 만들고 `os.replace()`로 current 교체
3. 첫 배포 또는 backend/run hash 변경 때만
   `systemctl restart zetin-webapp@<validated-site>.service`
4. `systemctl reload nginx`
5. manifest의 loopback backend health 확인
6. manifest의 모든 HTTPS health path를 `curl --resolve
   <domain>:443:127.0.0.1`로 확인

어느 post-switch 단계든 실패하면 이전 symlink로 같은 방식으로 복원한다. 첫 배포라
이전 current가 없었으면 자신이 만든 current symlink 하나만 unlink한다. release
directory는 자동 삭제하지 않는다. CLI는 machine-readable JSON으로 current,
previous, backend_restarted, score_reset을 출력한다.

- [ ] **Step 3: 시험 및 커밋**

```bash
PYTHONPYCACHEPREFIX=/tmp/oracle-web-activate \
  /home/light/anaconda3/bin/python -m unittest tools.test_oracle_web_host_release -v
git diff --check
git add tools/oracle_web/host_release.py tools/test_oracle_web_host_release.py
git commit -m "feat: activate Oracle web releases atomically"
```

---

## Task 4: Nginx·systemd config renderer와 host bootstrap을 TDD로 만든다

**Files:**
- Create: `tools/oracle_web/render_site.py`
- Create: `tools/oracle_web/host_firewall.py`
- Create: `tools/oracle_web/bootstrap_host.sh`
- Create: `tools/oracle_web/templates/nginx-limits.conf`
- Create: `tools/oracle_web/templates/nginx-site.conf`
- Create: `tools/oracle_web/templates/zetin-webapp@.service`
- Create: `tools/test_oracle_web_config.py`
- Create: `tools/test_oracle_web_firewall.py`

- [ ] **Step 1: renderer와 정책 실패 테스트 작성**

CLI를 임시 output directory에 실행해 실제 생성 파일을 검사한다.

```bash
/home/light/anaconda3/bin/python -m tools.oracle_web.render_site \
  --site-config tools/oracle_web/sites/mobile-lab.json \
  --certificate /etc/zetin-web/tls/uos-drone.kro.kr/fullchain.pem \
  --private-key /etc/zetin-web/tls/uos-drone.kro.kr/privkey.pem \
  --output-dir /tmp/mobile-lab-config
```

검증 항목:

- domain은 DNS label, port는 1024..65535, cert/key는 absolute path만 허용한다.
- HTTP 80은 동일 host HTTPS로 308 redirect한다.
- HTTPS root는 `/srv/zetin-web/apps/mobile-lab/current/public`이다.
- `/api/scores`는 exact location이고 `127.0.0.1:18080`만 proxy한다.
- body 4096, burst 100, GET/POST 외 거부, connect 2s/send·read 5s, access log off,
  autoindex off, upstream `Permissions-Policy` 숨김이다.
- CSP, HSTS, nosniff, referrer, frame 차단, `accelerometer=(self), gyroscope=(self)`가 있다.
- 글꼴은 1일 cache, HTML/JS/CSS는 장기 immutable cache가 아니다.
- systemd는 `DynamicUser=yes`, 단일 ExecStart launcher, `ProtectSystem=strict`, `ProtectHome=true`, capability empty, AF_INET/AF_UNIX, 128M/128 task, restart on failure다. 메모리 점수판은 영속 상태가 아니므로 `StateDirectory`와 writable path는 없다.
- 넓은 `/api/` proxy와 Nginx `alias`는 없다.
- 출력에 unresolved token이 없다.

- [ ] **Step 2: template과 strict renderer 구현**

`render_site.py`는 strict site manifest를 읽고 `str.replace()`로 허용된
`@@SITE@@`, `@@DOMAIN@@`, `@@PORT@@`, `@@CERTIFICATE@@`, `@@PRIVATE_KEY@@`만
치환한다. 누락·잔여 token이 있으면 실패한다. env는 정확히
`ZETIN_WEB_PORT=18080` 한 줄이고 mode 0644다.

- [ ] **Step 3: 멱등 bootstrap 구현**

`bootstrap_host.sh`는 root가 아니면 실패하고, 다음만 수행한다.

1. `apt-get update`, `apt-get install --yes nginx certbot python3-certbot-nginx curl rsync`.
2. `/srv/zetin-web/apps`, `/etc/zetin-web`, `/etc/zetin-web/tls`, `/var/lib/zetin-web`를 0755/root로 만든다.
3. `tools/oracle_web` Python package를 root-owned
   `/usr/local/lib/zetin-web/oracle_web/`에 설치한다. 고정 wrapper
   `/usr/local/sbin/zetin-web-release`와 `/usr/local/sbin/zetin-web-firewall`은
   `PYTHONPATH=/usr/local/lib/zetin-web /usr/bin/python3 -m
   oracle_web.<host_module>`만 실행한다. systemd template와 Nginx limits도 `install`로
   root 소유 배치한다.
4. 기존 대상 파일 내용이 다르면 `/var/backups/zetin-web/<UTC timestamp>/`에 mode 0600 복사한 뒤 교체한다.
5. `systemctl daemon-reload`, `nginx -t`를 실행한다. 사이트 생성·방화벽·인증서·DNS는 건드리지 않는다.

Shell syntax는 다음으로 검증한다.

```bash
bash -n tools/oracle_web/bootstrap_host.sh
```

- [ ] **Step 4: 기존 rules drift를 보존하는 방화벽 helper를 TDD로 구현**

`host_firewall.py`는 root 전용 `ensure-http`와 `rollback-http`만 제공한다. fixture
rules text를 사용한 시험에서 다음을 증명한다.

- 첫 unconditional `-A INPUT -j REJECT ...` 직전에 정확히
  `-A INPUT -p tcp -m tcp --dport 80 -m comment --comment zetin-web:http -j ACCEPT`
  한 줄을 삽입한다.
- REJECT 뒤의 기존 규칙, WireGuard 같은 live-only 규칙, nat/InstanceServices chain과
  모든 기존 byte를 그 외에는 보존한다.
- REJECT 앞에 이미 동등한 80 ACCEPT가 있으면 comment 유무와 무관하게 중복하지 않는다.
- unconditional REJECT나 filter table이 없으면 변경 없이 실패한다.
- rollback은 `zetin-web:http` comment가 있는 정확한 한 줄만 제거하며 사용자 소유 80
  규칙은 제거하지 않는다.
- production CLI는 변경 전 영속 파일 backup을 만들고 같은 directory의 0600 temp를
  fsync한 뒤 `os.replace()`하며 mode/owner를 보존한다. live 삽입 실패 시 영속 파일을
  원복하고, 영속 파일 교체 실패 시 새 live comment rule을 제거한다.

live rule 순서는 `iptables-save`를 파싱해 계산하며 고정 line number를 사용하지 않는다.
전체 `iptables-save` 결과를 `/etc/iptables/rules.v4`로 저장하지 않는다.

- [ ] **Step 5: 시험 및 커밋**

```bash
PYTHONPYCACHEPREFIX=/tmp/oracle-web-config \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_oracle_web_config tools.test_oracle_web_firewall -v
bash -n tools/oracle_web/bootstrap_host.sh
git diff --check
git add tools/oracle_web/render_site.py tools/oracle_web/host_firewall.py \
  tools/oracle_web/bootstrap_host.sh tools/oracle_web/templates \
  tools/test_oracle_web_config.py tools/test_oracle_web_firewall.py
git commit -m "feat: define reusable Oracle web host config"
```

---

## Task 5: SSH deploy/status/rollback wrapper를 TDD로 만든다

**Files:**
- Create: `tools/oracle_web/deploy_release.py`
- Create: `tools/oracle_web/check_status.py`
- Create: `tools/test_oracle_web_deploy.py`

- [ ] **Step 1: dry-run과 입력 경계 실패 테스트 작성**

CLI 계약:

```bash
/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release deploy \
  --target Oracle --site mobile-lab --release-id <id> --archive /tmp/mobile-lab-<id>.tar.gz
/home/light/anaconda3/bin/python -m tools.oracle_web.check_status \
  --target Oracle --site-config tools/oracle_web/sites/mobile-lab.json
/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release rollback \
  --target Oracle --site mobile-lab --release-id <previous-id>
```

`--dry-run`은 실행 없이 argv를 JSON lines로 출력한다. 시험은 다음을 검증한다.

- target/site/release/archive를 strict validation하고 archive의 `release.json` site/release와
  CLI가 일치하는지 확인한다.
- deploy는 local SHA-256을 계산하고 protected staging directory 생성 SSH 한 번, SCP 한
  번, fixed helper activation SSH 한 번, temp archive 정리 SSH 한 번만 구성한다.
- remote temp는 `/var/tmp/zetin-web-staging/<site>/<release>.tar.gz` 형식이고 shell
  metacharacter가 없다. staging directory는 deploy SSH 사용자 소유 mode 0700이다.
- activation 성공 전 wrapper 자체가 systemd/nginx를 건드리지 않으며, 검증·전환·health·
  automatic rollback 책임은 fixed root helper 안에 있다.
- status와 rollback은 fixed root helper·고정 read-only probe 외 임의 remote command를
  받지 않는다.
- subprocess 실패 exit code를 그대로 비정상 결과로 전달한다.
- `check_status.py`는 current release, Nginx active, 선택 backend active, loopback API,
  remote local-SNI HTTPS, local public-IP HTTPS와 8000/8443 negative probe를 구분해
  machine-readable 결과로 보고한다. body·닉네임·private key path는 출력하지 않는다.

- [ ] **Step 2: subprocess argv-only wrapper 구현**

shell=True, 사용자 제공 remote command, broad glob, remote `rm -rf`를 사용하지 않는다.
deploy 성공 후에만 명시 remote tar를 `/usr/bin/rm -- <validated-path>`로 지우고 빈 site
staging directory는 `/usr/bin/rmdir -- <validated-dir>`로 시도한다. helper의 JSON
결과에서 `score_reset`을 그대로 표시한다.

- [ ] **Step 3: 시험 및 커밋**

```bash
PYTHONPYCACHEPREFIX=/tmp/oracle-web-deploy \
  /home/light/anaconda3/bin/python -m unittest tools.test_oracle_web_deploy -v
git diff --check
git add tools/oracle_web/deploy_release.py tools/oracle_web/check_status.py \
  tools/test_oracle_web_deploy.py
git commit -m "feat: deploy Oracle web releases over SSH"
```

---

## Task 6: 재사용 운영 문서와 모바일 랩 runbook을 완성한다

**Files:**
- Create: `docs/oracle_web_hosting.md`
- Modify: `docs/README.md`
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/README.md`
- Modify: `docs/presentations/ai-startup-camp-drone/mobile-lab/presenter.html`
- Modify: `tools/check_repo_layout.py`
- Modify: `tools/test_mobile_lab_browser.py`
- Modify: `tools/test_repo_layout.py`

- [ ] **Step 1: 공통 운영 문서 작성**

`docs/oracle_web_hosting.md`에 다음을 실제 명령으로 기록한다.

- 서버 topology와 정적 전용/API 사이트 선택 기준
- 최초 bootstrap, 인증서 보호 전송, site config 설치, 방화벽 80/443 확인
- 새 사이트 naming/port 배정, release build/deploy/status/rollback
- `nginx -t`, systemd status/security, listener, loopback/external health 검사
- HTTP-01 Certbot 전환과 `certbot renew --dry-run`, timer 확인
- score API가 메모리 단일 프로세스이고 restart 시 초기화되는 의미
- 실패 시 current rollback, Nginx config rollback, firewall backup 복원
- 기존 tunnel 보존 항목과 금지된 broad firewall/recursive delete 명령
- OCI ingress와 DNS는 별도 계층이며 각각 확인해야 한다는 절차
- 기본 access log는 개인정보 최소화를 위해 꺼 두며, 일시 진단 로그를 켜야 할 때는
  참가자 사전 공지, 최대 보존 기간, 행사 후 명시 삭제와 삭제 확인 절차
- 다음 행사 체크리스트와 확인하지 않은 reboot/실기기/행사 Wi-Fi 경계

- [ ] **Step 2: 모바일 랩 README를 공개 URL 기준으로 갱신**

학생 `https://uos-drone.kro.kr/`, 발표자 `/presenter.html`, 선택 API `/api/scores`를 첫 경로로 추가하고 공통 운영 문서에 상대 링크한다. 기존 LAN HTTP/8443 절차는 비상·로컬 리허설로 유지하며 공개 production 권장으로 표현하지 않는다. 500 고유 제출 상한, 서버 재시작 시 순위 초기화, 로컬 결과 유지, 실제 기체 비연결을 명시한다.

앱의 `presenter.html`에는 순위가 클라이언트 제출값을 모은 교육용 비공식 결과이며
실제 비행 성능·검증값이 아니라는 짧은 안내를 추가한다. 먼저 Chrome presenter test가
이 문구를 실제 렌더링된 text로 찾지 못하는 RED를 확인하고, 기존 layout을 바꾸지 않는
copy만 추가한다.

- [ ] **Step 3: 링크·diff 검사 및 커밋**

먼저 fixture repo의 `docs/oracle_web_hosting.md`와 모바일 README에 깨진 링크를 넣었을
때 maintained Markdown scan이 오류를 반환하는 RED test를 추가한다. 그 다음
`docs/README.md`에 공통 운영 문서 링크를 추가하고 `maintained_markdown_files()`가 새
공통 문서와 모바일 README를 검사하도록 확장한다.

```bash
/home/light/anaconda3/bin/python -m unittest tools.test_repo_layout -v
MOBILE_LAB_SCREENSHOT_DIR=/tmp/oracle-doc-browser \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_mobile_lab_browser.MobileLabBrowserTests.test_presenter_renders_total_count_and_ordered_scores_from_product_server -v
/home/light/anaconda3/bin/python tools/check_repo_layout.py
git diff --check
git add docs/oracle_web_hosting.md docs/README.md \
  docs/presentations/ai-startup-camp-drone/mobile-lab/README.md \
  docs/presentations/ai-startup-camp-drone/mobile-lab/presenter.html \
  tools/check_repo_layout.py tools/test_mobile_lab_browser.py tools/test_repo_layout.py
git commit -m "docs: add reusable Oracle web hosting runbook"
```

---

## Task 7: 전체 로컬 검증과 release 동결

**Files:**
- Verify only; no new product files expected.

- [ ] **Step 1: 관련 전체 자동 시험을 fresh 실행**

```bash
node --test docs/presentations/ai-startup-camp-drone/mobile-lab/tests/*.test.mjs
PYTHONPYCACHEPREFIX=/tmp/mobile-lab-final-server \
  /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_server -v
PYTHONPYCACHEPREFIX=/tmp/mobile-lab-final-browser \
  /home/light/anaconda3/bin/python -m unittest tools.test_mobile_lab_browser -v
PYTHONPYCACHEPREFIX=/tmp/oracle-web-final \
  /home/light/anaconda3/bin/python -m unittest \
  tools.test_oracle_web_release tools.test_oracle_web_host_release \
  tools.test_oracle_web_config tools.test_oracle_web_firewall \
  tools.test_oracle_web_deploy tools.test_repo_layout -v
bash -n tools/oracle_web/bootstrap_host.sh
/home/light/anaconda3/bin/python tools/check_repo_layout.py
git diff --check
```

- [ ] **Step 2: 금지 경계와 변경 목록 검사**

```bash
rg -n "WebUSB|WebBluetooth|serial|UDP|arm|throttle|disarm|gain" \
  docs/presentations/ai-startup-camp-drone/mobile-lab tools/oracle_web
git status --short
git diff --name-only origin/feat/magcal-ellipsoid-fit...HEAD
git diff --name-only origin/feat/magcal-ellipsoid-fit...HEAD -- '*.pptx'
git diff --cached --name-only
```

검색 결과는 문서의 금지 설명 외 실제 제어 호출이 없어야 한다. PPTX 결과는 빈 출력이어야 하고, cached 목록에 `docs/cascade_vs_single_pid.pdf`가 없어야 한다.

- [ ] **Step 3: clean commit에서 production release 생성**

```bash
test -z "$(git status --porcelain --untracked-files=no)"
release_id=$(git rev-parse --short=12 HEAD)
/home/light/anaconda3/bin/python -m tools.oracle_web.build_release \
  --repo-root . --site-config tools/oracle_web/sites/mobile-lab.json \
  --release-id "$release_id" \
  --output "/tmp/mobile-lab-$release_id.tar.gz"
sha256sum "/tmp/mobile-lab-$release_id.tar.gz"
tar -tvzf "/tmp/mobile-lab-$release_id.tar.gz"
```

---

## Task 8: Oracle 기반을 bootstrap하고 첫 release를 배포한다

**Files:**
- Remote system paths only; never add generated secrets/config to Git.

- [ ] **Step 1: 변경 전 복구 자료와 불변 listener 캡처**

```bash
ssh -o BatchMode=yes Oracle \
  'sudo -n install -d -m 0700 /var/backups/zetin-web/pre-mobile-lab && \
   sudo -n iptables-save | sudo -n tee /var/backups/zetin-web/pre-mobile-lab/iptables.rules >/dev/null && \
   sudo -n cp -a /etc/ssh/sshd_config /etc/ssh/sshd_config.d /var/backups/zetin-web/pre-mobile-lab/ && \
   ss -lntp'
```

출력에서 기존 listener 22, 25565, 12222, 13389, loopback 18443을 기록한다. private key는 backup이나 출력에 포함하지 않는다.

- [ ] **Step 2: bootstrap bundle 전송·실행**

tracked clean commit에서 ID와 explicit staging을 만든다.

```bash
deploy_id=$(git rev-parse --short=12 HEAD)
bootstrap_local=$(mktemp -d /tmp/zetin-web-bootstrap.XXXXXX)
install -d -m 0700 "$bootstrap_local/oracle_web"
rsync -a --exclude __pycache__ tools/oracle_web/ "$bootstrap_local/oracle_web/"
bootstrap_remote="/var/tmp/zetin-web-bootstrap-$deploy_id"
ssh Oracle -- install -d -m 0700 "$bootstrap_remote"
scp -r "$bootstrap_local/oracle_web" "Oracle:$bootstrap_remote/"
ssh Oracle -- sudo -n bash "$bootstrap_remote/oracle_web/bootstrap_host.sh"
```

실행 뒤 `nginx -v`, `certbot --version`, `systemd-analyze verify /etc/systemd/system/zetin-webapp@.service`, `nginx -t`를 확인한다.

- [ ] **Step 3: 인증서와 site config를 보호 배치**

현재 로컬 Let’s Encrypt fullchain/key를 mode 0600 임시 파일로 준비한다. 원격에는 먼저
SSH deploy 사용자만 접근 가능한 0700 staging directory를 만들고 그 안으로 SCP한 뒤 다음 경로에
root 0644/0600으로 install한다. world-readable `/tmp` 파일명이나 명령 인자에 key
본문을 넣지 않으며, install 직후 보호 staging directory를 제거한다.

```text
/etc/zetin-web/tls/uos-drone.kro.kr/fullchain.pem
/etc/zetin-web/tls/uos-drone.kro.kr/privkey.pem
```

로컬 source는 다음 고정 경로이며 변수에는 key 본문이 아니라 경로만 둔다.

```bash
cert_source=/home/light/.local/share/letsencrypt/live/uos-drone.kro.kr/fullchain.pem
key_source=/home/light/.local/share/letsencrypt/live/uos-drone.kro.kr/privkey.pem
config_local=$(mktemp -d /tmp/zetin-web-site-config.XXXXXX)
/home/light/anaconda3/bin/python -m tools.oracle_web.render_site \
  --site-config tools/oracle_web/sites/mobile-lab.json \
  --certificate /etc/zetin-web/tls/uos-drone.kro.kr/fullchain.pem \
  --private-key /etc/zetin-web/tls/uos-drone.kro.kr/privkey.pem \
  --output-dir "$config_local"
cert_stage=/var/tmp/zetin-web-cert-stage
ssh Oracle -- install -d -m 0700 "$cert_stage"
scp "$cert_source" "Oracle:$cert_stage/fullchain.pem"
scp "$key_source" "Oracle:$cert_stage/privkey.pem"
scp "$config_local/mobile-lab.conf" "$config_local/mobile-lab.env" \
  "Oracle:$cert_stage/"
```

root install 명령은 staging의 regular file·mode를 확인하고 TLS target directory를
0755로 만든 뒤 certificate 0644, key 0600, Nginx config/env 0644로 배치한다. local/remote
certificate SHA-256과 `openssl x509 -checkend 86400 -noout`만 비교하며 key 본문을
출력하지 않는다. sites-enabled에는 `../sites-available/mobile-lab.conf` relative
symlink를 만든다. 배포 사이트가 검증된 뒤 Ubuntu 기본 `sites-enabled/default`
symlink만 제거한다. `nginx -t` 성공 전 reload하지 않는다.

- [ ] **Step 4: host firewall에 80만 멱등 추가**

443은 이미 최종 REJECT 앞에 있으므로 유지한다. 현재 live rules에는 WireGuard
UDP/51820이 추가되어 `/etc/iptables/rules.v4`보다 한 줄 많으므로 고정 line number와
`netfilter-persistent save`를 사용하지 않는다. 검증된 helper로 live와 영속 파일에
다음 comment-tagged 한 규칙만 semantic REJECT 직전에 삽입한다.

```text
-p tcp -m tcp --dport 80 -m comment --comment zetin-web:http -j ACCEPT
```

삽입 전후 `iptables-save`와 `/etc/iptables/rules.v4`를 비교해 tagged 80 한 줄 외 변화가
없는지 확인한다. rollback은 helper가 이 comment rule만 live/영속에서 제거한다. 다른
규칙 순서·수·listener를 변경하지 않는다. OCI ingress는 이 단계와 별개다.

```bash
ssh Oracle -- sudo -n /usr/local/sbin/zetin-web-firewall ensure-http
```

- [ ] **Step 5: Nginx를 준비하고 release를 활성화**

site config의 `nginx -t`가 성공한 뒤 Nginx를 먼저 enable/start하고 모바일 랩 service
instance는 enable만 한다. 아직 current release가 없으므로 service를 직접 start하지
않는다.

```bash
ssh Oracle 'sudo -n nginx -t && sudo -n systemctl enable --now nginx'
ssh Oracle 'sudo -n systemctl enable zetin-webapp@mobile-lab.service'
```

그 다음 Task 7 archive를 deploy wrapper로 전송·활성화한다. root activation helper가
current를 전환하고 첫 backend를 restart한 뒤 Nginx reload와 health, 실패 시 rollback을
책임진다. systemd가 `127.0.0.1:18080`, Nginx가 80/443에만 listen하는지 확인하고 공개
18080/8000/8443에 새 listener를 만들지 않았음을 확인한다.

```bash
release_id=$(git rev-parse --short=12 HEAD)
release_archive="/tmp/mobile-lab-$release_id.tar.gz"
/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release deploy \
  --target Oracle --site mobile-lab --release-id "$release_id" \
  --archive "$release_archive"
```

- [ ] **Step 6: DNS 전 기능·rollback integration 검증**

원격 localhost와 로컬 SSH port forward를 사용해 학생, 발표자, JS, 글꼴, GET/POST API,
invalid body/method, 보안 헤더, 50 동시 제출을 검사한다.

자동 rollback 실서버 drill은 DNS cutover 전에 별도 임시 local Git clone에서만 만든다.
현재 HEAD를 `/tmp` clone하고 그 clone의 `server.py` 시작부에 즉시 비정상 종료를
`apply_patch`로 넣어 commit한 다음 같은 manifest로 `rollback-drill-<sha>` release를
만든다. 이를 deploy하면 helper가 current를 canary로 전환하고 backend hash 변경을
감지해 restart한 뒤 loopback health 실패를 보아야 한다. deploy exit는 비정상이어야
하며, 직후 다음을 모두 확인한다.

- current symlink가 원래 production release ID로 복구됨
- production backend가 active이고 `/api/scores` 200
- Nginx local-SNI HTTPS health 200
- 실패 canary release는 immutable directory로 남고 자동 삭제되지 않음
- helper stderr/JSON이 원래 health 실패와 rollback 성공을 함께 보고함

그 다음 명시 rollback subcommand로 정상 release 두 ID 사이 전환도 확인하고 원래
production ID로 복구한다. 기존 tunnel listener와 `ssh Oracle` 연결이 유지되는지 다시
확인한다.

OCI 443 판별은 Nginx가 실제 listen한 뒤 원격의 443 INPUT accept-rule packet counter를
기록하고, 로컬에서 공인 IP에 TLS ClientHello를 보낸 다음 같은 counter를 다시 읽어
판별한다. counter가 증가하지 않고 외부 timeout이면 패킷이 host firewall에 도달하지
않은 것이므로 OCI ingress 계층 문제로 분리 보고한다. access log는 개인정보 최소화
정책대로 켜지 않는다.

---

## Task 9: 공개 cutover, 실제 HTTPS smoke, 최종 push

**Files:**
- Verify only, plus documentation correction only if observed commands differ.

- [ ] **Step 1: 필요한 외부 ingress와 DNS 최소 조치**

passive tcpdump 비교에서 같은 출발지의 TCP/25565 SYN은 VM에 도착했지만 80·443 SYN은
도착하지 않아 현재 OCI/상위 ingress 차단이 확인됐다. 구성된 OCI CLI/자격이 없으므로
서버 내부 검증 후 사용자에게 전용 stateful NSG의 IPv4 TCP 80,443 ingress 추가와 VNIC
연결을 요청한다. source는 `0.0.0.0/0`, destination port는 각각 80과 443이며 기존
Security List를 교체하지 않는다. 외부 IP health가 통과한 뒤 사용자가 A record를
`140.83.83.165`로 바꾸도록 요청한다. 검증되지 않은 AAAA는 추가하지 않는다.

- [ ] **Step 2: public DNS와 HTTPS 검증**

Google·Cloudflare DoH 양쪽의 A가 `140.83.83.165`로 일치한 뒤 다음을 검사한다.

```bash
curl --fail --silent --show-error https://uos-drone.kro.kr/ >/dev/null
curl --fail --silent --show-error https://uos-drone.kro.kr/presenter.html >/dev/null
curl --fail --silent --show-error https://uos-drone.kro.kr/api/scores
curl --fail --silent --show-error --head https://uos-drone.kro.kr/src/app.mjs
curl --fail --silent --show-error --head \
  https://uos-drone.kro.kr/vendor/uos-slide-template/fonts/NotoSansCJKkr-Regular.woff2
```

50 concurrent static GET과 50 unique score POST를 공개 URL로 실행하고 count/top score를 확인한다. 이는 50대 실제 Wi-Fi 검증이 아니라 host-side HTTP concurrency smoke라고 기록한다.

- [ ] **Step 3: 실제 Chrome 렌더링 재검증**

공개 URL을 desktop Chrome에서 360×800, 390×844로 렌더링해 horizontal overflow와 CTA clipping, 발표자 QR URL을 확인한다. synthetic DeviceOrientation은 API 흐름만 검증하며 실제 iOS/Android 센서 축·권한 proof로 표현하지 않는다.

- [ ] **Step 4: HTTP-01 renewal 준비 상태 확인**

공개 80이 도달한 뒤 Certbot HTTP-01 lineage를 만들거나 기존 bootstrap 인증서 만료 전에 전환한다. `kro.kr` 공유 rate limit을 확인하고 불필요한 강제 재발급은 하지 않는다. managed lineage가 준비된 경우에만 `certbot renew --dry-run`과 timer를 통과했다고 보고한다. 자동 renewal이 준비되지 않으면 Git push는 가능하지만 “재사용 호스트 기반 완료”로 표시하지 않고, 만료일 2026-11-07 전 반드시 해결할 미완료 운영 항목으로 명시한다.

- [ ] **Step 5: 최종 fresh verification, exact staging, push**

Task 7 전체 명령을 새로 재실행한다. 실제 변경 목록과 remote deploy source commit을 대조한 뒤 이번 작업 파일만 stage/commit한다. 사용자 PDF와 PPTX가 cached에 없음을 다시 확인한다.

```bash
git status --short
git diff --cached --name-only
git push origin feat/magcal-ellipsoid-fit
git rev-parse HEAD
git rev-parse origin/feat/magcal-ellipsoid-fit
git rev-list --left-right --count \
  HEAD...origin/feat/magcal-ellipsoid-fit
```

마지막 divergence는 정확히 `0 0`이어야 한다. 실제 모바일 기기, 행사 Wi-Fi/LTE, Oracle reboot를 실행하지 않았다면 명시적으로 현장 확인사항에 남긴다.
