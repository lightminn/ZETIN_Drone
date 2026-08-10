# Oracle 재사용 웹 호스팅 운영 가이드

이 문서는 개인 Oracle VM에 작은 정적 웹앱과 선택적 Python API를 반복 배포하는
운영 절차다. 첫 사이트는 AI 창업 캠프 모바일 드론 랩이다. 앱별 파일은 manifest의
allowlist만 release에 들어가며 저장소 전체를 서버에 복사하지 않는다.

모바일 랩은 [학생·발표자 운영 가이드](presentations/ai-startup-camp-drone/mobile-lab/README.md)도
함께 따른다. 이 앱은 교육용 시뮬레이션이며 실제 기체·펌웨어·지상국과 연결되지 않는다.

## 구성과 운영 모드

```text
모바일 브라우저
  → DNS
  → OCI NSG 또는 Security List
  → VM 호스트 방화벽
  → Nginx 80/443 + TLS
      ├─ immutable release의 public/ 정적 파일
      └─ 선택: exact API path → 127.0.0.1:<site-port> systemd 단일 프로세스
```

- **정적 전용**: 개인별 브라우저 결과만 필요할 때 쓴다. manifest에서 `backend`와
  `run`을 모두 생략하고 앱 systemd 인스턴스를 만들지 않는다.
- **API 포함**: 통합 순위처럼 작은 서버 기능이 필요할 때만 쓴다. manifest의 고유
  loopback port, `backend.health_path`, allowlist의 `backend/`와 검토된 `run` launcher를
  함께 둔다. launcher는 반드시 `127.0.0.1`에만 bind한다.

두 모드 모두 학생 50명의 자세·물리 시뮬레이션과 점수 계산은 각 브라우저에서
실행한다. 서버는 정적 파일과 선택 점수 집계만 맡는다. 현재 `render_site`와 제공된
Nginx 템플릿은 backend가 있는 사이트용이다. 정적 전용 새 사이트는 release 도구는
그대로 쓰되, 그 사이트용 정적 Nginx 설정을 별도 검토·설치한 뒤 배포한다.

## 공통 변수와 안전 원칙

저장소 루트에서 실제 값이 맞는지 확인한 뒤 변수를 설정한다.

```bash
repo_root=$(git rev-parse --show-toplevel)
cd "$repo_root"
ssh_target=Oracle
site=mobile-lab
domain=uos-drone.kro.kr
public_ipv4=140.83.83.165
site_config=tools/oracle_web/sites/mobile-lab.json
```

다음 원칙은 모든 사이트에 적용한다.

- 변경 전 listener, live firewall, 영속 firewall, SSH/Nginx 설정을 별도 backup에
  남긴다. 개인키는 backup 출력이나 Git에 넣지 않는다.
- 기존 `22`, `25565`, `12222`, `13389`, loopback `18443` listener를 보존한다.
  `rdptun`, `gate1tun`, SSH Match/키/포트는 이 절차에서 바꾸지 않는다.
- UFW를 새로 켜지 않고 방화벽 전체를 교체하지 않는다.
  `netfilter-persistent save`도 실행하지 않는다. live 규칙에만 있는 기존 규칙이
  영속 파일에서 사라질 수 있기 때문이다.
- 넓은 재귀 삭제, 해석되지 않은 경로, `rm -rf`를 쓰지 않는다. 임시 디렉터리는
  생성한 정확한 경로의 내용을 확인한 뒤 `find ... -mindepth 1 -delete`와 `rmdir`로
  지운다.
- bootstrap과 설정 변경 뒤에도 VM을 재부팅하지 않는다. reboot persistence는
  별도의 승인된 유지보수 시간에 확인한다.

## 최초 bootstrap

### 1. 변경 전 상태와 복구 자료

로컬에서 고유 backup 경로를 만들고 원격에 전달한다.

```bash
backup_stamp=$(date -u +%Y%m%dT%H%M%SZ)-$$
backup_remote="/var/backups/zetin-web/pre-$backup_stamp"
(
set -euo pipefail
ssh "$ssh_target" "test ! -e '$backup_remote' && sudo -n install -d -o root -g root -m 0700 '$backup_remote'"
ssh "$ssh_target" "sudo -n /bin/sh -c 'umask 077; /usr/sbin/iptables-save > \"\$1\"' sh '$backup_remote/iptables-save.txt'"
ssh "$ssh_target" -- sudo -n test -s "$backup_remote/iptables-save.txt"
ssh "$ssh_target" "if sudo -n test -f /etc/iptables/rules.v4; then sudo -n cp -a -- /etc/iptables/rules.v4 '$backup_remote/rules.v4'; fi"
ssh "$ssh_target" "sudo -n cp -a -- /etc/ssh/sshd_config /etc/ssh/sshd_config.d '$backup_remote/'"
ssh "$ssh_target" "if sudo -n test -d /etc/nginx; then sudo -n cp -a -- /etc/nginx '$backup_remote/nginx'; fi"
ssh "$ssh_target" -- sudo -n ss -lntup
ssh "$ssh_target" "sudo -n /bin/sh -c 'umask 077; printf \"%s\\n\" ready > \"\$1\"' sh '$backup_remote/READY'"
ssh "$ssh_target" -- sudo -n test -s "$backup_remote/READY"
)
```

출력에서 22, 25565, 12222, 13389와 `127.0.0.1:18443`이 예상대로인지 확인한다.
없는 listener가 있으면 원인을 확인할 때까지 bootstrap을 진행하지 않는다.

### 2. allowlist host package 전송

bootstrap은 패키지 설치 중 서비스 자동 시작을 막고, Nginx·Certbot·curl·rsync,
root helper, Nginx 공통 설정과 hardened systemd template를 설치한다. 기존 대상이
다르면 `/var/backups/zetin-web/` 아래에 먼저 보관한다.

```bash
deploy_id=$(git rev-parse --short=12 HEAD) &&
bootstrap_local=$(mktemp -d /tmp/zetin-web-bootstrap.XXXXXX) &&
bootstrap_remote="/var/tmp/zetin-web-bootstrap-$deploy_id" &&
(
set -euo pipefail
: "${backup_remote:?먼저 변경 전 backup 블록을 같은 shell에서 실행하십시오}"
ssh "$ssh_target" -- sudo -n test -s "$backup_remote/iptables-save.txt"
ssh "$ssh_target" -- sudo -n test -s "$backup_remote/READY"
install -d -m 0700 "$bootstrap_local/oracle_web"
git archive --format=tar HEAD:tools/oracle_web | \
  tar -x -C "$bootstrap_local/oracle_web"
test -f "$bootstrap_local/oracle_web/bootstrap_host.sh"
ssh "$ssh_target" "test ! -e '$bootstrap_remote' && install -d -m 0700 '$bootstrap_remote'"
scp -r "$bootstrap_local/oracle_web" "$ssh_target:$bootstrap_remote/"
ssh "$ssh_target" -- sudo -n bash "$bootstrap_remote/oracle_web/bootstrap_host.sh"
)
```

`git archive`는 현재 working tree가 아니라 tracked `HEAD`의
`tools/oracle_web/`만 추출한다. 따라서 그 아래의 untracked·수정 중 파일은 bundle에
들어가지 않는다. backup 파일이 없거나 비어 있거나, archive·전송이 실패하면 root
bootstrap 명령까지 진행하지 않는다.

설치 결과를 확인한다. 최초 실행 전에 `nginx.service`와 `certbot.timer`가 없었다면
두 unit은 설치 뒤에도 inactive·disabled여야 한다. bootstrap을 재실행하면 시작 시점의
active/inactive와 enabled/disabled를 각각 보존한다. 따라서 이미 active·enabled인
Nginx를 불필요하게 stop/start하거나 disable하지 않는다. 아래 상태 단정은 unit이
없던 호스트의 최초 설치에만 사용하고, 재실행에서는 실행 전 기록과 실행 후 상태를
비교한다.

```bash
ssh "$ssh_target" -- nginx -v
ssh "$ssh_target" -- certbot --version
ssh "$ssh_target" 'test "$(sudo -n systemctl show -p ActiveState --value nginx.service)" = inactive && test "$(sudo -n systemctl show -p UnitFileState --value nginx.service)" = disabled && test "$(sudo -n systemctl show -p ActiveState --value certbot.timer)" = inactive && test "$(sudo -n systemctl show -p UnitFileState --value certbot.timer)" = disabled'
ssh "$ssh_target" -- sudo -n systemd-analyze verify /etc/systemd/system/zetin-webapp@.service
ssh "$ssh_target" -- sudo -n nginx -t
```

확인 뒤 이 실행에서 만든 staging만 지운다.

```bash
ssh "$ssh_target" "find '$bootstrap_remote' -mindepth 1 -delete && rmdir '$bootstrap_remote'"
find "$bootstrap_local" -mindepth 1 -delete
rmdir "$bootstrap_local"
```

### 3. 인증서와 사이트 설정 보호 배치

기존 수동 발급 인증서는 bootstrap에만 사용할 수 있다. 개인키 본문을 변수, 명령행,
로그에 넣지 않고 경로만 지정한다. 다음 예시는 모바일 랩의 현재 로컬 파일을 원격
root 전용 경로에 복사한다.

```bash
cert_source=/home/light/.local/share/letsencrypt/live/uos-drone.kro.kr/fullchain.pem
key_source=/home/light/.local/share/letsencrypt/live/uos-drone.kro.kr/privkey.pem
cert_local=$(mktemp -d /tmp/zetin-web-cert.XXXXXX)
install -m 0600 "$cert_source" "$cert_local/fullchain.pem"
install -m 0600 "$key_source" "$cert_local/privkey.pem"

config_local=$(mktemp -d /tmp/zetin-web-site-config.XXXXXX)
cert_remote="/etc/zetin-web/tls/$domain/fullchain.pem"
key_remote="/etc/zetin-web/tls/$domain/privkey.pem"
/home/light/anaconda3/bin/python -m tools.oracle_web.render_site \
  --site-config "$site_config" \
  --certificate "$cert_remote" \
  --private-key "$key_remote" \
  --output-dir "$config_local"

cert_stage="/var/tmp/zetin-web-cert-$deploy_id"
ssh "$ssh_target" -- install -d -m 0700 "$cert_stage"
scp "$cert_local/fullchain.pem" "$cert_local/privkey.pem" \
  "$config_local/$site.conf" "$config_local/$site.env" \
  "$ssh_target:$cert_stage/"
ssh "$ssh_target" "test -f '$cert_stage/fullchain.pem' && test ! -L '$cert_stage/fullchain.pem' && test -f '$cert_stage/privkey.pem' && test ! -L '$cert_stage/privkey.pem'"
```

원격 설치 권한은 인증서 0644, 개인키 0600, 사이트 설정과 비밀값이 없는 port env는
0644다.

```bash
ssh "$ssh_target" "sudo -n install -d -o root -g root -m 0755 '/etc/zetin-web/tls/$domain' && \
sudo -n install -o root -g root -m 0644 '$cert_stage/fullchain.pem' '$cert_remote' && \
sudo -n install -o root -g root -m 0600 '$cert_stage/privkey.pem' '$key_remote' && \
sudo -n install -o root -g root -m 0644 '$cert_stage/$site.conf' '/etc/nginx/sites-available/$site.conf' && \
sudo -n install -o root -g root -m 0644 '$cert_stage/$site.env' '/etc/zetin-web/$site.env'"

sha256sum "$cert_local/fullchain.pem"
ssh "$ssh_target" -- sudo -n sha256sum "$cert_remote"
ssh "$ssh_target" -- sudo -n openssl x509 -in "$cert_remote" -checkend 86400 -noout
ssh "$ssh_target" -- sudo -n stat -c '%U:%G %a %n' "$cert_remote" "$key_remote" \
  "/etc/nginx/sites-available/$site.conf" "/etc/zetin-web/$site.env"
```

두 SHA-256이 같고 인증서 유효성 검사가 성공해야 한다. 사이트 symlink는 처음 한
번만 만들고, 이미 있다면 먼저 `readlink` 결과가 같은지 확인한다.

```bash
ssh "$ssh_target" -- sudo -n readlink "/etc/nginx/sites-enabled/$site.conf"
# 파일이 아직 없을 때만 실행
ssh "$ssh_target" -- sudo -n ln -s "../sites-available/$site.conf" "/etc/nginx/sites-enabled/$site.conf"
ssh "$ssh_target" -- sudo -n nginx -t
```

보호 staging은 명시한 네 파일만 제거한다.

```bash
ssh "$ssh_target" "rm -- '$cert_stage/fullchain.pem' '$cert_stage/privkey.pem' '$cert_stage/$site.conf' '$cert_stage/$site.env' && rmdir '$cert_stage'"
find "$cert_local" -mindepth 1 -delete && rmdir "$cert_local"
find "$config_local" -mindepth 1 -delete && rmdir "$config_local"
```

### 4. 호스트 방화벽의 tagged HTTP 규칙

호스트마다 80과 443이 최종 INPUT REJECT보다 앞에서 허용되는지 live 규칙과
`/etc/iptables/rules.v4`를 각각 확인한다. 2026-08-10에 감사한 이 Oracle 호스트는
443은 이미 허용됐고 80만 없었다. 이 결과를 다른 호스트나 미래 상태에 일반화하지
않는다. 443이 없다면 현재 helper가 관리하지 않으므로 임의 조작하지 말고 별도
검토된 변경을 준비한다.

```bash
firewall_audit=$(mktemp -d /tmp/zetin-web-firewall.XXXXXX)
(
set -euo pipefail
ssh "$ssh_target" -- sudo -n /usr/sbin/iptables-save >"$firewall_audit/live.before"
ssh "$ssh_target" -- sudo -n cat /etc/iptables/rules.v4 >"$firewall_audit/persistent.before"
test -s "$firewall_audit/live.before"
test -s "$firewall_audit/persistent.before"
ssh "$ssh_target" -- sudo -n /usr/local/sbin/zetin-web-firewall ensure-http
ssh "$ssh_target" -- sudo -n /usr/sbin/iptables-save >"$firewall_audit/live.after"
ssh "$ssh_target" -- sudo -n cat /etc/iptables/rules.v4 >"$firewall_audit/persistent.after"
test -s "$firewall_audit/live.after"
test -s "$firewall_audit/persistent.after"
diff -u "$firewall_audit/live.before" "$firewall_audit/live.after" || true
diff -u "$firewall_audit/persistent.before" "$firewall_audit/persistent.after" || true
)
```

차이는 최종 REJECT 앞의 TCP/80 ACCEPT 한 줄이어야 하며 comment는
`zetin-web:http`다. helper는 live 전체를 저장하지 않고 영속 파일의 같은 의미 한
줄만 원자적으로 바꾼다. 감사 파일을 확인한 뒤 이 디렉터리만 지운다.

```bash
find "$firewall_audit" -mindepth 1 -delete && rmdir "$firewall_audit"
```

### 5. Nginx와 앱 service 준비

첫 release 전에는 앱 unit을 직접 start하지 않는다. `current/run`이 아직 없기 때문이다.

```bash
ssh "$ssh_target" 'sudo -n nginx -t && sudo -n systemctl enable --now nginx.service'
ssh "$ssh_target" -- sudo -n systemctl enable "zetin-webapp@$site.service"
```

정적 전용 사이트는 두 번째 명령을 실행하지 않는다. release 배포와 local-SNI 검증이
끝난 뒤 Ubuntu 기본 site symlink가 남아 있을 때만 정확한 파일을 제거하고 reload한다.

```bash
ssh "$ssh_target" -- sudo -n unlink /etc/nginx/sites-enabled/default
ssh "$ssh_target" 'sudo -n nginx -t && sudo -n systemctl reload nginx.service'
```

## 새 사이트와 release 배포

### manifest와 port

- `site`는 `[a-z][a-z0-9-]{0,62}`를 만족해야 한다.
- 새 API 사이트는 다른 manifest와 `ss -lntp`를 확인해 충돌하지 않는 1024~65535
  loopback port를 배정한다. Nginx upstream, env, manifest가 같은 값을 써야 한다.
- `files`는 공개·실행에 필요한 tracked regular file만 source/destination으로
  명시한다. glob, 상위 경로, symlink, 중복 destination을 쓰지 않는다.
- 정적 전용은 `backend`와 destination `run`을 둘 다 생략한다.
- API 포함은 `backend.port`, `backend.health_path`, backend 파일과 destination
  `run`을 함께 둔다. env에서 임의 command 문자열을 실행하지 않는다.

제공된 `render_site`와 Nginx stock template는 exact `/api/scores`만 proxy한다.
따라서 stock template에서는 `backend.health_path`도 `/api/scores`여야 한다.
다른 API path는 Nginx template와 테스트를 먼저 검토·추가한 뒤 manifest에 사용한다.

현재 모바일 랩 manifest와 launcher는 다음 명령으로 검토한다.

```bash
sed -n '1,240p' tools/oracle_web/sites/mobile-lab.json
sed -n '1,120p' tools/oracle_web/sites/mobile-lab.run
ssh "$ssh_target" -- sudo -n ss -lntp
```

### 결정적 build와 dry-run

release build는 manifest와 allowlist source가 모두 tracked·clean일 때만 성공한다.
동일 commit, manifest, release ID의 tar.gz는 결정적이다.

```bash
release_id=$(git rev-parse --short=12 HEAD)
release_archive="/tmp/$site-$release_id.tar.gz"
/home/light/anaconda3/bin/python -m tools.oracle_web.build_release \
  --repo-root "$repo_root" \
  --site-config "$site_config" \
  --release-id "$release_id" \
  --output "$release_archive"
sha256sum "$release_archive"
tar -tvzf "$release_archive"

/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release deploy \
  --target "$ssh_target" \
  --site "$site" \
  --release-id "$release_id" \
  --archive "$release_archive" \
  --dry-run
```

dry-run 출력은 실행할 고정 SSH/SCP argv를 검토하기 위한 것이며 실제 전송이나
활성화를 하지 않는다.

### deploy, status, rollback

```bash
/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release deploy \
  --target "$ssh_target" \
  --site "$site" \
  --release-id "$release_id" \
  --archive "$release_archive"

/home/light/anaconda3/bin/python -m tools.oracle_web.check_status \
  --target "$ssh_target" \
  --site-config "$site_config"

ssh "$ssh_target" -- sudo -n /usr/local/sbin/zetin-web-release status --site "$site"
```

호스트는 `/srv/zetin-web/apps/<site>/releases/<release-id>`를 root 소유 읽기 전용
release로 설치하고, 상대 symlink `current`만 원자적으로 바꾼다. 활성화 전에
`nginx -t`를 실행하며, 필요한 backend restart와 Nginx reload 뒤 loopback API와
local-SNI HTTPS health를 검사한다. health가 실패하면 이전 `current`와 runtime을
자동 복구하고 배포 명령은 실패한다. 실패 release 디렉터리는 사후 분석을 위해
자동 삭제하지 않는다.

명시 rollback은 이미 설치된 release ID만 받는다.

```bash
rollback_release=REPLACE_WITH_VALID_INSTALLED_RELEASE_ID
/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release rollback \
  --target "$ssh_target" \
  --site "$site" \
  --release-id "$rollback_release"
```

JSON의 `backend_restarted`와 `score_reset`이 `true`이면 in-memory 순위가 초기화됐다는
뜻이다. 행사 중 backend 변경·rollback을 피하고, 필요하면 참가자에게 먼저 알린다.

## 배포 검증

### 원격 service와 listener

```bash
ssh "$ssh_target" -- sudo -n systemctl status nginx.service --no-pager
ssh "$ssh_target" -- sudo -n systemctl status "zetin-webapp@$site.service" --no-pager
ssh "$ssh_target" -- sudo -n systemd-analyze security "zetin-webapp@$site.service" --no-pager
ssh "$ssh_target" -- sudo -n ss -lntp
ssh "$ssh_target" -- curl --fail --silent --show-error --max-time 5 \
  "http://127.0.0.1:18080/api/scores" >/dev/null
```

Nginx만 공개 80/443에 있고 API는 `127.0.0.1:18080`이어야 한다. 기존 22, 25565,
12222, 13389, loopback 18443도 그대로여야 한다. 새 사이트에서는 18080 대신 그
manifest의 port를 쓴다. 정적 전용 사이트는 API와 앱 service 검사를 생략한다.

### TLS와 노출 범위

원격 VM 내부 local-SNI 검사는 OCI/DNS와 독립적으로 Nginx·인증서·release를 확인한다.

```bash
ssh "$ssh_target" -- curl --fail --silent --show-error --max-time 5 \
  --resolve "$domain:443:127.0.0.1" "https://$domain/" >/dev/null
ssh "$ssh_target" -- curl --fail --silent --show-error --max-time 5 \
  --resolve "$domain:443:127.0.0.1" "https://$domain/presenter.html" >/dev/null
ssh "$ssh_target" -- curl --fail --silent --show-error --max-time 5 \
  --resolve "$domain:443:127.0.0.1" "https://$domain/api/scores" >/dev/null
```

운영자 PC에서 공인 IP로 직접 검사한다. 이 검사는 DNS를 바꾸기 전에도 가능하지만
OCI 443 ingress가 열려 있어야 한다.

```bash
curl --fail --silent --show-error --max-time 10 \
  --resolve "$domain:443:$public_ipv4" "https://$domain/" >/dev/null
curl --fail --silent --show-error --max-time 10 \
  --resolve "$domain:443:$public_ipv4" "https://$domain/api/scores" >/dev/null
```

통합 status 명령은 current, Nginx, 선택 backend·loopback API, 원격 local-SNI HTTPS,
로컬 public-IP HTTPS, public 8000/8443 TCP 상태를 JSON으로 보여 준다.

```bash
/home/light/anaconda3/bin/python -m tools.oracle_web.check_status \
  --target "$ssh_target" --site-config "$site_config" | \
  /home/light/anaconda3/bin/python -m json.tool
```

`negative_ports.8000`과 `.8443`은 `closed`가 정상이다. timeout이나
`indeterminate`는 차단 계층을 더 조사해야 하며 닫힘으로 단정하지 않는다.

## OCI ingress와 DNS

OCI ingress, VM 방화벽, DNS는 서로 다른 계층이다. 한 계층의 성공을 다른 계층의
완료 증거로 쓰지 않는다. OCI Console에서 다음을 수행한다.

1. 이 VM용 **전용 stateful NSG**를 만들거나 기존 전용 NSG를 선택한다.
2. IPv4 ingress 두 개를 추가한다: source `0.0.0.0/0`, protocol TCP, destination
   port `80`; 같은 조건의 port `443`.
3. NSG를 `140.83.83.165`가 연결된 올바른 VNIC에 attach한다.
4. 기존 Security List/NSG 규칙을 교체하거나 삭제하지 않는다.

이 저장소 도구는 OCI 제어면을 변경하지 않는다. 위 작업은 실행·검증 전에는
완료됐다고 기록하지 않는다.

현재 원하는 DNS는 다음 한 줄이다.

```text
uos-drone.kro.kr.  A  140.83.83.165
```

기존 `192.168.x.x`, `10.x.x.x` 같은 LAN/private A 값은 public 운영 레코드에서
제거한다. 검증된 IPv6 경로가 없으면 AAAA를 추가하지 않는다. DNS를 바꾼 뒤 서로
독립된 resolver와 실제 Wi-Fi/LTE에서 확인한다.

```bash
dig +short A uos-drone.kro.kr @1.1.1.1
dig +short A uos-drone.kro.kr @8.8.8.8
curl --fail --silent --show-error https://uos-drone.kro.kr/ >/dev/null
curl --fail --silent --show-error https://uos-drone.kro.kr/presenter.html >/dev/null
curl --fail --silent --show-error https://uos-drone.kro.kr/api/scores >/dev/null
```

## Certbot HTTP-01 자동 갱신 전환

현재 수동 발급 인증서는 2026-11-07에 만료된다. 공개 80과 DNS가 모두 실제로
도달한 뒤에만 HTTP-01로 전환한다. `kro.kr`은 여러 사용자가 공유할 수 있어 CA rate
limit 영향을 받을 수 있으므로 시험 삼아 반복 발급하거나 `--force-renewal`을 쓰지
않는다.

먼저 Nginx 설정을 backup하고 실제 운영 이메일로 한 번 발급한다.

```bash
backup_stamp=$(date -u +%Y%m%dT%H%M%SZ)-$$ &&
acme_email=REPLACE_WITH_OPERATIONS_EMAIL &&
(
set -euo pipefail
ssh "$ssh_target" -- sudo -n test ! -e \
  "/var/backups/zetin-web/$site.conf.before-certbot-$backup_stamp"
ssh "$ssh_target" -- sudo -n cp -a \
  "/etc/nginx/sites-available/$site.conf" \
  "/var/backups/zetin-web/$site.conf.before-certbot-$backup_stamp"
ssh "$ssh_target" -- sudo -n test -s \
  "/var/backups/zetin-web/$site.conf.before-certbot-$backup_stamp"
ssh "$ssh_target" -- sudo -n certbot --nginx \
  --cert-name "$domain" -d "$domain" \
  --email "$acme_email" --agree-tos --no-eff-email
)
```

발급 뒤 config를 `/etc/letsencrypt/live/<domain>/` lineage로 다시 render하여 저장소
템플릿과 일치시키고, `nginx -t` 뒤 reload한다.

```bash
managed_config=$(mktemp -d /tmp/zetin-web-managed-cert.XXXXXX)
/home/light/anaconda3/bin/python -m tools.oracle_web.render_site \
  --site-config "$site_config" \
  --certificate "/etc/letsencrypt/live/$domain/fullchain.pem" \
  --private-key "/etc/letsencrypt/live/$domain/privkey.pem" \
  --output-dir "$managed_config"
scp "$managed_config/$site.conf" "$ssh_target:/var/tmp/$site-managed.conf"
ssh "$ssh_target" "sudo -n install -o root -g root -m 0644 '/var/tmp/$site-managed.conf' '/etc/nginx/sites-available/$site.conf' && rm -- '/var/tmp/$site-managed.conf' && sudo -n nginx -t && sudo -n systemctl reload nginx.service"
find "$managed_config" -mindepth 1 -delete && rmdir "$managed_config"
```

자동 갱신은 다음이 모두 성공해야만 준비됐다고 기록한다.

```bash
ssh "$ssh_target" -- sudo -n certbot certificates
ssh "$ssh_target" -- sudo -n certbot renew --dry-run
ssh "$ssh_target" -- sudo -n systemctl status certbot.timer --no-pager
ssh "$ssh_target" -- sudo -n systemctl list-timers certbot.timer --no-pager
```

dry-run, timer, 발급 lineage 중 하나라도 확인하지 못했으면 기존 인증서 만료 전
미완료 운영 항목으로 남긴다.

## 모바일 랩 점수 API 특성

- 점수 API는 loopback 단일 Python 프로세스의 메모리 안에서만 집계한다.
- 학생 브라우저는 로컬에만 저장하는 `익명-XXXXXXXX` 표시 이름을 기본으로 사용하며,
  같은 표시 이름의 여러 제출은 점수판에서 최고점 한 건과 고유 이름 수로 집계한다.
- 서로 다른 `submission_id`는 최대 500개다. 동일 ID·동일 내용 retry는 상한 뒤에도
  idempotent하게 기존 결과를 반환하며, 동일 ID의 다른 내용은 거부한다.
- 500개 뒤 새 제출은 503으로 거부되지만 기존 순위와 학생 화면의 로컬 결과는 남는다.
- backend restart, backend 코드가 바뀐 release, API 포함 rollback은 순위를 0으로
  초기화할 수 있다.
- 점수는 참가자 브라우저가 계산·제출한 교육용 비공식 결과다. 실제 비행 성능이나
  검증된 측정값이 아니며 부정행위 방지 순위도 아니다.
- API를 끄거나 서버 제출이 실패해도 IMU·터치 체험, 호버링, 개인 결과와 재도전은
  각 브라우저에서 완전히 동작한다.

## 장애 복구

### release/current

`current`를 손으로 바꾸지 말고 검증된 installed release로 rollback한다.

```bash
ssh "$ssh_target" -- sudo -n /usr/local/sbin/zetin-web-release status --site "$site"
/home/light/anaconda3/bin/python -m tools.oracle_web.deploy_release rollback \
  --target "$ssh_target" --site "$site" --release-id "$rollback_release"
```

배포 health 실패는 자동 rollback된다. 오류에 `rollback failed`가 함께 나오면 자동
복구도 실패한 것이므로 Nginx·systemd·`current` 상태를 읽기 전용으로 수집하고 추가
변경을 멈춘다.

### Nginx 설정

bootstrap 또는 변경 전에 만든 정확한 backup 파일을 명시해 복원한다. 다른 site를
덮어쓰거나 `/etc/nginx` 전체를 교체하지 않는다.

```bash
nginx_backup=REPLACE_WITH_EXACT_BACKUP_FILE
ssh "$ssh_target" -- sudo -n install -o root -g root -m 0644 \
  "$nginx_backup" "/etc/nginx/sites-available/$site.conf"
ssh "$ssh_target" 'sudo -n nginx -t && sudo -n systemctl reload nginx.service'
```

### tagged HTTP firewall 규칙

다음 명령은 `zetin-web:http` comment가 붙은 exact TCP/80 한 줄만 live와 영속
파일에서 제거한다. 사용자 소유의 다른 80 허용이나 tunnel 규칙은 제거하지 않는다.

```bash
ssh "$ssh_target" -- sudo -n /usr/local/sbin/zetin-web-firewall rollback-http
```

실행 전후 `iptables-save`와 `/etc/iptables/rules.v4`를 다시 비교한다. helper가 만든
`/var/backups/zetin-web/firewall/` 파일은 별도 복구 자료이며 전체 live state를
그대로 restore하는 명령으로 사용하지 않는다.

## access log 임시 진단

기본 Nginx 설정은 `access_log off`다. 임시 로그가 꼭 필요하면 수집 전에 참가자에게
목적과 항목을 알리고, 수집 시작 전에 최대 보존 종료시각을 **행사 종료 후 24시간
이내**로 기록한다. IP, query string, referrer, user-agent는 수집하지 않고 다음처럼
시간·status·method·path만 별도 exact 파일에 기록한다.

```nginx
log_format zetin_event '$time_iso8601 $status $request_method $uri';
access_log /var/log/nginx/mobile-lab-access.log zetin_event;
```

적용 전 site config를 backup하고 `nginx -t`를 통과시킨다. 진단 종료 즉시
`access_log off`로 되돌리고 reload한 뒤 exact 파일을 비우고 삭제한다.

```bash
(
set -euo pipefail
ssh "$ssh_target" 'sudo -n nginx -t && sudo -n systemctl reload nginx.service'
ssh "$ssh_target" -- sudo -n truncate -s 0 -- /var/log/nginx/mobile-lab-access.log
ssh "$ssh_target" -- sudo -n unlink -- /var/log/nginx/mobile-lab-access.log
ssh "$ssh_target" -- sudo -n test ! -e /var/log/nginx/mobile-lab-access.log
)
```

삭제 전에 `find /var/log/nginx -maxdepth 1 -type f -name
'mobile-lab-access.log*' -print`로 회전 파일을 확인한다. 발견하면 wildcard로 지우지
말고 각 확인된 exact 파일명을 `truncate`, `unlink`, `test ! -e`에 하나씩 명시한다.
삭제 검증 결과를 행사 기록에 남긴다.

## 재사용 체크리스트

- [ ] 사이트 이름, domain, public IPv4, 정적/API 모드를 정했다.
- [ ] manifest allowlist에 비밀, Git 메타데이터, 사용자 문서, 불필요한 파일이 없다.
- [ ] API가 있으면 고유 loopback port와 health path, 엄격한 `run` launcher를 검토했다.
- [ ] 변경 전 listener/firewall/설정 backup을 만들고 기존 tunnel을 기록했다.
- [ ] 인증서·개인키를 보호 staging으로 옮기고 원격 owner/mode를 확인했다.
- [ ] OCI 80/443, host 80/443, Nginx 80/443을 각각 확인했다.
- [ ] 결정적 build, dry-run, deploy, status와 이전 release rollback을 확인했다.
- [ ] local-SNI와 public-IP HTTPS, loopback API, 8000/8443 비노출을 확인했다.
- [ ] DNS A에서 private/LAN 값을 제거했고 미검증 AAAA를 만들지 않았다.
- [ ] Certbot HTTP-01 dry-run과 timer가 실제로 통과했거나 만료 전 미완료 항목으로 남겼다.
- [ ] access log는 꺼져 있거나, 사전 공지·24시간 이내 보존·exact 삭제 계획이 있다.

## 행사 전 체크리스트와 검증 경계

- [ ] 공개 학생·발표자 URL, API status와 QR 내용을 행사 전날과 당일 다시 확인했다.
- [ ] 실제 iOS Safari와 Android Chrome에서 HTTPS 권한 gesture, 중립 보정, 실제 센서
  축과 터치 대체 입력을 확인했다.
- [ ] 행사 Wi-Fi와 LTE에서 접속하고, 50대 현장 동시 접속·제출을 리허설했다.
- [ ] API 없이도 학생 로컬 결과가 남는 비상 절차와 대체 QR을 준비했다.
- [ ] 기존 SSH/tunnel listener가 유지되는지 행사 직전에 확인했다.
- [ ] 재부팅을 승인된 시간에 별도로 수행했다면 Nginx, 앱, firewall persistence와
  tunnel을 재검증했다.

자동 단위·Chrome·서버 테스트와 host-side 50개 동시 요청은 실제 iOS/Android IMU,
LTE/행사 Wi-Fi, 50대 물리 기기 부하, Oracle 재부팅 뒤 persistence를 증명하지 않는다.
그 검증을 실제로 수행하지 않았다면 체크하지 않고 현장 확인사항으로 남긴다.
