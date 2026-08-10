# Oracle 재사용 웹 배포 환경 설계

## 승인과 목표

사용자는 2026-08-10에 개인 Oracle 서버를 앞으로의 간단한 웹 실습·시연 배포에도
재사용할 수 있도록 환경을 구성하고 문서화하는 방향을 승인했다. 첫 적용 대상은 AI
창업 캠프 모바일 드론 랩이다.

이 설계의 목표는 다음과 같다.

- 서로 다른 Wi-Fi와 이동통신망의 약 50명이 하나의 공개 HTTPS 주소로 접속한다.
- 정적 전용 사이트와 정적 자산 + 작은 Python API 사이트를 같은 호스트 운영 규칙으로
  배포한다.
- 새 배포와 롤백이 원자적이고, 사이트별 파일·서비스·포트가 명확하다.
- TLS, systemd, Nginx, 방화벽, 검증 절차를 한 번 마련해 다음 행사에서도 재사용한다.
- 전체 저장소가 아니라 공개가 허용된 산출물만 서버로 전송한다.
- 학생 브라우저와 공개 서버에 실제 드론·펌웨어·지상국 제어 경로를 만들지 않는다.

## 확인된 Oracle 기준 상태

읽기 전용 감사에서 다음을 확인했다.

- 접속: SSH 별칭 `Oracle`, 원격 사용자 `ubuntu`, SSH 포트 `25565`
- 운영체제: Ubuntu 24.04.4 LTS, aarch64, systemd
- 자원: 4 vCPU, RAM 23 GiB, 루트 파일시스템 가용 약 14 GiB
- 런타임: Python 3.12.3, Git과 rsync 설치됨
- 웹 계층: Nginx, Caddy, Apache, Certbot 및 배포용 인증서 없음
- 공개 웹 포트: 80과 443에 리스너가 없고 외부 TCP 연결은 timeout
- 호스트 방화벽: iptables-nft + netfilter-persistent 사용, 443 허용 규칙은 있으나
  80 허용 규칙은 없음
- 기존 인프라: `12222`, `13389`는 `rdptun`, `127.0.0.1:18443`은
  `gate1tun`, `22`와 `25565`는 SSH에 사용 중
- sudo: 현재 `ubuntu` 사용자는 비대화형 sudo 사용 가능

OCI Network Security Group 또는 Security List 상태는 SSH만으로 확인하지 않았다.
외부 80·443 timeout은 OCI 제어면과 호스트 방화벽을 모두 확인해야 한다는 뜻이지,
어느 한쪽만 원인이라고 확정한 결과가 아니다.

## 검토한 접근

### 채택: Nginx 정적 제공 + 선택적 loopback Python API

Nginx가 공개 80·443, TLS 종료, 정적 자산, 캐시, 연결 시간 제한, 요청 크기 제한과
API 속도 제한을 담당한다. Python API는 필요한 사이트에서만 단일 프로세스로
`127.0.0.1`의 비공개 고포트에 바인딩한다.

이 구성이 정적 전용 사이트와 작은 API 사이트를 모두 수용하면서 Python 표준
라이브러리 서버를 인터넷에 직접 노출하지 않는 최소 구조다.

```text
모바일 브라우저
  → DNS
  → OCI 80/443 ingress
  → 호스트 방화벽
  → Nginx
      ├─ 정적 public/ 파일
      └─ 선택 경로 /api/... → 127.0.0.1:<site-port> 단일 systemd 서비스
```

### 대안: 정적 전용 Nginx

API와 통합 순위가 필요 없는 사이트에는 이 모드를 사용한다. systemd 앱 서비스와
loopback 포트가 없으므로 가장 안전하고 복구가 쉽다. 모바일 드론 랩도 개인 결과와
터치·IMU 체험만 필요하다면 이 모드로 완전히 동작한다.

### 기각: Python 직접 TLS 공개

구성 요소는 적지만 TLS handshake, 느린 클라이언트, 요청별 스레드, 캐시, 속도 제한,
인증서 갱신을 앱 서버가 직접 감당한다. 로컬 리허설에는 유지하되 Oracle 공개
운영 경로로 사용하지 않는다.

### 보류: GitHub Pages + 별도 Oracle API

정적 자산 가용성은 높지만 CORS, API 주소 설정, 두 개의 배포 경로와 두 개의 장애
영역이 생긴다. Oracle 단일 출처가 충분한 현재 행사에는 추가하지 않는다. 나중에
정적 백업이 실제로 필요할 때 별도 설계한다.

## 재사용 호스트 기반

### 한 번 설치할 구성 요소

- Ubuntu 저장소의 Nginx
- Ubuntu 패키지 `certbot`과 `python3-certbot-nginx`
- rsync, tar, sha256sum, curl
- Nginx 공통 보안 헤더·정적 캐시·API 제한 snippet
- 선택적 Python 앱을 위한 hardened systemd unit template
- 원자적 release 활성화·롤백과 읽기 전용 상태 확인 helper

새로운 UFW를 활성화하지 않는다. 이미 iptables-nft와 netfilter-persistent 및 여러
reverse-SSH 서비스가 있으므로 기존 규칙 집합에 최소 80·443 규칙만 정확한 순서로
추가하고 재부팅 후에도 보존되는지 확인한다.

### 사이트 디렉터리 계약

```text
/srv/zetin-web/apps/<site>/
├── releases/
│   └── <release-id>/
│       ├── public/       # Nginx가 읽는 allowlist 정적 산출물
│       ├── backend/      # 선택적 loopback 앱
│       └── run           # 선택적, root 소유 executable launcher
└── current -> releases/<release-id>

/var/lib/zetin-web/<site>/             # 승인된 영속 상태가 있을 때만
/etc/nginx/sites-available/<site>.conf
/etc/nginx/sites-enabled/<site>.conf
/etc/systemd/system/zetin-webapp@.service
/etc/zetin-web/<site>.env              # 비밀값이 아닌 포트·운영 설정
```

release 디렉터리는 배포 뒤 수정하지 않고 root 소유로 둔다. `current`만 같은
파일시스템에서 원자적으로 교체한다. helper는 `/srv/zetin-web/apps/<site>` 아래의
검증된 명시 경로만 다루며 넓은 경로, 해석되지 않은 변수 또는 재귀 삭제를 사용하지
않는다.

### systemd 계약

정적 전용 사이트에는 앱 unit을 만들지 않는다. API가 있는 사이트는
`zetin-webapp@<site>.service` 인스턴스 하나만 사용한다.

- release의 root 소유 `run` launcher를 실행한다.
- `DynamicUser=yes`로 영구 로그인 계정을 만들지 않고, release 파일은 root 소유
  0555/0444 권한으로 읽기만 허용한다.
- 외부 인터페이스가 아닌 `127.0.0.1:<site-port>`에만 바인딩한다.
- `Restart=on-failure`, `NoNewPrivileges`, `PrivateTmp`, `PrivateDevices`,
  `ProtectSystem`, `ProtectHome`, capability 제거와 task/memory 한도를 적용한다.
- 영속 데이터가 승인된 앱만 `StateDirectory=zetin-web/<site>`로 만든
  `/var/lib/zetin-web/<site>` 쓰기를 허용한다.
- 여러 프로세스가 메모리 상태를 분할할 수 있는 앱은 worker 수를 늘리지 않는다.

다른 런타임이 필요한 미래 앱은 임의 문자열 명령을 env 파일에서 실행하지 않는다.
검토된 `run` launcher 또는 전용 unit을 새 release와 함께 추가한다.

### Nginx 사이트 계약

- HTTP 80은 HTTPS로 리다이렉트하고 실제 콘텐츠는 443에서만 제공한다.
- 사이트별 `server_name`, 정적 root, API exact path와 loopback upstream을 명시한다.
- 디렉터리 목록, 임의 proxy path, 넓은 alias를 허용하지 않는다.
- HTML은 행사 중 즉시 갱신할 수 있도록 짧게 캐시하고, 해시되지 않은 JS/CSS도
  과도하게 장기 캐시하지 않는다. 큰 불변 글꼴은 하루 캐시한다.
- `Permissions-Policy`의 accelerometer·gyroscope self 허용을 유지하고,
  `nosniff`, referrer 제한, frame 차단과 앱에 맞는 CSP를 적용한다.
- API request body, method, connect/read/send timeout, connection 수와 rate를 제한한다.
- 같은 행사 Wi-Fi의 50대가 하나의 NAT IP로 보일 수 있으므로 정상 burst를 막는 작은
  per-IP 제한을 사용하지 않는다.
- 학생 개인정보를 수집하지 않는 정책과 맞추기 위해 이 사이트의 access log는
  기본 비활성화한다. 일시 진단 로그가 필요하면 사전 공지, 제한된 보존 기간과 삭제
  절차를 문서화한다.

## TLS, DNS와 네트워크 변경 순서

모바일 드론 랩의 공개 이름은 `uos-drone.kro.kr`이고 Oracle 공인 IPv4는
`140.83.83.165`다. 현재 A 레코드 `192.168.0.6`은 로컬 리허설 전용이다.

1. OCI NSG/Security List에 IPv4 TCP 80·443 ingress를 추가한다.
2. 기존 host firewall의 reverse-SSH와 SSH 규칙을 보존한 채 TCP 80·443을 최종
   REJECT 앞에서 허용하고 netfilter-persistent에 저장한다.
3. 앱, Nginx와 TLS 파일을 DNS 변경 전에 구성한다.
4. 현재 발급된 `uos-drone.kro.kr` 인증서와 개인키를 SSH로 보호 전송해 bootstrap에
   사용한다. 저장소, release tarball, 로그 또는 world-readable 경로에 넣지 않는다.
5. `curl --resolve uos-drone.kro.kr:443:140.83.83.165`로 인증서, 정적 파일,
   API와 보안 헤더를 검증한다.
6. 검증 뒤 A 레코드를 `140.83.83.165`로 바꾼다. 검증된 IPv6 경로가 없으므로 AAAA는
   만들지 않는다.
7. 공개 DNS 전파 뒤 Wi-Fi와 LTE에서 접속한다.
8. 장기 운영 전 HTTP-01 자동 갱신을 구성하고 dry-run을 통과시킨다. 기존 수동
   DNS-01 인증서 만료에 의존하지 않는다.

현재 인증서는 2026-11-07까지 유효하다. `kro.kr` 공유 발급 제한 때문에 bootstrap
시점에 불필요한 재발급을 반복하지 않는다. DNS-01에 사용한 TXT는 발급 검증이 끝난
뒤 제거할 수 있다.

기존 `22`, `12222`, `13389`, `127.0.0.1:18443`, `25565` listener와 관련 SSH Match
블록, 계정, authorized_keys, GatewayPorts 정책은 이 작업에서 수정하지 않는다.

## 재사용 배포 도구와 문서

저장소에는 다음을 버전 관리한다.

- `tools/oracle_web/`: release 제작·전송·활성화·상태 확인 helper와 Nginx/systemd
  template
- `docs/oracle_web_hosting.md`: 새 정적 사이트와 Python API 사이트를 추가하는 운영
  가이드, 인증서, DNS, 방화벽, 배포, 롤백, 장애 대응
- 모바일 랩 README의 Oracle 행사 배포 절차와 위 공통 문서 링크

helper의 책임은 다음으로 제한한다.

1. 명시한 allowlist 파일로 release를 만든다.
2. release ID와 SHA-256을 출력하고 확인한다.
3. SSH 별칭과 사이트 이름을 명시적으로 받아 원격 staging 디렉터리로 전송한다.
4. 원격에서 새 release 경로·소유권·필수 파일을 확인한다.
5. `current.next`를 만든 뒤 원자적으로 `current`로 바꾼다.
6. `nginx -t`, 선택적 systemd restart, loopback health와 외부 HTTPS health를 확인한다.
7. 실패 시 이전 symlink를 복구하고 결과를 비정상 종료 코드로 반환한다.

앱별 파일 배치와 실행 명령은 manifest 또는 앱별 얇은 wrapper에 명시한다. helper가
저장소 전체를 추론하거나 모든 프로젝트를 자동으로 공개하지 않는다.

## 모바일 드론 랩 첫 적용

### 공개 파일 allowlist

`public/`에는 다음만 배치한다.

- `index.html`, `presenter.html`, `styles.css`
- `src/*.mjs`
- `vendor/qrcode-generator/qrcode.js`와 라이선스·출처 문서
- Noto Sans CJK KR Regular/Medium/Bold WOFF2 세 파일

파일 배치는 앱을 도메인 root에서 제공하도록 정규화한다.

```text
https://uos-drone.kro.kr/
https://uos-drone.kro.kr/presenter.html
https://uos-drone.kro.kr/src/...
https://uos-drone.kro.kr/vendor/qrcode-generator/...
https://uos-drone.kro.kr/vendor/uos-slide-template/fonts/...
```

펌웨어, `scripts/control_dualsense.py`, 로그, 발표 HTML/PPTX 원본, Git 메타데이터,
테스트 인증서·개인키와 사용자 PDF는 배포물에 포함하지 않는다.

### 점수 API

- `server.py`는 Nginx가 exact `/api/scores`에 전달하는 요청만 loopback에서 처리한다.
- `ScoreStore`가 프로세스 메모리 기반이므로 backend는 정확히 한 프로세스다.
- 재시작·backend 롤백 시 순위는 0으로 초기화되지만 각 학생의 화면 결과는 유지된다.
- 공개 고유 제출 누적으로 메모리가 무한 증가하지 않도록 최대 500개의 서로 다른
  제출을 받는다. 같은 ID의 동일 retry는 상한과 무관하게 idempotent 응답한다.
- 상한 뒤 신규 제출은 명시적인 503과 안정된 JSON 오류를 반환하고 기존 순위를
  유지한다. 클라이언트는 이를 선택 점수 서버 미연결과 같은 로컬 결과 보존 경로로
  처리한다.
- 점수는 클라이언트가 계산하므로 조작 방지 순위가 아니다. 발표자는 이를 교육용
  비공식 결과로 안내한다.
- 데이터베이스, 계정, 로그인, 장기 보존은 추가하지 않는다.

Nginx rate burst는 같은 NAT 뒤 50명의 동시 제출을 받아야 하므로 최소 100으로 잡는다.
본문은 현재 애플리케이션 계약인 4096 bytes로 제한한다.

## 실패 처리와 롤백

- 정적 release 활성화는 Python을 재시작하지 않으므로 기존 점수를 유지한다.
- backend 변경 또는 launcher 변경 때만 서비스를 재시작하며, 행사 중에는 배포하지
  않는다.
- 새 release health 실패 시 `current`를 이전 release로 되돌리고 Nginx를 reload한다.
- backend 롤백에 재시작이 필요하면 순위 초기화를 운영자에게 명확히 알린다.
- API 장애 때 학생 페이지의 로컬 결과와 재도전은 유지되지만, Oracle/Nginx 전체 장애는
  신규 페이지 로드를 막는다. 현장 노트북 HTTPS와 터치 전용 HTTP 주소를 비상 절차로
  문서화한다.
- DNS rollback은 이전 사설 주소로 즉시 되돌리는 방식이 아니다. 공개 장애가 길어지면
  사전 준비한 대체 QR/URL을 안내하고 DNS TTL만 믿지 않는다.

## 검증 계약

### 로컬 release 전

- Node 단위 테스트 전체
- Python 서버·브라우저 테스트 전체
- TLS idle-client 회귀 테스트
- 500개 제출 상한과 idempotent retry 테스트
- `tools/check_repo_layout.py`
- `git diff --check`
- 금지된 실제 드론 통신 API와 배포 allowlist 검사
- 사용자 PDF 및 기존 발표 HTML/PPTX 비포함 확인

### Oracle 구성 후 DNS 전

- Nginx config test와 systemd unit 보안 속성 확인
- 공개 80·443, loopback backend, 기존 reverse-SSH listener 유지 확인
- 8000·8443이 외부에서 닫혀 있는지 확인
- `curl --resolve`로 학생·발표자·폰트·JS·API·인증서·보안 헤더 확인
- 잘못된 API 요청, 큰 본문, 허용하지 않은 method와 제출 상한 거부 확인
- 새 release 활성화 실패와 이전 release rollback 연습
- 재부팅 뒤 Nginx, 선택 API, 방화벽과 기존 reverse tunnel 복구 확인

### 공개 DNS 후

- Google과 Cloudflare 공개 DNS에서 A 레코드 일치 확인
- LTE와 행사 Wi-Fi에서 공개 HTTPS 접속
- 50개 동시 정적 fetch와 50개 동시 고유 제출 smoke test
- 발표자 제출 수·상위 점수와 서버 미연결 상태 확인
- 실제 iOS Safari·Android Chrome의 권한, 중립 보정, 물리 센서 축 확인

자동 테스트, 합성 센서 이벤트와 서버 부하는 실제 행사장 Wi-Fi, 실제 폰 센서 또는
실제 비행 검증이 아니다.

## 비목표

- 범용 PaaS, 컨테이너 오케스트레이션, Kubernetes, PostgreSQL 클러스터
- 사용자 계정, 로그인, 개인정보 저장, 장기 점수 보존
- 임의 저장소 자동 공개 또는 서버에서 Git credential 보관
- 실제 드론 arm, throttle, disarm, gain, UDP, serial, WebUSB, WebBluetooth 제어
- 기존 Oracle reverse-SSH·Samba·Slurm 인프라 재구성
- 이번 승인 없이 GitHub Pages, Cloudflare 계정 또는 다른 외부 서비스를 생성하는 작업
