# AI 창업 캠프 모바일 드론 랩 운영 가이드

이 자료는 설치·로그인 없이 QR로 접속하는 교육용 가상 드론 실습입니다. 학생 입력은
브라우저 안의 Roll·Pitch 시뮬레이션과 20초 호버링 점수에만 쓰이며, 실제 드론·펌웨어·지상국·모터에는 연결되지 않습니다. 실제 비행 시연이 있다면 학생 실습과 분리된 진행자 전용 구역에서 진행합니다.

## 권장 공개 운영 주소

```text
학생:           https://uos-drone.kro.kr/
발표자·프로젝터: https://uos-drone.kro.kr/presenter.html
선택 점수 API:  https://uos-drone.kro.kr/api/scores
```

행사 공개 운영은 위 신뢰 HTTPS 주소를 우선 사용합니다. Oracle 호스트의 최초 구성,
release 배포, OCI ingress, DNS, TLS 갱신과 rollback은
[Oracle 재사용 웹 호스팅 운영 가이드](../../../oracle_web_hosting.md)를 따릅니다.
LAN HTTP와 직접 8443 제공은 인터넷 공개 운영 권장 경로가 아니라 비상·로컬
리허설 경로입니다.

## 화면 역할과 주소

- 학생: `https://uos-drone.kro.kr/` 또는 같은 출처의 `/index.html`
- 발표자·프로젝터: 같은 출처의 `/presenter.html`
- 선택 점수 API: 같은 출처의 `/api/scores`

발표자 페이지는 QR을 로컬에서 만들고 URL을 복사합니다. 점수 API가 없거나 중단되어도 QR과 학생 실습은 계속 사용할 수 있습니다.

`<LAN-IP>`는 진행자 노트북의 행사 Wi-Fi IPv4 주소, `<trusted-name>`은 그 IP가 아닌 행사에서 신뢰되는 HTTPS 호스트 이름으로 바꿉니다. QR에는 `127.0.0.1`이나 `localhost`를 넣지 않습니다. 그것들은 진행자 자신의 장치만 가리킵니다.

## 비상·로컬 리허설: HTTP 터치 미리보기

HTTP는 레이아웃·QR·터치 조이스틱·가상 호버링을 확인하는 미리보기 용도입니다. 앱은 HTTP에서 센서가 제한됨을 알리고 터치 경로를 제공합니다. 실제 휴대폰 기울임·권한 리허설에는 다음 HTTPS 절차를 사용합니다.

저장소 루트에서 실행합니다.

```bash
/home/light/anaconda3/bin/python -m http.server 8000 \
  --directory docs/presentations/ai-startup-camp-drone/mobile-lab
```

진행자 노트북에서는 다음을 엽니다.

```text
학생:    http://127.0.0.1:8000/
발표자:  http://127.0.0.1:8000/presenter.html
```

같은 Wi-Fi의 휴대폰에는 노트북의 실제 LAN 주소를 사용합니다.

```text
학생:    http://<LAN-IP>:8000/
발표자:  http://<LAN-IP>:8000/presenter.html
```

이 정적 HTTP 미리보기에는 점수 API가 없습니다. 발표자 화면의 `점수판은 선택 기능입니다` 상태가 정상이며, 학생 결과는 각 휴대폰 화면에 남습니다. `Ctrl+C`로 종료합니다.

## 비상·로컬 점수 서버와 신뢰된 HTTPS

`server.py`는 정적 파일과 점수 API를 함께 제공합니다. 점수는 서버 메모리에만 있으며,
서버를 다시 시작하면 참가자 수와 순위가 초기화됩니다. 서로 다른 고유 제출 ID는 한
프로세스에서 최대 500개이며, 같은 ID·같은 내용의 재시도는 중복 접수되지 않습니다.
앞뒤 공백을 정리한 표시 이름이 완전히 같으면 점수판에는 최고점 한 건만 남고 동점은
먼저 접수된 결과를 유지합니다. `count`는 고유 표시 이름 수입니다. 상한, 오프라인 또는
제출 실패가 발생해도 학생 화면의 로컬 결과와 재도전은 유지됩니다. 영속 저장·로그인·
외부 서비스 생성은 이 실습 범위에 없습니다.

### 점수 서버를 포함한 HTTP 리허설

```bash
cd docs/presentations/ai-startup-camp-drone/mobile-lab
/home/light/anaconda3/bin/python server.py --host 0.0.0.0 --port 8000
```

```text
학생:    http://<LAN-IP>:8000/
발표자:  http://<LAN-IP>:8000/presenter.html
```

### 센서용 비상 HTTPS 리허설 또는 소규모 직접 제공

행사용으로 신뢰되는 인증서와 개인키가 이미 있다면 둘을 함께 지정합니다. `--cert`와 `--key`는 하나만 지정할 수 없습니다.

```bash
cd docs/presentations/ai-startup-camp-drone/mobile-lab
/home/light/anaconda3/bin/python server.py \
  --host 0.0.0.0 --port 8443 \
  --cert /secure/path/fullchain.pem \
  --key /secure/path/privkey.pem
```

```text
학생:    https://<trusted-name>:8443/
발표자:  https://<trusted-name>:8443/presenter.html
```

휴대폰이 인증서를 신뢰하고 주소의 호스트 이름이 인증서와 일치할 때만 이 주소로 센서 시험을 합니다. 경고가 나는 자체 서명 인증서, IP 주소와 맞지 않는 인증서, HTTP 주소는 iOS/Android 센서 권한 실습의 통과 근거가 아닙니다. HTTP에서도 터치 실습은 가능합니다.

Device Orientation/Motion은 사용자 동작과 secure context를 요구하는 브라우저
기능이므로, 공개 행사 IMU 경로는 신뢰된 HTTPS로 제공해야 합니다. 위 8443 직접
제공은 Oracle/Nginx 운영 경로가 중단된 경우의 제한된 리허설 대안이지 공개 배포
기본값이 아닙니다.

### 정적 HTTPS 호스트에 배포할 때

학교나 행사장의 기존 신뢰 HTTPS 호스트를 사용하면 학생 앱 자체는 정적으로 배포할 수 있습니다. 학생 경로와 발표자 경로가 같은 출처에 있도록 다음과 같이 배치합니다.

```text
https://<trusted-name>/mobile-lab/                 -> mobile-lab/index.html
https://<trusted-name>/mobile-lab/presenter.html   -> mobile-lab/presenter.html
https://<trusted-name>/mobile-lab/src/...          -> mobile-lab/src/...
https://<trusted-name>/mobile-lab/vendor/...       -> mobile-lab/vendor/...
https://<trusted-name>/vendor/uos-slide-template/fonts/... -> sibling font files
```

스타일시트는 `../vendor/uos-slide-template/fonts/`의 기존 발표 번들 글꼴을 참조합니다. 따라서 `mobile-lab/`만 떼어 올리지 말고, 같은 상대 위치의 `vendor/uos-slide-template/fonts/` 세 WOFF2 파일도 함께 제공해야 합니다. 이 정적 구성에는 `/api/scores`가 없으므로 점수판은 선택 기능 상태가 정상입니다. 점수판까지 쓸 경우에는 같은 HTTPS 출처에서 `server.py`를 TLS로 직접 제공하거나, 검토된 역방향 프록시가 `/mobile-lab/` 정적 자산과 `/api/scores`를 같은 출처로 전달하도록 구성합니다.

## QR·학생 진행 순서

1. 발표자 화면 `https://uos-drone.kro.kr/presenter.html`을 열고 `학생 접속 URL`에
   `https://uos-drone.kro.kr/`이 들어 있는지 확인합니다. 비상 리허설에서만
   `https://<trusted-name>:8443/presenter.html`과 그 학생 `/` 주소로 바꿉니다.
2. `QR 갱신`을 눌러 화면 QR이 입력한 URL을 담았는지 확인합니다. `URL 복사`로 채팅·발표 자료용 주소도 복사합니다.
3. 프로젝터에는 발표자 페이지를 띄우고, 학생은 QR을 스캔해 학생 `/` 페이지를 엽니다.
4. 학생 브라우저에는 `익명-XXXXXXXX` 표시 이름이 자동 생성되어 재접속에도 유지됩니다. 학생은 그대로 쓰거나 바꾼 뒤 `센서로 체험 시작` 또는 `터치로 체험`을 고릅니다. 실명·연락처 등 개인정보는 입력하지 않으며, 표시 이름과 점수가 발표자 화면에 공개될 수 있음을 먼저 안내합니다.
5. 센서 경로는 중립 자세에서 `이 자세를 0°로 보정`한 뒤 Roll·Pitch와 AX·AY·AZ를 보고, 20초 호버링을 수행합니다. 터치 경로는 가상 Roll·Pitch 축 관찰과 호버링을 끝까지 수행합니다. 물리 가속도·센서 측정은 제공하지 않으므로 AX·AY·AZ는 `—`로 남습니다.
6. 결과는 먼저 해당 휴대폰에 보입니다. 점수 서버를 켠 경우에만 `선택 점수판에 제출`을 누릅니다. 제출 실패·오프라인이어도 로컬 결과는 지워지지 않습니다.

## 휴대폰 센서와 터치 대체 경로

### iPhone/iPad (Safari)

1. 반드시 신뢰된 `https://` 학생 URL을 Safari에서 엽니다.
2. 학생이 화면의 `센서로 체험 시작`을 직접 누릅니다. 이 사용자 동작에서만 권한 요청이 발생할 수 있습니다.
3. `Motion & Orientation` 관련 Safari/OS 허용 대화상자가 나타나면 실습용 기기에서 허용하고, 첫 기울임 값이 들어오면 중립점을 보정합니다.
4. 권한을 거부했거나 대화상자가 나타나지 않거나 값이 도착하지 않으면 브라우저 설정을 현장에서 강제하지 말고 `터치 체험으로 계속`을 누릅니다. 이 경로도 정식 실습 완료 경로입니다.

### Android (Chrome 등)

1. 신뢰된 `https://` 학생 URL을 Chrome에서 열고 `센서로 체험 시작`을 누릅니다.
2. 브라우저·기기 조합에 따라 별도 권한 대화상자 없이 센서값이 들어올 수 있습니다. 첫 값이 들어오면 중립점을 보정합니다.
3. 센서값이 없거나 사이트/브라우저의 센서 접근이 차단되어 있으면 `터치 체험`으로 전환합니다. Android의 실제 프롬프트 문구와 설정 위치는 기기·브라우저 버전에 따라 다르므로 행사 전 실기기에서 확인합니다.

### 모든 기기 공통

- 세로 화면에서 사용합니다. 화면을 누른 채 원형 조이스틱을 움직이면 좌우는 Roll, 위아래는 Pitch입니다.
- 데스크톱과 센서 없는 기기에서도 터치/포인터 경로로 학습과 도전을 끝낼 수 있습니다. 키보드가 있으면 방향키도 사용할 수 있습니다.
- 권한 허용은 위치·연락처·사진 접근이 아닙니다. 그러나 권한 허용이나 합성 브라우저 테스트가 실제 센서 축·샘플 주기를 검증하는 것은 아닙니다.

## 50명 행사 전 체크리스트

- [ ] 공용 resolver 두 곳에서 `uos-drone.kro.kr` A가 `140.83.83.165`로 보이고, 예전 LAN/private A와 미검증 AAAA가 남지 않았다.
- [ ] OCI의 올바른 VNIC에 연결된 stateful NSG/Security List에서 TCP 80·443 ingress를 확인했고 기존 SSH·tunnel 규칙을 교체하지 않았다.
- [ ] Oracle status 검사에서 current release, Nginx, 선택 backend·loopback API, local-SNI/public-IP HTTPS가 정상이고 public 8000·8443에 앱 listener가 없음을 확인했다.
- [ ] 공개 80/DNS 전환 뒤 Certbot HTTP-01 `renew --dry-run`과 timer를 확인했거나, 현재 인증서 만료일 2026-11-07 전의 명시적 미완료 항목으로 남겼다.
- [ ] 신뢰된 HTTPS 학생 URL과 발표자 URL을 두 대의 실제 iOS 기기와 두 대의 실제 Android 기기에서 각각 열었다.
- [ ] 위 네 기기에서 센서 시작 버튼, 각 플랫폼의 실제 권한/센서 동작, 중립 보정, Roll·Pitch 표기를 직접 확인했다.
- [ ] 네 기기 모두에서 세로 화면이 가로 스크롤·잘린 주요 버튼 없이 읽히고, 안전 영역의 하단 버튼을 누를 수 있다.
- [ ] 센서 권한 거부·미지원·값 미수신 때 터치 조이스틱으로 가상 Roll·Pitch 축 관찰과 20초 도전을 완료했고, 물리 가속도·센서 측정이 없어 AX·AY·AZ가 `—`로 남는 것을 확인했다.
- [ ] QR을 프로젝터/인쇄물의 실제 크기로 띄우고 뒤쪽 좌석 예상 거리에서 스캔해 올바른 HTTPS 학생 URL로 열었다.
- [ ] 행사 Wi-Fi에서 50대가 정적 자산을 동시에 받는 리허설을 했고, QR 접속·터치 실습·오프라인 전환을 관찰했다.
- [ ] 점수판을 쓸 경우 50대 제출 리허설을 했고, 고유 표시 이름 수와 같은 이름의 최고점 한 건만 순위에 남는지 확인했다.
- [ ] 선택 표시 이름에 실명·연락처 등 개인정보를 쓰지 않고, 제출한 표시 이름과 점수가 발표자 화면에 공개될 수 있음을 학생에게 안내했다.
- [ ] 점수판은 클라이언트가 제출한 교육용 비공식 결과이며 실제 비행 성능이나 검증된 측정값이 아니라고 발표자 화면과 진행 멘트로 안내했다.
- [ ] 최대 500개 고유 제출 뒤 새 제출은 거부될 수 있지만 학생 로컬 결과는 유지된다는 운영 경계를 진행자가 알고 있다.
- [ ] 점수 서버 재시작 뒤 순위와 제출 수가 0으로 초기화되는 것을 진행자가 알고, 재시작 시점과 안내 문구를 정했다.
- [ ] 점수 서버 없이도 학생 결과·QR·발표자 화면이 작동하는 오프라인 드릴을 했다.
- [ ] 진행자 노트북 전원, 프로젝터 입력·해상도·가독성, 충전기, Wi-Fi 연결, 예비 QR/URL 안내를 확인했다.
- [ ] 실제 비행 시연이 있다면 학생 실습 구역과 별도의 진행자 전용 비행 구역, 통제 인원, 중지 절차를 마련했다. 학생 QR·터치·센서 입력은 실제 하드웨어에 도달하지 않는다.

## 확인된 자동화와 현장 확인의 경계

자동 검증은 다음 소프트웨어 동작을 다룹니다.

- Node 테스트: 브라우저 익명 ID 생성·재사용·저장소 실패, 권한 분기, 자세 보정·평활화, 조이스틱 벡터, 결정적 호버링·점수, 종료·재시작, 점수 제출 실패 뒤 로컬 결과 보존.
- Python 서버 테스트: 요청 검증, UUID 중복 제출, 같은 이름의 최고점 집계, 정적 파일, 50개 동시 제출 및 50개 중복 재시도.
- Chrome 자동화: 센서 없음·허용·거부의 브라우저 분기, 합성 `DeviceOrientationEvent`와 `DeviceMotionEvent`, 실제 Pointer Event 경로, 점수 API 부재, 발표자 QR, 360×800과 390×844 모바일 뷰포트의 overflow·버튼 잘림.
- 저장소 전체 테스트, 레이아웃 검사, 공백 오류 검사, 금지된 학생 제어 통신 경로 정적 검색.

다음은 자동화만으로 확인하지 않았으며, 행사 전 체크리스트에서 실제로 확인해야 합니다.

- iOS Safari와 Android 브라우저의 실제 물리 센서 축, 권한 프롬프트, 첫 표본 시간, 센서 샘플 주기
- 신뢰 HTTPS 인증서, 행사장 Wi-Fi의 50대 동시 접속 품질, QR 인식 거리, 프로젝터·실제 휴대폰 safe area
- 실제 기체, 펌웨어, 지상국, USB·Bluetooth·serial·UDP 등 하드웨어 경로와 실제 비행 성능

특히 Chrome 테스트의 합성 `DeviceOrientationEvent`는 권한 분기·보정·DOM 표시를 검증할 뿐, 실제 휴대폰 센서 검증이나 비행 검증이 아닙니다.

## 행사 뒤

점수 서버를 사용했다면 서버를 종료하면 메모리 순위는 사라집니다. 결과를 보존해야 한다면 행사 전 별도로 승인된 개인정보·보관 정책과 저장 방식을 마련해야 하며, 이 실습 서버에 임의의 계정·외부 서비스·영속 저장을 추가하지 않습니다.
