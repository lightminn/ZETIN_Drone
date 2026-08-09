# 발표자료 원클릭 실행기 설계

## 목표

`docs/presentations/ai-startup-camp-drone/`에서 스크립트 하나를 실행하면 로컬
HTTP 서버와 Chrome 발표 창이 함께 열리고, 그 Chrome 창을 닫으면 서버도 자동으로
종료되게 한다.

## 선택한 방식

`present.sh`가 임시 Chrome 사용자 프로필을 만들고 `--app=<URL>`로 독립 창을
실행한다. 기존 Chrome 프로필에 URL만 전달하면 명령이 즉시 반환할 수 있어 창
종료를 알 수 없으므로 사용하지 않는다. `xdg-open`도 같은 이유로 제외한다.

스크립트는 다음 순서로 동작한다.

1. 포트, conda Python, Chrome 실행 파일을 검증한다.
2. 발표자료 디렉터리를 루트로 `python -m http.server`를 시작한다.
3. 서버가 응답할 때까지 최대 5초 기다린다.
4. 임시 프로필의 독립 Chrome 앱 창을 열고 프로세스 종료를 기다린다.
5. 창 종료, 브라우저 오류, `Ctrl+C`, `TERM` 모두 같은 정리 경로로 서버와 임시
   프로필을 제거한다.

기본 포트는 8000이고 첫 번째 인자로 다른 포트를 받을 수 있다. 테스트에서는
`PRESENTATION_CHROME_BIN`으로 실제 Chrome만 대체하며 HTTP 서버와 종료 처리는
실제 스크립트를 그대로 실행한다.

## 오류 처리

- 포트가 1~65535의 정수가 아니면 서버를 시작하지 않고 실패한다.
- Python이나 Chrome을 찾지 못하면 이유를 출력하고 실패한다.
- 포트가 이미 사용 중이거나 서버가 5초 안에 준비되지 않으면 서버 로그를
  출력하고 실패한다.
- Chrome이 비정상 종료하면 그 종료 코드를 반환하되 서버와 임시 파일은 정리한다.

## 검증

통합 테스트는 임시 Chrome 대역이 전달받은 URL을 실제로 내려받게 한다. 첫
테스트는 HTML에 `ZETIN Drone`이 있는지와 브라우저 종료 뒤 포트 연결이 거부되는지
확인한다. 둘째 테스트는 브라우저가 실패해도 같은 포트와 임시 자원이 정리되고
브라우저 종료 코드가 보존되는지 확인한다.

## 파일 범위

- 새 파일: `docs/presentations/ai-startup-camp-drone/present.sh`
- 새 파일: `tools/test_presentation_launcher.py`
- 수정: `docs/presentations/ai-startup-camp-drone/README.md`

그 밖의 발표자료와 현재 작업트리의 기존 변경은 수정하지 않는다.
